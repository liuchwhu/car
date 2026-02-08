"""PCA9685 servo/ESC driver for Jetson Nano."""

import logging
from pathlib import Path

import numpy as np
from omegaconf import DictConfig, OmegaConf

from pilotnano.bus.messages import ControlCommand
from pilotnano.hardware.base import Actuator

logger = logging.getLogger(__name__)


class PCA9685Actuator(Actuator):
    """Controls steering servo and ESC via PCA9685 I2C PWM driver.

    Uses Adafruit ServoKit. Steering is position-controlled (angle),
    ESC is continuous-servo mode (throttle).
    """

    def __init__(self, cfg: DictConfig) -> None:
        self._cfg = cfg
        self._kit = None
        self._steering_ch = cfg.get("steering_channel", 0)
        self._throttle_ch = cfg.get("throttle_channel", 1)
        self._steer_min = cfg.get("steering_min_angle", 30)
        self._steer_max = cfg.get("steering_max_angle", 150)
        self._steer_center = cfg.get("steering_center_angle", 90)
        self._steer_trim = cfg.get("steering_trim", 0.0)
        self._throttle_max = cfg.get("throttle_max", 0.4)

        # Calibration overrides (loaded from calibration_result.yaml if it exists)
        self._cal = None
        self._load_calibration()

    def _load_calibration(self) -> None:
        """Load saved calibration constants if available."""
        cal_path = Path(__file__).resolve().parents[2] / "configs" / "calibration" / "calibration_result.yaml"
        if cal_path.exists():
            try:
                self._cal = OmegaConf.load(cal_path)
                # Override trim and limits from calibration
                if "steering_trim" in self._cal:
                    self._steer_trim = self._cal.steering_trim
                if "steering_limits" in self._cal:
                    self._steer_min = self._cal.steering_limits.get("min_angle", self._steer_min)
                    self._steer_max = self._cal.steering_limits.get("max_angle", self._steer_max)
                logger.info("Loaded calibration from %s (timestamp: %s)",
                            cal_path, self._cal.get("timestamp", "unknown"))
            except Exception:
                logger.warning("Failed to load calibration file %s, using defaults", cal_path)

    def start(self) -> None:
        import board
        import busio
        from adafruit_servokit import ServoKit

        i2c_bus = self._cfg.get("i2c_bus", 0)
        if i2c_bus == 0:
            i2c = busio.I2C(board.SCL_1, board.SDA_1)
        else:
            i2c = busio.I2C(board.SCL, board.SDA)

        self._kit = ServoKit(channels=16, i2c=i2c)

        # Configure servo pulse width range if specified
        min_pulse = self._cfg.get("steering_min_pulse", 1000)
        max_pulse = self._cfg.get("steering_max_pulse", 2000)
        self._kit.servo[self._steering_ch].set_pulse_width_range(min_pulse, max_pulse)

        logger.info("PCA9685Actuator started (bus=%d, steering=ch%d [%d°-%d°], throttle=ch%d, trim=%.3f)",
                     i2c_bus, self._steering_ch, self._steer_min, self._steer_max,
                     self._throttle_ch, self._steer_trim)

    def send(self, command: ControlCommand) -> None:
        if self._kit is None:
            return

        # Map normalized steering [-1, 1] to servo angle, applying trim
        steer_range = (self._steer_max - self._steer_min) / 2.0
        angle = self._steer_center + (command.steering + self._steer_trim) * steer_range
        # Clamp to physical limits
        angle = max(self._steer_min, min(self._steer_max, angle))
        self._kit.servo[self._steering_ch].angle = angle

        # Map throttle [-1, 1] with safety cap
        throttle = max(-self._throttle_max, min(self._throttle_max, command.throttle))
        self._kit.continuous_servo[self._throttle_ch].throttle = throttle

    def stop(self) -> None:
        if self._kit is not None:
            # Send neutral
            self._kit.servo[self._steering_ch].angle = self._steer_center
            self._kit.continuous_servo[self._throttle_ch].throttle = 0.0
            logger.info("PCA9685Actuator stopped (neutral sent)")
