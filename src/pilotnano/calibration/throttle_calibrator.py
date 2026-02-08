"""Automatic throttle-to-speed calibration using depth measurements."""

import logging
import time

from omegaconf import DictConfig

from pilotnano.bus.messages import ControlCommand
from pilotnano.calibration.motion_estimator import MotionEstimator
from pilotnano.hardware.base import Camera, Actuator

logger = logging.getLogger(__name__)


class ThrottleCalibrator:
    """Measures actual speed at each throttle value using depth deltas."""

    def __init__(self, cfg: DictConfig, camera: Camera, actuator: Actuator) -> None:
        self._cfg = cfg
        self._camera = camera
        self._actuator = actuator
        self._estimator = MotionEstimator()

    def calibrate(self) -> dict:
        """Measure speed at each throttle step, find dead zone.

        Returns dict with 'curve' (throttle→speed mapping) and 'dead_zone' (min moving throttle).
        """
        cal_cfg = self._cfg.calibration.throttle_curve
        test_duration = cal_cfg.get("test_duration", 1.0)
        steps = list(cal_cfg.get("steps", [0.1, 0.15, 0.2, 0.25, 0.3, 0.4]))

        curve = {}
        dead_zone = 0.0
        found_movement = False

        logger.info("Starting throttle calibration (%d steps, facing wall)", len(steps))

        for throttle_val in steps:
            self._estimator.reset()
            speed_samples = []

            # Drive forward at this throttle
            self._actuator.send(ControlCommand(steering=0.0, throttle=throttle_val))
            start = time.monotonic()

            while time.monotonic() - start < test_duration:
                frame = self._camera.read()
                speed = self._estimator.measure_speed_from_depth(frame)
                if speed is not None:
                    speed_samples.append(speed)
                time.sleep(0.05)

            # Stop and reverse briefly to return to position
            self._actuator.send(ControlCommand(steering=0.0, throttle=0.0))
            time.sleep(0.5)

            if speed_samples:
                avg_speed = sum(speed_samples) / len(speed_samples)
                curve[throttle_val] = avg_speed

                if avg_speed > 0.05 and not found_movement:
                    dead_zone = throttle_val
                    found_movement = True

                logger.info("  throttle=%.2f → speed=%.3f m/s", throttle_val, avg_speed)
            else:
                logger.warning("  throttle=%.2f → no measurements", throttle_val)

            # Reverse to starting position
            if speed_samples and sum(speed_samples) / len(speed_samples) > 0.05:
                self._actuator.send(ControlCommand(steering=0.0, throttle=-throttle_val * 0.5))
                time.sleep(test_duration * 0.8)
                self._actuator.send(ControlCommand(steering=0.0, throttle=0.0))
                time.sleep(0.5)

        logger.info("Throttle dead zone: %.2f", dead_zone)
        return {"curve": curve, "dead_zone": dead_zone}
