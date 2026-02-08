"""Automatic steering calibration — trim finding and steering curve measurement."""

import logging
import time

from omegaconf import DictConfig

from pilotnano.bus.messages import ControlCommand, CameraFrame
from pilotnano.calibration.motion_estimator import MotionEstimator
from pilotnano.hardware.base import Camera, Actuator

logger = logging.getLogger(__name__)


class SteeringCalibrator:
    """Auto-calibrates steering trim and steering-to-turn-rate curve."""

    def __init__(self, cfg: DictConfig, camera: Camera, actuator: Actuator) -> None:
        self._cfg = cfg
        self._camera = camera
        self._actuator = actuator
        self._estimator = MotionEstimator()

    def calibrate_trim(self) -> float:
        """Find the steering trim that produces straight-line driving.

        Drives forward slowly, measures drift via optical flow, adjusts trim.
        Returns the optimal trim value.
        """
        cal_cfg = self._cfg.calibration.steering_trim
        test_throttle = cal_cfg.get("test_throttle", 0.15)
        test_duration = cal_cfg.get("test_duration", 1.0)
        max_iter = cal_cfg.get("max_iterations", 10)
        tolerance = cal_cfg.get("tolerance", 0.02)

        trim = 0.0
        logger.info("Starting steering trim calibration (throttle=%.2f)", test_throttle)

        for iteration in range(max_iter):
            self._estimator.reset()
            drift_samples = []

            # Drive forward for test_duration, measuring drift
            self._actuator.send(ControlCommand(steering=trim, throttle=test_throttle))
            start = time.monotonic()

            while time.monotonic() - start < test_duration:
                frame = self._camera.read()
                drift = self._estimator.measure_drift(frame)
                if drift is not None:
                    drift_samples.append(drift)
                time.sleep(0.05)

            # Stop
            self._actuator.send(ControlCommand(steering=0.0, throttle=0.0))
            time.sleep(0.3)

            if not drift_samples:
                logger.warning("No drift measurements — skipping iteration %d", iteration)
                continue

            avg_drift = sum(drift_samples) / len(drift_samples)
            logger.info("Trim iteration %d: trim=%.4f, avg_drift=%.4f", iteration, trim, avg_drift)

            if abs(avg_drift) < tolerance:
                logger.info("Trim converged at %.4f (drift=%.4f)", trim, avg_drift)
                return trim

            # Adjust trim: if drifting right (positive), steer more left (negative trim)
            trim -= avg_drift * 0.5

        logger.warning("Trim did not converge after %d iterations, using trim=%.4f", max_iter, trim)
        return trim

    def calibrate_curve(self, trim: float) -> dict[float, float]:
        """Measure actual turn rate at each steering value.

        Returns {commanded_steering: measured_turn_rate_normalized}.
        """
        cal_cfg = self._cfg.calibration.steering_curve
        test_throttle = cal_cfg.get("test_throttle", 0.15)
        step_duration = cal_cfg.get("step_duration", 0.5)
        steps = list(cal_cfg.get("steps", [-1.0, -0.6, -0.2, 0.0, 0.2, 0.6, 1.0]))

        curve = {}
        logger.info("Starting steering curve calibration (%d steps)", len(steps))

        for steer_val in steps:
            self._estimator.reset()
            turn_samples = []

            self._actuator.send(ControlCommand(steering=steer_val + trim, throttle=test_throttle))
            start = time.monotonic()

            while time.monotonic() - start < step_duration:
                frame = self._camera.read()
                turn = self._estimator.measure_turn_rate(frame)
                if turn is not None:
                    turn_samples.append(turn)
                time.sleep(0.05)

            # Stop between steps
            self._actuator.send(ControlCommand(steering=0.0, throttle=0.0))
            time.sleep(0.3)

            if turn_samples:
                avg_turn = sum(turn_samples) / len(turn_samples)
                curve[steer_val] = avg_turn
                logger.info("  steering=%.2f → turn_rate=%.4f", steer_val, avg_turn)
            else:
                logger.warning("  steering=%.2f → no measurements", steer_val)

        return curve

    def discover_limits(self) -> tuple[float, float]:
        """Discover physical steering limits by sweeping to extremes.

        Returns (min_angle, max_angle) in normalized steering units.
        For now, uses the config defaults since limit detection via
        optical flow stall requires careful tuning per vehicle.
        """
        min_angle = self._cfg.hardware.actuator.get("steering_min_angle", 30)
        max_angle = self._cfg.hardware.actuator.get("steering_max_angle", 150)
        logger.info("Using steering limits from config: %d° to %d°", min_angle, max_angle)
        return float(min_angle), float(max_angle)
