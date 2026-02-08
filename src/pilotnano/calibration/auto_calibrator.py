"""Auto-calibrator — orchestrates all calibration routines."""

import datetime
import json
import logging
from pathlib import Path

from omegaconf import DictConfig, OmegaConf

from pilotnano.bus.base import MessageBus
from pilotnano.calibration.steering_calibrator import SteeringCalibrator
from pilotnano.calibration.throttle_calibrator import ThrottleCalibrator
from pilotnano.hardware.base import Camera, Actuator
from pilotnano.modes.base import DrivingMode

logger = logging.getLogger(__name__)

CONFIGS_DIR = Path(__file__).resolve().parents[2] / "configs"


class AutoCalibrator(DrivingMode):
    """Orchestrates all auto-calibration routines.

    Runs as a DrivingMode so it can be invoked via `pilotnano --mode calibrate`.
    Performs calibration in a single pass through step(), then sets running=False.
    """

    def __init__(
        self,
        cfg: DictConfig,
        bus: MessageBus,
        camera: Camera,
        actuator: Actuator,
        force: bool = False,
    ) -> None:
        super().__init__(cfg, bus, camera, actuator)
        self._force = force
        self._done = False

    def start(self) -> None:
        super().start()
        logger.info("AutoCalibrator started")

    def step(self) -> None:
        if self._done:
            self.running = False
            return

        result_path = CONFIGS_DIR / "calibration" / "calibration_result.yaml"

        # Check for existing calibration
        if result_path.exists() and not self._force:
            existing = OmegaConf.load(result_path)
            ts = existing.get("timestamp", "unknown")
            logger.info("Using saved calibration from %s. Use --force-calibrate to re-run.", ts)
            self._done = True
            self.running = False
            return

        logger.info("=== Starting Auto-Calibration ===")

        # 1. Steering calibration
        steer_cal = SteeringCalibrator(self.cfg, self.camera, self.actuator)

        logger.info("--- Step 1/4: Steering Limit Discovery ---")
        min_angle, max_angle = steer_cal.discover_limits()

        logger.info("--- Step 2/4: Steering Trim Calibration ---")
        trim = steer_cal.calibrate_trim()

        logger.info("--- Step 3/4: Steering Curve Calibration ---")
        steering_curve = steer_cal.calibrate_curve(trim)

        # 2. Throttle calibration
        throttle_cal = ThrottleCalibrator(self.cfg, self.camera, self.actuator)

        logger.info("--- Step 4/4: Throttle Speed Calibration ---")
        throttle_result = throttle_cal.calibrate()

        # 3. Save results
        result = OmegaConf.create({
            "timestamp": datetime.datetime.now().isoformat(),
            "steering_trim": trim,
            "steering_limits": {
                "min_angle": min_angle,
                "max_angle": max_angle,
            },
            "steering_curve": {str(k): v for k, v in steering_curve.items()},
            "throttle_curve": {str(k): v for k, v in throttle_result["curve"].items()},
            "throttle_dead_zone": throttle_result["dead_zone"],
        })

        result_path.parent.mkdir(parents=True, exist_ok=True)
        OmegaConf.save(result, result_path)
        logger.info("Calibration saved to %s", result_path)

        # 4. Write human-readable report
        report_path = CONFIGS_DIR / "calibration" / "calibration_report.json"
        report = OmegaConf.to_container(result, resolve=True)
        report["status"] = "pass"
        with open(report_path, "w") as f:
            json.dump(report, f, indent=2)
        logger.info("Report written to %s", report_path)

        logger.info("=== Auto-Calibration Complete ===")
        self._done = True
        self.running = False

    def stop(self) -> None:
        # Ensure neutral on exit
        from pilotnano.bus.messages import ControlCommand
        self.actuator.send(ControlCommand(steering=0.0, throttle=0.0))
        super().stop()
        logger.info("AutoCalibrator stopped")
