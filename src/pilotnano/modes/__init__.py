"""Driving mode factory."""

from omegaconf import DictConfig

from pilotnano.bus.base import MessageBus
from pilotnano.hardware.base import Camera, Actuator
from pilotnano.modes.base import DrivingMode


def create_mode(
    name: str,
    cfg: DictConfig,
    bus: MessageBus,
    camera: Camera,
    actuator: Actuator,
    behavior_manager=None,
) -> DrivingMode:
    """Create a driving mode by name."""
    if name == "manual":
        from pilotnano.modes.manual import ManualMode
        return ManualMode(cfg, bus, camera, actuator)
    elif name == "wander":
        from pilotnano.modes.wander import WanderMode
        return WanderMode(cfg, bus, camera, actuator, behavior_manager)
    elif name == "calibrate":
        from pilotnano.calibration.auto_calibrator import AutoCalibrator
        return AutoCalibrator(cfg, bus, camera, actuator)
    else:
        raise ValueError(f"Unknown mode: {name}. Available: manual, wander, calibrate")
