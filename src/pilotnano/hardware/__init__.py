"""Hardware factory functions."""

from omegaconf import DictConfig

from pilotnano.hardware.base import Camera, Actuator


def create_camera(cfg: DictConfig) -> Camera:
    """Create a camera instance based on config."""
    cam_type = cfg.camera.type
    if cam_type == "realsense":
        from pilotnano.hardware.realsense import RealSenseCamera
        return RealSenseCamera(cfg.camera)
    elif cam_type == "mock":
        from pilotnano.hardware.mock import MockCamera
        return MockCamera(cfg.camera)
    else:
        raise ValueError(f"Unknown camera type: {cam_type}")


def create_actuator(cfg: DictConfig) -> Actuator:
    """Create an actuator instance based on config."""
    act_type = cfg.actuator.type
    if act_type == "pca9685":
        from pilotnano.hardware.pca9685 import PCA9685Actuator
        return PCA9685Actuator(cfg.actuator)
    elif act_type == "mock":
        from pilotnano.hardware.mock import MockActuator
        return MockActuator(cfg.actuator)
    else:
        raise ValueError(f"Unknown actuator type: {act_type}")
