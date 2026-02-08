"""Driving mode abstract base class."""

from abc import ABC, abstractmethod

from omegaconf import DictConfig

from pilotnano.bus.base import MessageBus
from pilotnano.hardware.base import Camera, Actuator


class DrivingMode(ABC):
    """Base class for all driving modes (manual, wander, follow, etc.)."""

    def __init__(
        self,
        cfg: DictConfig,
        bus: MessageBus,
        camera: Camera,
        actuator: Actuator,
    ) -> None:
        self.cfg = cfg
        self.bus = bus
        self.camera = camera
        self.actuator = actuator
        self.running = False

    def start(self) -> None:
        """Called once before the main loop begins."""
        self.running = True

    @abstractmethod
    def step(self) -> None:
        """Called once per main loop tick."""

    def stop(self) -> None:
        """Called once when the mode is ending."""
        self.running = False
