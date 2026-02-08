"""Abstract base classes for camera and actuator hardware."""

from abc import ABC, abstractmethod

from pilotnano.bus.messages import CameraFrame, ControlCommand


class Camera(ABC):
    """Abstract camera interface."""

    @abstractmethod
    def start(self) -> None:
        """Initialize and start the camera."""

    @abstractmethod
    def read(self) -> CameraFrame:
        """Read a single frame (blocks until available)."""

    @abstractmethod
    def stop(self) -> None:
        """Stop and release camera resources."""

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


class Actuator(ABC):
    """Abstract actuator interface for steering + throttle."""

    @abstractmethod
    def start(self) -> None:
        """Initialize the actuator hardware."""

    @abstractmethod
    def send(self, command: ControlCommand) -> None:
        """Send a control command to the actuator."""

    @abstractmethod
    def stop(self) -> None:
        """Send neutral command and release resources."""

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
