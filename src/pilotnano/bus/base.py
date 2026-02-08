"""Message bus abstract base class."""

from abc import ABC, abstractmethod
from typing import Any, Callable


class MessageBus(ABC):
    """Abstract message bus for pub/sub communication."""

    @abstractmethod
    def publish(self, topic: str, message: Any) -> None:
        """Publish a message to a topic."""

    @abstractmethod
    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        """Subscribe a callback to a topic."""

    @abstractmethod
    def unsubscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        """Remove a callback from a topic."""
