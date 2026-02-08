"""Synchronous in-process message bus."""

import logging
from collections import defaultdict
from typing import Any, Callable

from pilotnano.bus.base import MessageBus

logger = logging.getLogger(__name__)


class LocalBus(MessageBus):
    """Synchronous local pub/sub bus.

    Callbacks are invoked in the caller's thread when publish() is called.
    Suitable for single-threaded main loops (20Hz is well within budget).
    """

    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callable]] = defaultdict(list)

    def publish(self, topic: str, message: Any) -> None:
        for callback in self._subscribers[topic]:
            try:
                callback(message)
            except Exception:
                logger.exception("Error in subscriber for topic %s", topic)

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        if callback not in self._subscribers[topic]:
            self._subscribers[topic].append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        try:
            self._subscribers[topic].remove(callback)
        except ValueError:
            pass
