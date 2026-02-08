"""Behavior abstract base class."""

from __future__ import annotations

from abc import ABC, abstractmethod

from pilotnano.bus.messages import CameraFrame, ControlCommand, DepthReport


class Behavior(ABC):
    """Base class for all behaviors in the priority stack.

    Each behavior evaluates the current situation and either:
    - Returns a ControlCommand to override the proposed action
    - Returns None to abstain (let lower-priority behaviors decide)
    """

    def __init__(self, name: str, priority: int) -> None:
        self.name = name
        self.priority = priority  # Lower number = higher priority

    @abstractmethod
    def evaluate(
        self,
        frame: CameraFrame,
        depth_report: DepthReport,
        proposed_cmd: ControlCommand,
    ) -> ControlCommand | None:
        """Evaluate whether to intervene.

        Returns a ControlCommand to take over, or None to pass.
        """

    def reset(self) -> None:
        """Reset any internal state (called when mode changes)."""
