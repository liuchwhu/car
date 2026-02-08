"""Passthrough behavior — lowest priority, forwards the proposed command unchanged."""

from __future__ import annotations

from pilotnano.behavior.base import Behavior
from pilotnano.bus.messages import CameraFrame, ControlCommand, DepthReport


class PassthroughBehavior(Behavior):
    """Always returns the proposed command. Acts as the fallback behavior."""

    def __init__(self) -> None:
        super().__init__(name="passthrough", priority=100)

    def evaluate(
        self,
        frame: CameraFrame,
        depth_report: DepthReport,
        proposed_cmd: ControlCommand,
    ) -> ControlCommand | None:
        return proposed_cmd
