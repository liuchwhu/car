"""Emergency stop behavior — highest priority safety layer."""

from __future__ import annotations

import logging

from pilotnano.behavior.base import Behavior
from pilotnano.bus.messages import CameraFrame, ControlCommand, DepthReport

logger = logging.getLogger(__name__)


class EmergencyStopBehavior(Behavior):
    """Stops the car when anything is dangerously close.

    Uses hysteresis: triggers at `distance`, resumes at `resume_distance`.
    """

    def __init__(self, distance: float = 0.20, resume_distance: float = 0.35) -> None:
        super().__init__(name="emergency_stop", priority=0)
        self._stop_dist = distance
        self._resume_dist = resume_distance
        self._active = False

    def evaluate(
        self,
        frame: CameraFrame,
        depth_report: DepthReport,
        proposed_cmd: ControlCommand,
    ) -> ControlCommand | None:
        if self._active:
            # Stay stopped until clear
            if depth_report.closest_dist > self._resume_dist:
                self._active = False
                logger.info("Emergency stop RELEASED (closest=%.2fm)", depth_report.closest_dist)
                return None
            return ControlCommand(steering=0.0, throttle=0.0)

        if depth_report.closest_dist < self._stop_dist:
            self._active = True
            logger.warning("EMERGENCY STOP — obstacle at %.2fm", depth_report.closest_dist)
            return ControlCommand(steering=0.0, throttle=0.0)

        return None

    def reset(self) -> None:
        self._active = False
