"""Recovery behavior — gets the car unstuck when forward progress stalls."""

from __future__ import annotations

import enum
import logging
import time

from pilotnano.behavior.base import Behavior
from pilotnano.bus.messages import CameraFrame, ControlCommand, DepthReport

logger = logging.getLogger(__name__)


class _State(enum.Enum):
    MONITORING = "monitoring"
    REVERSING = "reversing"
    TURNING = "turning"
    RESUMING = "resuming"


class RecoveryBehavior(Behavior):
    """Detects when the car is stuck and performs a reverse+turn maneuver."""

    def __init__(
        self,
        stuck_timeout: float = 1.0,
        depth_change_threshold: float = 0.05,
        reverse_duration: float = 1.0,
        reverse_throttle: float = -0.3,
        turn_duration: float = 0.5,
        turn_throttle: float = 0.2,
    ) -> None:
        super().__init__(name="recovery", priority=1)
        self._stuck_timeout = stuck_timeout
        self._depth_threshold = depth_change_threshold
        self._reverse_duration = reverse_duration
        self._reverse_throttle = reverse_throttle
        self._turn_duration = turn_duration
        self._turn_throttle = turn_throttle

        self._state = _State.MONITORING
        self._prev_center_dist: float | None = None
        self._stuck_start: float | None = None
        self._maneuver_start: float = 0.0
        self._turn_direction: float = 1.0

    def evaluate(
        self,
        frame: CameraFrame,
        depth_report: DepthReport,
        proposed_cmd: ControlCommand,
    ) -> ControlCommand | None:
        now = time.monotonic()

        if self._state == _State.MONITORING:
            return self._monitor(depth_report, proposed_cmd, now)
        elif self._state == _State.REVERSING:
            return self._reverse(now)
        elif self._state == _State.TURNING:
            return self._turn(now)
        elif self._state == _State.RESUMING:
            self._state = _State.MONITORING
            self._stuck_start = None
            self._prev_center_dist = None
            logger.info("Recovery complete, resuming normal control")
            return None

        return None

    def _monitor(
        self, depth_report: DepthReport, proposed_cmd: ControlCommand, now: float
    ) -> ControlCommand | None:
        # Only check for stuck when we're trying to move forward
        if proposed_cmd.throttle <= 0:
            self._stuck_start = None
            self._prev_center_dist = None
            return None

        center = depth_report.center_dist
        if self._prev_center_dist is not None:
            depth_change = abs(center - self._prev_center_dist)
            if depth_change < self._depth_threshold:
                # Not making progress
                if self._stuck_start is None:
                    self._stuck_start = now
                elif now - self._stuck_start > self._stuck_timeout:
                    # Stuck! Begin recovery
                    logger.warning("STUCK detected (%.1fs no progress), starting recovery",
                                   now - self._stuck_start)
                    self._turn_direction = 1.0 if depth_report.right_dist > depth_report.left_dist else -1.0
                    self._state = _State.REVERSING
                    self._maneuver_start = now
                    return ControlCommand(steering=0.0, throttle=self._reverse_throttle)
            else:
                self._stuck_start = None

        self._prev_center_dist = center
        return None

    def _reverse(self, now: float) -> ControlCommand:
        if now - self._maneuver_start > self._reverse_duration:
            logger.info("Recovery: reversing done, turning %s",
                        "right" if self._turn_direction > 0 else "left")
            self._state = _State.TURNING
            self._maneuver_start = now
            return ControlCommand(steering=self._turn_direction, throttle=self._turn_throttle)
        return ControlCommand(steering=0.0, throttle=self._reverse_throttle)

    def _turn(self, now: float) -> ControlCommand:
        if now - self._maneuver_start > self._turn_duration:
            self._state = _State.RESUMING
            return ControlCommand(steering=0.0, throttle=0.0)
        return ControlCommand(steering=self._turn_direction, throttle=self._turn_throttle)

    def reset(self) -> None:
        self._state = _State.MONITORING
        self._prev_center_dist = None
        self._stuck_start = None
