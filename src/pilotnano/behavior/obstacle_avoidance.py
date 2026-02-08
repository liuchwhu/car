"""Obstacle avoidance behavior — steers toward open space."""

from __future__ import annotations

import logging

from pilotnano.behavior.base import Behavior
from pilotnano.bus.messages import CameraFrame, ControlCommand, DepthReport

logger = logging.getLogger(__name__)


class ObstacleAvoidanceBehavior(Behavior):
    """Steers away from obstacles when they're within trigger distance.

    Priority 2 — after emergency stop and recovery, before passthrough.
    """

    def __init__(self, trigger_distance: float = 1.0, steer_gain: float = 0.8) -> None:
        super().__init__(name="obstacle_avoidance", priority=2)
        self._trigger_dist = trigger_distance
        self._steer_gain = steer_gain

    def evaluate(
        self,
        frame: CameraFrame,
        depth_report: DepthReport,
        proposed_cmd: ControlCommand,
    ) -> ControlCommand | None:
        # Only intervene when center obstacle is within trigger distance
        if depth_report.center_dist > self._trigger_dist:
            return None

        left = depth_report.left_dist
        right = depth_report.right_dist

        # If both sides are blocked, let recovery handle it
        if left < self._trigger_dist * 0.5 and right < self._trigger_dist * 0.5:
            return None

        # Steer toward the side with more clearance
        if right > left:
            steering = self._steer_gain  # Turn right (positive)
        else:
            steering = -self._steer_gain  # Turn left (negative)

        # Scale steering by how close the obstacle is (closer = harder turn)
        proximity = 1.0 - (depth_report.center_dist / self._trigger_dist)
        steering *= proximity

        # Reduce throttle proportionally to proximity
        throttle = proposed_cmd.throttle * (1.0 - proximity * 0.7)

        logger.debug("ObstacleAvoidance: center=%.2fm, steer=%.2f, throttle=%.2f",
                      depth_report.center_dist, steering, throttle)

        return ControlCommand(steering=steering, throttle=throttle)
