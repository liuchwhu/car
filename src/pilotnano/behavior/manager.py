"""Behavior manager — priority-based behavior arbitration."""

import logging

from omegaconf import DictConfig

from pilotnano.behavior.base import Behavior
from pilotnano.behavior.emergency_stop import EmergencyStopBehavior
from pilotnano.behavior.obstacle_avoidance import ObstacleAvoidanceBehavior
from pilotnano.behavior.passthrough import PassthroughBehavior
from pilotnano.behavior.recovery import RecoveryBehavior
from pilotnano.bus.base import MessageBus
from pilotnano.bus.messages import (
    CameraFrame, ControlCommand, DepthReport,
    TOPIC_BEHAVIOR_ACTIVE, TOPIC_DEPTH_REPORT,
)
from pilotnano.perception.depth_analyzer import DepthAnalyzer

logger = logging.getLogger(__name__)


class BehaviorManager:
    """Orchestrates the behavior priority stack.

    On each tick, runs the DepthAnalyzer, then iterates through behaviors
    in priority order. The first behavior to return a non-None command wins.
    """

    def __init__(self, cfg: DictConfig, bus: MessageBus) -> None:
        self._bus = bus
        self._depth_analyzer = DepthAnalyzer(cfg.behavior)

        # Build behavior stack from config
        bcfg = cfg.behavior
        self._behaviors: list[Behavior] = sorted([
            EmergencyStopBehavior(
                distance=bcfg.emergency_stop.get("distance", 0.20),
                resume_distance=bcfg.emergency_stop.get("resume_distance", 0.35),
            ),
            RecoveryBehavior(
                stuck_timeout=bcfg.recovery.get("stuck_timeout", 1.0),
                depth_change_threshold=bcfg.recovery.get("depth_change_threshold", 0.05),
                reverse_duration=bcfg.recovery.get("reverse_duration", 1.0),
                reverse_throttle=bcfg.recovery.get("reverse_throttle", -0.3),
                turn_duration=bcfg.recovery.get("turn_duration", 0.5),
                turn_throttle=bcfg.recovery.get("turn_throttle", 0.2),
            ),
            ObstacleAvoidanceBehavior(
                trigger_distance=bcfg.obstacle_avoidance.get("trigger_distance", 1.0),
                steer_gain=bcfg.obstacle_avoidance.get("steer_gain", 0.8),
            ),
            PassthroughBehavior(),
        ], key=lambda b: b.priority)

        self._active_behavior: str = "none"

    def process(self, frame: CameraFrame, proposed_cmd: ControlCommand) -> ControlCommand:
        """Run the behavior stack and return the winning command."""
        depth_report = self._depth_analyzer.analyze(frame)
        self._bus.publish(TOPIC_DEPTH_REPORT, depth_report)

        for behavior in self._behaviors:
            result = behavior.evaluate(frame, depth_report, proposed_cmd)
            if result is not None:
                if behavior.name != self._active_behavior:
                    self._active_behavior = behavior.name
                    if behavior.name != "passthrough":
                        logger.info("Behavior active: %s", behavior.name)
                    self._bus.publish(TOPIC_BEHAVIOR_ACTIVE, behavior.name)
                return result

        # Should never reach here (passthrough always returns)
        return proposed_cmd

    def reset(self) -> None:
        """Reset all behaviors."""
        for behavior in self._behaviors:
            behavior.reset()
        self._active_behavior = "none"
