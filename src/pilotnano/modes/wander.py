"""Wander mode — autonomous wandering with obstacle avoidance."""

import logging

import cv2
from omegaconf import DictConfig

from pilotnano.behavior.manager import BehaviorManager
from pilotnano.bus.base import MessageBus
from pilotnano.bus.messages import ControlCommand, TOPIC_CAMERA_FRAME, TOPIC_CONTROL_CMD
from pilotnano.hardware.base import Camera, Actuator
from pilotnano.modes.base import DrivingMode

logger = logging.getLogger(__name__)


class WanderMode(DrivingMode):
    """Drive forward and let the behavior stack handle everything else.

    The mode itself just says "go straight at wander_throttle".
    Emergency stop, obstacle avoidance, and recovery behaviors
    make it safe and autonomous.
    """

    def __init__(
        self,
        cfg: DictConfig,
        bus: MessageBus,
        camera: Camera,
        actuator: Actuator,
        behavior_manager: BehaviorManager,
    ) -> None:
        super().__init__(cfg, bus, camera, actuator)
        self._behavior_manager = behavior_manager
        self._wander_throttle = cfg.wander.get("throttle", 0.3)

    def start(self) -> None:
        super().start()
        logger.info("WanderMode started (throttle=%.2f) — press 'q' to quit", self._wander_throttle)

    def step(self) -> None:
        frame = self.camera.read()
        self.bus.publish(TOPIC_CAMERA_FRAME, frame)

        # Propose: go straight forward
        proposed = ControlCommand(steering=0.0, throttle=self._wander_throttle)

        # Let behavior manager decide the actual command
        command = self._behavior_manager.process(frame, proposed)
        self.bus.publish(TOPIC_CONTROL_CMD, command)

        self.actuator.send(command)

        # Show camera feed with HUD
        display = frame.rgb.copy()
        cv2.putText(display, f"S:{command.steering:+.2f} T:{command.throttle:+.2f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("PilotNano — Wander", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.running = False

    def stop(self) -> None:
        self.actuator.send(ControlCommand(steering=0.0, throttle=0.0))
        cv2.destroyAllWindows()
        super().stop()
        logger.info("WanderMode stopped")
