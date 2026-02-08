"""Manual mode — display camera feed, car driven by RC remote."""

import logging

import cv2
from omegaconf import DictConfig

from pilotnano.bus.base import MessageBus
from pilotnano.bus.messages import TOPIC_CAMERA_FRAME
from pilotnano.hardware.base import Camera, Actuator
from pilotnano.modes.base import DrivingMode

logger = logging.getLogger(__name__)


class ManualMode(DrivingMode):
    """Displays the camera feed. Steering/throttle come from the RC remote directly."""

    def __init__(self, cfg: DictConfig, bus: MessageBus, camera: Camera, actuator: Actuator):
        super().__init__(cfg, bus, camera, actuator)

    def start(self) -> None:
        super().start()
        logger.info("ManualMode started — press 'q' to quit")

    def step(self) -> None:
        frame = self.camera.read()
        self.bus.publish(TOPIC_CAMERA_FRAME, frame)

        cv2.imshow("PilotNano — Manual", frame.rgb)

        if frame.depth is not None:
            # Normalize depth for visualization (0-5m range)
            depth_vis = (frame.depth / 5.0 * 255).clip(0, 255).astype("uint8")
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("PilotNano — Depth", depth_colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.running = False

    def stop(self) -> None:
        cv2.destroyAllWindows()
        super().stop()
        logger.info("ManualMode stopped")
