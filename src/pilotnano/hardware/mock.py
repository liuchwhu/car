"""Mock camera and actuator for desktop development and testing."""

import logging
import time

import numpy as np
from omegaconf import DictConfig

from pilotnano.bus.messages import CameraFrame, ControlCommand
from pilotnano.hardware.base import Camera, Actuator

logger = logging.getLogger(__name__)


class MockCamera(Camera):
    """Generates synthetic frames for testing without real hardware."""

    def __init__(self, cfg: DictConfig) -> None:
        self._cfg = cfg
        self._width = cfg.get("width", 640)
        self._height = cfg.get("height", 480)
        self._enable_depth = cfg.get("enable_depth", True)
        self._frame_id = 0
        self._running = False

    def start(self) -> None:
        logger.info("MockCamera started (%dx%d, depth=%s)", self._width, self._height, self._enable_depth)
        self._running = True

    def read(self) -> CameraFrame:
        # Generate a gradient frame with shifting hue based on frame count
        self._frame_id += 1
        hue_shift = (self._frame_id * 2) % 180

        # Create a gradient image
        row = np.linspace(0, 179, self._width, dtype=np.uint8)
        hsv = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        hsv[:, :, 0] = (row[np.newaxis, :] + hue_shift) % 180  # Hue
        hsv[:, :, 1] = 200  # Saturation
        hsv[:, :, 2] = 180  # Value

        import cv2
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # Overlay frame counter
        cv2.putText(rgb, f"MOCK #{self._frame_id}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Generate fake depth map — default 3m everywhere, with a "wall" in the center
        depth = None
        if self._enable_depth:
            depth = np.full((self._height, self._width), 3.0, dtype=np.float32)
            # Simulate a wall in the center getting closer over time
            cx, cy = self._width // 2, self._height // 2
            wall_dist = max(0.3, 3.0 - (self._frame_id % 100) * 0.03)
            depth[cy - 50:cy + 50, cx - 80:cx + 80] = wall_dist

        return CameraFrame(rgb=rgb, depth=depth, frame_id=self._frame_id)

    def stop(self) -> None:
        self._running = False
        logger.info("MockCamera stopped")


class MockActuator(Actuator):
    """Logs commands instead of driving real hardware. Stores history for tests."""

    def __init__(self, cfg: DictConfig) -> None:
        self._cfg = cfg
        self.command_history: list[ControlCommand] = []
        self._running = False

    def start(self) -> None:
        logger.info("MockActuator started")
        self._running = True

    def send(self, command: ControlCommand) -> None:
        self.command_history.append(command)
        logger.debug("MockActuator: steering=%.2f throttle=%.2f", command.steering, command.throttle)

    def stop(self) -> None:
        self.send(ControlCommand(steering=0.0, throttle=0.0))
        self._running = False
        logger.info("MockActuator stopped (neutral sent)")
