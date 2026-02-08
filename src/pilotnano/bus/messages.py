"""Message dataclasses and topic constants."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import time

import numpy as np

# Topic constants (ROS2-style naming for future compatibility)
TOPIC_CAMERA_FRAME = "camera/frame"
TOPIC_CONTROL_CMD = "control/command"
TOPIC_DEPTH_REPORT = "perception/depth_report"
TOPIC_BEHAVIOR_ACTIVE = "behavior/active"
TOPIC_DETECTION = "perception/detection"


@dataclass
class CameraFrame:
    """A single camera frame with optional depth."""
    rgb: np.ndarray                     # HxWx3 uint8 BGR
    depth: np.ndarray | None = None     # HxW float32 meters (None if depth disabled)
    timestamp: float = field(default_factory=time.time)
    frame_id: int = 0


@dataclass
class ControlCommand:
    """Steering and throttle command, both normalized to [-1, 1]."""
    steering: float = 0.0   # -1 = full left, +1 = full right
    throttle: float = 0.0   # -1 = full reverse, +1 = full forward
    timestamp: float = field(default_factory=time.time)


@dataclass
class DepthReport:
    """Processed depth information by zone."""
    left_dist: float = float("inf")
    center_dist: float = float("inf")
    right_dist: float = float("inf")
    closest_dist: float = float("inf")
    timestamp: float = field(default_factory=time.time)


@dataclass
class Detection:
    """A single object detection result."""
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1
