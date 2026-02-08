"""Depth frame analysis — converts raw depth into actionable zone distances."""

import logging

import numpy as np
from omegaconf import DictConfig

from pilotnano.bus.messages import CameraFrame, DepthReport

logger = logging.getLogger(__name__)


class DepthAnalyzer:
    """Processes depth frames into left/center/right zone distances."""

    def __init__(self, cfg: DictConfig) -> None:
        da_cfg = cfg.get("depth_analyzer", {})
        self._min_depth = da_cfg.get("min_valid_depth", 0.1)
        self._max_depth = da_cfg.get("max_valid_depth", 10.0)
        self._roi_top = da_cfg.get("roi_top", 0.3)
        self._roi_bottom = da_cfg.get("roi_bottom", 1.0)

    def analyze(self, frame: CameraFrame) -> DepthReport:
        """Analyze a depth frame and return zone distances.

        Divides the depth image into left/center/right thirds (within the ROI),
        computes the minimum reliable distance in each zone.
        """
        if frame.depth is None:
            return DepthReport()

        h, w = frame.depth.shape
        top = int(h * self._roi_top)
        bottom = int(h * self._roi_bottom)
        roi = frame.depth[top:bottom, :]

        third = w // 3
        left_zone = roi[:, :third]
        center_zone = roi[:, third:2 * third]
        right_zone = roi[:, 2 * third:]

        left_dist = self._min_distance(left_zone)
        center_dist = self._min_distance(center_zone)
        right_dist = self._min_distance(right_zone)
        closest_dist = min(left_dist, center_dist, right_dist)

        return DepthReport(
            left_dist=left_dist,
            center_dist=center_dist,
            right_dist=right_dist,
            closest_dist=closest_dist,
        )

    def _min_distance(self, zone: np.ndarray) -> float:
        """Compute minimum valid distance in a depth zone."""
        valid = zone[(zone > self._min_depth) & (zone < self._max_depth)]
        if len(valid) == 0:
            return float("inf")
        # Use 5th percentile instead of absolute min to filter noise
        return float(np.percentile(valid, 5))
