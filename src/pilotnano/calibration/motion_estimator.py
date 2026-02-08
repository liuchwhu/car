"""Motion estimation using D435 depth deltas and optical flow."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

from pilotnano.bus.messages import CameraFrame

logger = logging.getLogger(__name__)


class MotionEstimator:
    """Measures actual vehicle motion using the D435 camera.

    Two measurement methods:
    - Speed from depth: rate of change of center depth between frames
    - Turn rate from optical flow: dominant horizontal flow indicates yaw rate
    """

    def __init__(self) -> None:
        self._prev_gray: np.ndarray | None = None
        self._prev_depth_center: float | None = None
        self._prev_time: float | None = None

        # Lucas-Kanade optical flow params
        self._lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        # Feature detection params
        self._feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7,
        )

    def reset(self) -> None:
        """Reset state for a new measurement sequence."""
        self._prev_gray = None
        self._prev_depth_center = None
        self._prev_time = None

    def measure_speed_from_depth(self, frame: CameraFrame) -> float | None:
        """Measure forward/backward speed using center depth change.

        Returns speed in m/s (positive = approaching wall, negative = moving away).
        Returns None if not enough data yet.
        """
        if frame.depth is None:
            return None

        h, w = frame.depth.shape
        # Sample center region
        roi = frame.depth[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
        valid = roi[(roi > 0.1) & (roi < 10.0)]
        if len(valid) == 0:
            return None

        center_depth = float(np.median(valid))
        now = time.monotonic()

        if self._prev_depth_center is None or self._prev_time is None:
            self._prev_depth_center = center_depth
            self._prev_time = now
            return None

        dt = now - self._prev_time
        if dt < 0.01:
            return None

        # Depth decreasing = moving toward wall = positive speed
        speed = -(center_depth - self._prev_depth_center) / dt

        self._prev_depth_center = center_depth
        self._prev_time = now
        return speed

    def measure_turn_rate(self, frame: CameraFrame) -> float | None:
        """Measure yaw rate from optical flow.

        Returns normalized horizontal flow (positive = scene moving left = car turning right).
        Returns None if not enough data.
        """
        gray = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2GRAY)

        if self._prev_gray is None:
            self._prev_gray = gray
            return None

        # Find features in previous frame
        prev_pts = cv2.goodFeaturesToTrack(self._prev_gray, mask=None, **self._feature_params)
        if prev_pts is None or len(prev_pts) < 10:
            self._prev_gray = gray
            return None

        # Calculate optical flow
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, prev_pts, None, **self._lk_params
        )

        if next_pts is None:
            self._prev_gray = gray
            return None

        # Select good matches
        good_mask = status.ravel() == 1
        if good_mask.sum() < 5:
            self._prev_gray = gray
            return None

        prev_good = prev_pts[good_mask]
        next_good = next_pts[good_mask]

        # Compute horizontal flow
        dx = next_good[:, 0, 0] - prev_good[:, 0, 0]
        # Normalize by image width
        h, w = gray.shape
        mean_dx = float(np.median(dx)) / w

        self._prev_gray = gray
        return mean_dx

    def measure_drift(self, frame: CameraFrame) -> float | None:
        """Measure lateral drift — useful for steering trim calibration.

        Positive = drifting right, negative = drifting left.
        """
        return self.measure_turn_rate(frame)
