"""Intel RealSense D435 camera driver."""

import logging

import numpy as np
from omegaconf import DictConfig

from pilotnano.bus.messages import CameraFrame
from pilotnano.hardware.base import Camera

logger = logging.getLogger(__name__)


class RealSenseCamera(Camera):
    """Wraps pyrealsense2 pipeline for RGB + depth streaming."""

    def __init__(self, cfg: DictConfig) -> None:
        self._cfg = cfg
        self._pipeline = None
        self._align = None
        self._frame_id = 0

    def start(self) -> None:
        import pyrealsense2 as rs

        self._pipeline = rs.pipeline()
        config = rs.config()

        width = self._cfg.get("width", 640)
        height = self._cfg.get("height", 480)
        fps = self._cfg.get("fps", 30)

        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        if self._cfg.get("enable_depth", True):
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        profile = self._pipeline.start(config)

        # Check USB speed
        device = profile.get_device()
        try:
            usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
            if usb_type and float(usb_type) < 3.0:
                logger.warning("RealSense connected via USB %.1s — USB 3.0 recommended for full FPS", usb_type)
            else:
                logger.info("RealSense connected via USB %s", usb_type)
        except Exception:
            pass

        if self._cfg.get("align_depth", True) and self._cfg.get("enable_depth", True):
            self._align = rs.align(rs.stream.color)

        logger.info("RealSenseCamera started (%dx%d @ %dfps, depth=%s)",
                     width, height, fps, self._cfg.get("enable_depth", True))

    def read(self) -> CameraFrame:
        import pyrealsense2 as rs

        frames = self._pipeline.wait_for_frames()

        if self._align:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        rgb = np.asanyarray(color_frame.get_data())

        depth = None
        if self._cfg.get("enable_depth", True):
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                # Convert from millimeters (uint16) to meters (float32)
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth = depth_raw.astype(np.float32) * depth_frame.get_units()

        self._frame_id += 1
        return CameraFrame(rgb=rgb, depth=depth, frame_id=self._frame_id)

    def stop(self) -> None:
        if self._pipeline:
            self._pipeline.stop()
            self._pipeline = None
        logger.info("RealSenseCamera stopped")
