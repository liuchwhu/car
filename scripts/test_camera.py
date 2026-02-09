#!/usr/bin/env python3
"""Quick RealSense D435 test. Run on Jetson."""

import time
import numpy as np
import pyrealsense2 as rs

print("=== RealSense D435 Test ===\n")

# Check connected devices
ctx = rs.context()
devices = ctx.query_devices()
print(f"Found {len(devices)} RealSense device(s)")
for dev in devices:
    print(f"  Name: {dev.get_info(rs.camera_info.name)}")
    print(f"  Serial: {dev.get_info(rs.camera_info.serial_number)}")
    try:
        print(f"  USB: {dev.get_info(rs.camera_info.usb_type_descriptor)}")
    except Exception:
        pass
    print()

if len(devices) == 0:
    print("ERROR: No RealSense device found! Check USB connection.")
    exit(1)

# Start pipeline
print("Starting pipeline (640x480 @ 30fps, RGB + Depth)...")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

print("Warming up (5 frames)...")
for i in range(5):
    pipeline.wait_for_frames()

# Capture 10 frames and report stats
print("\nCapturing 10 frames...\n")
for i in range(10):
    t0 = time.monotonic()
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    color = frames.get_color_frame()
    depth = frames.get_depth_frame()

    rgb = np.asanyarray(color.get_data())
    depth_raw = np.asanyarray(depth.get_data())
    depth_m = depth_raw.astype(np.float32) * depth.get_units()

    elapsed = (time.monotonic() - t0) * 1000

    # Depth stats (center region)
    h, w = depth_m.shape
    center = depth_m[h//3:2*h//3, w//3:2*w//3]
    valid = center[center > 0.1]

    if len(valid) > 0:
        center_dist = np.median(valid)
        print(f"  Frame {i+1}: rgb={rgb.shape} depth={depth_m.shape} "
              f"center={center_dist:.2f}m min={valid.min():.2f}m "
              f"({elapsed:.1f}ms)")
    else:
        print(f"  Frame {i+1}: rgb={rgb.shape} depth={depth_m.shape} "
              f"no valid depth ({elapsed:.1f}ms)")

pipeline.stop()
print("\n=== CAMERA TEST PASSED ===")
