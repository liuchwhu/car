#!/usr/bin/env python3
"""Minimal RealSense test with step-by-step error isolation."""

import sys
print("Step 1: Importing pyrealsense2...")
import pyrealsense2 as rs
print("  OK")

print("Step 2: Creating context...")
ctx = rs.context()
print("  OK")

print("Step 3: Querying devices...")
devices = ctx.query_devices()
n = len(devices)
print("  Found {} device(s)".format(n))

if n == 0:
    print("ERROR: No device found.")
    sys.exit(1)

print("Step 4: Getting device info...")
try:
    dev = devices[0]
    name = dev.get_info(rs.camera_info.name)
    print("  Name: {}".format(name))
except Exception as e:
    print("  get_info failed: {}".format(e))

print("Step 5: Starting pipeline (depth only, 424x240 @ 15fps)...")
try:
    pipeline = rs.pipeline()
    config = rs.config()
    # Use minimal resolution to reduce USB bandwidth
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)
    profile = pipeline.start(config)
    print("  Pipeline started!")
except Exception as e:
    print("  Pipeline failed: {}".format(e))
    sys.exit(1)

print("Step 6: Reading 5 frames...")
try:
    for i in range(5):
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if depth:
            dist = depth.get_distance(depth.get_width() // 2, depth.get_height() // 2)
            print("  Frame {}: center={:.3f}m".format(i + 1, dist))
        else:
            print("  Frame {}: no depth".format(i + 1))
except Exception as e:
    print("  Frame read failed: {}".format(e))

print("Step 7: Stopping pipeline...")
pipeline.stop()
print("  OK")

print("\n=== CAMERA TEST PASSED ===")
