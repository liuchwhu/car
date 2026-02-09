#!/usr/bin/env python3
"""RealSense test - skip get_info, go straight to pipeline."""

import sys
print("Importing pyrealsense2...")
import pyrealsense2 as rs
print("OK")

print("Starting pipeline (depth only, 424x240 @ 15fps)...")
try:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)
    profile = pipeline.start(config)
    print("Pipeline started!")
except Exception as e:
    print("Pipeline failed: {}".format(e))
    sys.exit(1)

print("Reading 5 frames...")
for i in range(5):
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    if depth:
        dist = depth.get_distance(depth.get_width() // 2, depth.get_height() // 2)
        print("  Frame {}: center={:.3f}m".format(i + 1, dist))
    else:
        print("  Frame {}: no depth".format(i + 1))

pipeline.stop()
print("\n=== CAMERA TEST PASSED ===")
