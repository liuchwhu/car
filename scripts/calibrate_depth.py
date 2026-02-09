#!/usr/bin/env python3
"""
Depth calibration — place the car at known distances from a wall
and compare reported depth vs actual distance.

Run on Jetson: python3 calibrate_depth.py

Instructions:
1. Place the car facing a flat wall
2. Measure the distance from the CAMERA LENS to the wall
3. Press Enter to take a reading
4. Repeat at different distances
5. Press 'q' to quit
"""

import numpy as np
import pyrealsense2 as rs

# Same ROI as wander_demo
ROI_TOP = 0.30
ROI_BOTTOM = 0.65
MIN_VALID_DEPTH = 0.10
MAX_VALID_DEPTH = 5.0


def analyze_depth(depth_frame):
    depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_frame.get_units()
    h, w = depth_m.shape

    top = int(h * ROI_TOP)
    bottom = int(h * ROI_BOTTOM)
    roi = depth_m[top:bottom, :]

    third = w // 3
    left_zone = roi[:, :third]
    center_zone = roi[:, third:2 * third]
    right_zone = roi[:, 2 * third:]

    def stats(zone):
        valid = zone[(zone > MIN_VALID_DEPTH) & (zone < MAX_VALID_DEPTH)]
        if len(valid) == 0:
            return float("inf"), 0, 0
        return float(np.percentile(valid, 5)), float(np.median(valid)), len(valid)

    l5, lmed, ln = stats(left_zone)
    c5, cmed, cn = stats(center_zone)
    r5, rmed, rn = stats(right_zone)

    return {
        "left": {"p5": l5, "median": lmed, "pixels": ln},
        "center": {"p5": c5, "median": cmed, "pixels": cn},
        "right": {"p5": r5, "median": rmed, "pixels": rn},
    }


def main():
    print("Starting RealSense pipeline...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # Warm up
    for _ in range(15):
        pipeline.wait_for_frames()

    print("\n=== DEPTH CALIBRATION ===")
    print("Place car facing a flat wall.")
    print("Type the actual distance (cm) from CAMERA LENS to wall, then Enter.")
    print("Type 'q' to quit.\n")

    # Continuous display
    print("Live readings (updates every second):\n")

    readings = []

    try:
        while True:
            # Average 10 frames for stability
            samples = []
            for _ in range(10):
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    result = analyze_depth(depth_frame)
                    samples.append(result["center"]["p5"])

            avg_depth = np.mean(samples)
            med_depth = np.median(samples)
            result = analyze_depth(depth_frame)

            print("  Reported: center={:.1f}cm (p5={:.1f}cm, median={:.1f}cm) | L={:.1f}cm R={:.1f}cm".format(
                avg_depth * 100, result["center"]["p5"] * 100, result["center"]["median"] * 100,
                result["left"]["p5"] * 100, result["right"]["p5"] * 100))

            user = input("  Actual distance (cm), or Enter to re-read, 'q' to quit: ").strip()

            if user.lower() == 'q':
                break
            elif user == '':
                continue
            else:
                try:
                    actual_cm = float(user)
                    readings.append({
                        "actual_cm": actual_cm,
                        "reported_cm": avg_depth * 100,
                        "p5_cm": result["center"]["p5"] * 100,
                        "median_cm": result["center"]["median"] * 100,
                    })
                    print("    Recorded: actual={:.0f}cm, reported={:.1f}cm, error={:+.1f}cm\n".format(
                        actual_cm, avg_depth * 100, avg_depth * 100 - actual_cm))
                except ValueError:
                    print("    Invalid number, try again\n")

    finally:
        pipeline.stop()

    if readings:
        print("\n=== CALIBRATION RESULTS ===")
        print("{:>12s} {:>12s} {:>12s} {:>12s}".format(
            "Actual(cm)", "Reported(cm)", "Error(cm)", "Ratio"))
        for r in readings:
            err = r["reported_cm"] - r["actual_cm"]
            ratio = r["reported_cm"] / r["actual_cm"] if r["actual_cm"] > 0 else 0
            print("{:12.0f} {:12.1f} {:12.1f} {:12.3f}".format(
                r["actual_cm"], r["reported_cm"], err, ratio))

        actuals = [r["actual_cm"] for r in readings]
        reporteds = [r["reported_cm"] for r in readings]
        errors = [r - a for r, a in zip(reporteds, actuals)]
        print("\nMean error: {:.1f}cm".format(np.mean(errors)))
        print("Mean ratio (reported/actual): {:.3f}".format(np.mean([r/a for r, a in zip(reporteds, actuals) if a > 0])))


if __name__ == "__main__":
    main()
