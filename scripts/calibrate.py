#!/usr/bin/env python3
"""Interactive and auto-calibration CLI for servo and ESC."""

import argparse
import sys
import time


def interactive_calibrate():
    """Walk through manual servo/ESC calibration."""
    try:
        import board
        import busio
        from adafruit_servokit import ServoKit
    except ImportError:
        print("ERROR: Adafruit ServoKit not installed. Run on Jetson with [jetson] extras.")
        sys.exit(1)

    print("=== PilotNano Calibration Tool ===\n")

    # Initialize I2C bus 0
    print("Initializing I2C bus 0 (pins 27/28)...")
    i2c = busio.I2C(board.SCL_1, board.SDA_1)
    kit = ServoKit(channels=16, i2c=i2c)
    print("PCA9685 connected.\n")

    # Steering calibration
    print("--- Steering Servo (Channel 0) ---")
    print("The servo will sweep through its range. Watch the wheels.\n")

    print("Setting center (90°)...")
    kit.servo[0].angle = 90
    time.sleep(1)

    print("Sweeping left (30°)...")
    for angle in range(90, 29, -2):
        kit.servo[0].angle = angle
        time.sleep(0.02)
    time.sleep(0.5)
    left_limit = input("Enter the actual left limit angle you observed (default 30): ").strip()
    left_limit = int(left_limit) if left_limit else 30

    print("Sweeping right (150°)...")
    for angle in range(30, 151, 2):
        kit.servo[0].angle = angle
        time.sleep(0.02)
    time.sleep(0.5)
    right_limit = input("Enter the actual right limit angle you observed (default 150): ").strip()
    right_limit = int(right_limit) if right_limit else 150

    print("Returning to center...")
    kit.servo[0].angle = 90
    time.sleep(0.5)

    # Find center trim
    print("\nDoes the car steer straight at 90°? Adjust trim if needed.")
    trim_angle = input("Enter center angle that makes wheels straight (default 90): ").strip()
    trim_angle = int(trim_angle) if trim_angle else 90
    trim_offset = (trim_angle - 90) / ((right_limit - left_limit) / 2)

    # ESC calibration
    print("\n--- ESC (Channel 1, Continuous Servo Mode) ---")
    print("Make sure the car is on a stand (wheels off ground)!\n")
    input("Press Enter when ready to arm ESC...")

    print("Sending neutral (0.0)...")
    kit.continuous_servo[1].throttle = 0.0
    time.sleep(2)

    print("Testing gentle forward (0.15)...")
    kit.continuous_servo[1].throttle = 0.15
    time.sleep(2)
    kit.continuous_servo[1].throttle = 0.0
    time.sleep(1)

    print("Testing gentle reverse (-0.15)...")
    kit.continuous_servo[1].throttle = -0.15
    time.sleep(2)
    kit.continuous_servo[1].throttle = 0.0
    time.sleep(1)

    # Save results
    print("\n=== Calibration Results ===")
    print(f"Steering range: {left_limit}° to {right_limit}°")
    print(f"Steering center: {trim_angle}° (trim offset: {trim_offset:.3f})")
    print(f"ESC: responding to throttle commands")

    from pathlib import Path
    from omegaconf import OmegaConf
    import datetime

    result = OmegaConf.create({
        "timestamp": datetime.datetime.now().isoformat(),
        "steering_trim": float(trim_offset),
        "steering_limits": {
            "min_angle": left_limit,
            "max_angle": right_limit,
            "center_angle": trim_angle,
        },
    })

    result_path = Path(__file__).resolve().parents[1] / "configs" / "calibration" / "calibration_result.yaml"
    OmegaConf.save(result, result_path)
    print(f"\nSaved to {result_path}")


def auto_calibrate():
    """Run auto-calibration using the camera as measurement instrument."""
    from pilotnano.config import load_config
    from pilotnano.utils.logging import setup_logging
    from pilotnano.bus import LocalBus
    from pilotnano.hardware import create_camera, create_actuator
    from pilotnano.calibration.auto_calibrator import AutoCalibrator

    cfg = load_config()
    setup_logging(cfg.get("log_level", "INFO"))

    bus = LocalBus()
    camera = create_camera(cfg.hardware)
    actuator = create_actuator(cfg.hardware)

    calibrator = AutoCalibrator(cfg, bus, camera, actuator)

    with camera, actuator:
        calibrator.start()
        try:
            while calibrator.running:
                calibrator.step()
        except KeyboardInterrupt:
            print("\nCalibration interrupted.")
        finally:
            calibrator.stop()


def main():
    parser = argparse.ArgumentParser(description="PilotNano Calibration")
    parser.add_argument("--auto", action="store_true",
                        help="Run auto-calibration using camera (requires open space + wall)")
    args = parser.parse_args()

    if args.auto:
        auto_calibrate()
    else:
        interactive_calibrate()


if __name__ == "__main__":
    main()
