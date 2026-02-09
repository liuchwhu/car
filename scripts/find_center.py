#!/usr/bin/env python3
"""Find the actual steering center angle interactively."""

import board
import busio
from adafruit_servokit import ServoKit

i2c = busio.I2C(board.SCL_1, board.SDA_1)
kit = ServoKit(channels=16, i2c=i2c)

angle = 90
kit.servo[0].angle = angle
print(f"Current angle: {angle}°")
print("Commands: +/- to adjust by 1°, ++/-- by 5°, or type a number. 'q' to quit.\n")

while True:
    cmd = input(f"[{angle}°] > ").strip()
    if cmd == "q":
        break
    elif cmd == "+":
        angle = min(150, angle + 1)
    elif cmd == "-":
        angle = max(30, angle - 1)
    elif cmd == "++":
        angle = min(150, angle + 5)
    elif cmd == "--":
        angle = max(30, angle - 5)
    else:
        try:
            angle = max(30, min(150, int(cmd)))
        except ValueError:
            print("  Use +, -, ++, --, a number, or q")
            continue

    kit.servo[0].angle = angle
    print(f"  -> {angle}°")

print(f"\nYour center angle is: {angle}°")
trim = (angle - 90) / 60.0  # normalized to [-1, 1] range
print(f"Steering trim (normalized): {trim:.4f}")
print(f"\nUpdate configs/hardware/jetson.yaml:")
print(f"  steering_center_angle: {angle}")
print(f"  steering_trim: {trim:.4f}")
