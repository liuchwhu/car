#!/usr/bin/env python3
"""Find the actual steering left/right limits interactively."""

import board
import busio
from adafruit_servokit import ServoKit

i2c = busio.I2C(board.SCL_1, board.SDA_1)
kit = ServoKit(channels=16, i2c=i2c)

CENTER = 75
angle = CENTER
kit.servo[0].angle = angle

print(f"Center is {CENTER}°. Starting there.")
print("Commands: +/- by 1°, ++/-- by 5°, number, 'q' to quit\n")
print("Step 1: Find LEFT limit — use - to go left until wheels max out.")
print("Step 2: Type 'L' to save left limit, then find RIGHT with +.")
print("Step 3: Type 'R' to save right limit, then 'q' to finish.\n")

left_limit = None
right_limit = None

while True:
    label = f"[{angle}°]"
    if left_limit is not None:
        label += f" L={left_limit}°"
    if right_limit is not None:
        label += f" R={right_limit}°"

    cmd = input(f"{label} > ").strip()

    if cmd == "q":
        break
    elif cmd == "L":
        left_limit = angle
        print(f"  Left limit saved: {left_limit}°")
        continue
    elif cmd == "R":
        right_limit = angle
        print(f"  Right limit saved: {right_limit}°")
        continue
    elif cmd == "+":
        angle = min(180, angle + 1)
    elif cmd == "-":
        angle = max(0, angle - 1)
    elif cmd == "++":
        angle = min(180, angle + 5)
    elif cmd == "--":
        angle = max(0, angle - 5)
    elif cmd == "c":
        angle = CENTER
    else:
        try:
            angle = max(0, min(180, int(cmd)))
        except ValueError:
            print("  Use +, -, ++, --, L, R, c (center), number, or q")
            continue

    kit.servo[0].angle = angle
    print(f"  -> {angle}°")

# Return to center
kit.servo[0].angle = CENTER

print(f"\n=== Results ===")
print(f"Center: {CENTER}°")
if left_limit is not None:
    print(f"Left limit: {left_limit}°")
if right_limit is not None:
    print(f"Right limit: {right_limit}°")
if left_limit and right_limit:
    print(f"\nUpdate configs/hardware/jetson.yaml:")
    print(f"  steering_min_angle: {left_limit}")
    print(f"  steering_max_angle: {right_limit}")
    print(f"  steering_center_angle: {CENTER}")
