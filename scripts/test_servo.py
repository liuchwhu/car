#!/usr/bin/env python3
"""Quick hardware test — steering sweep + throttle pulse. Run on Jetson with car on stand."""

import time
import board
import busio
from adafruit_servokit import ServoKit

# Calibrated values
CENTER = 75
LEFT_LIMIT = 30
RIGHT_LIMIT = 120

print("Initializing I2C bus 0 (pins 27/28)...")
i2c = busio.I2C(board.SCL_1, board.SDA_1)
kit = ServoKit(channels=16, i2c=i2c)
print("PCA9685 connected.\n")

# --- Steering test ---
print(f"=== STEERING TEST (ch0, {LEFT_LIMIT}°-{RIGHT_LIMIT}°, center={CENTER}°) ===")
print(f"Center ({CENTER}°)...")
kit.servo[0].angle = CENTER
time.sleep(1)

print(f"Left ({LEFT_LIMIT}°)...")
for a in range(CENTER, LEFT_LIMIT - 1, -2):
    kit.servo[0].angle = a
    time.sleep(0.02)
time.sleep(0.5)

print(f"Right ({RIGHT_LIMIT}°)...")
for a in range(LEFT_LIMIT, RIGHT_LIMIT + 1, 2):
    kit.servo[0].angle = a
    time.sleep(0.02)
time.sleep(0.5)

print(f"Back to center ({CENTER}°)...")
for a in range(RIGHT_LIMIT, CENTER - 1, -2):
    kit.servo[0].angle = a
    time.sleep(0.02)
time.sleep(0.5)
print("Steering OK\n")

# --- Throttle test ---
print("=== THROTTLE TEST (ch1, continuous servo mode) ===")
print("Make sure wheels are OFF the ground!\n")
time.sleep(1)

print("Forward gentle (0.15) for 2s...")
kit.continuous_servo[1].throttle = 0.15
time.sleep(2)

print("Stop...")
kit.continuous_servo[1].throttle = 0.0
time.sleep(1)

print("Reverse gentle (-0.15) for 2s...")
kit.continuous_servo[1].throttle = -0.15
time.sleep(2)

print("Stop...")
kit.continuous_servo[1].throttle = 0.0
time.sleep(0.5)

print("\n=== ALL TESTS PASSED ===")
