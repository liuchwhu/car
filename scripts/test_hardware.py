#!/usr/bin/env python3
"""Pre-camera hardware tests. Run on Jetson with car on a stand."""

import time
import board
import busio
from adafruit_servokit import ServoKit

CENTER = 75
LEFT_LIMIT = 30
RIGHT_LIMIT = 120

print("Initializing I2C bus 0 (pins 27/28)...")
i2c = busio.I2C(board.SCL_1, board.SDA_1)
kit = ServoKit(channels=16, i2c=i2c)
kit.servo[0].angle = CENTER
kit.continuous_servo[1].throttle = 0.0
print("PCA9685 connected.\n")


# === Test 1: Normalized steering mapping ===
print("=== TEST 1: Steering Mapping (-1.0 to +1.0) ===")
print("Verifying that normalized values map to correct angles.\n")

steer_range = (RIGHT_LIMIT - LEFT_LIMIT) / 2.0
test_values = [("full left", -1.0), ("half left", -0.5), ("center", 0.0),
               ("half right", 0.5), ("full right", 1.0)]

for label, norm in test_values:
    angle = CENTER + norm * steer_range
    angle = max(LEFT_LIMIT, min(RIGHT_LIMIT, angle))
    kit.servo[0].angle = angle
    print(f"  steering={norm:+.1f} -> {angle:.0f}° ({label})")
    time.sleep(0.8)

kit.servo[0].angle = CENTER
time.sleep(0.5)
print("Steering mapping OK\n")


# === Test 2: ESC dead zone ===
print("=== TEST 2: ESC Dead Zone (finding minimum throttle) ===")
print("Testing throttle from 0.05 to 0.30 in steps of 0.05.")
print("Watch the wheels — note which value first produces movement.\n")
time.sleep(1)

for throttle in [0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30]:
    print(f"  throttle={throttle:.2f} ...", end=" ", flush=True)
    kit.continuous_servo[1].throttle = throttle
    time.sleep(1.5)
    kit.continuous_servo[1].throttle = 0.0
    time.sleep(0.5)
    print("done")

print("Dead zone test complete. Note the first value that moved.\n")


# === Test 3: Reverse dead zone ===
print("=== TEST 3: Reverse Dead Zone ===")
for throttle in [-0.05, -0.10, -0.15, -0.20]:
    print(f"  throttle={throttle:.2f} ...", end=" ", flush=True)
    kit.continuous_servo[1].throttle = throttle
    time.sleep(1.5)
    kit.continuous_servo[1].throttle = 0.0
    time.sleep(0.5)
    print("done")

print("Reverse dead zone test complete.\n")


# === Test 4: Steering under throttle ===
print("=== TEST 4: Steering While Moving ===")
print("Gentle forward + steering sweep to check for binding.\n")
time.sleep(1)

kit.continuous_servo[1].throttle = 0.15
time.sleep(0.5)

for norm in [-0.5, 0.0, 0.5, 0.0]:
    angle = CENTER + norm * steer_range
    kit.servo[0].angle = angle
    label = "left" if norm < 0 else ("right" if norm > 0 else "center")
    print(f"  steering={norm:+.1f} ({label})")
    time.sleep(1.0)

kit.continuous_servo[1].throttle = 0.0
kit.servo[0].angle = CENTER
time.sleep(0.5)
print("Steering under throttle OK\n")


print("=== ALL TESTS COMPLETE ===")
