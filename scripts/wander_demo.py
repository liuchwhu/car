#!/usr/bin/env python3
"""
Standalone wander demo — car drives around avoiding obstacles.
No pilotnano package required. Run on Jetson with car ON THE GROUND.

Strategy: drive straight → stop when obstacle ahead → reverse with steering → repeat.
Simple and decisive — no gradual steering into walls.

Usage:
    python3 wander_demo.py              # run headless (console output)
    python3 wander_demo.py --stream     # also start MJPEG stream on :8080

Press Ctrl+C to stop (sends neutral command before exiting).

Camera mounting: 8cm behind front bumper, 12cm off ground.
Vehicle width: 17.5cm.
"""

import argparse
import enum
import random
import signal
import time
import threading

import numpy as np
import pyrealsense2 as rs
import board
import busio
from adafruit_servokit import ServoKit


# ─── Hardware constants ───────────────────────────────────────────────
STEERING_CENTER = 75
STEERING_MIN = 30
STEERING_MAX = 120
STEERING_RANGE = (STEERING_MAX - STEERING_MIN) / 2.0  # 45 degrees each side

# ─── Tuning ───────────────────────────────────────────────────────────
WANDER_THROTTLE = 0.08       # gentle forward
TURN_THROTTLE = 0.10         # throttle while turning (needs to overcome friction)
REVERSE_THROTTLE = -0.10     # reverse speed
LOOP_HZ = 15                 # main loop rate

# Depth ROI
# Must see low obstacles (as short as 15cm / 6 inches).
# Camera at 12cm height — use nearly full frame, only skip top 20% (ceiling).
# Ground will appear at ~25cm depth at very bottom rows, but we filter
# by MIN_VALID_DEPTH and the stop distance is larger than that.
ROI_TOP = 0.20               # ignore top 20% (ceiling/sky)
ROI_BOTTOM = 0.85            # include most of the frame to catch low obstacles
MIN_VALID_DEPTH = 0.15       # meters — ignore closer (noise)
MAX_VALID_DEPTH = 5.0        # meters — ignore further

# Distances
STOP_DISTANCE = 0.25         # meters — stop and reverse when obstacle this close
AVOIDANCE_DISTANCE = 0.80    # meters — start steering away when obstacle this close
AVOIDANCE_GAIN = 1.0         # how aggressively to steer (multiplier on steering)

# Stuck detection — wheels commanded but depth not changing
STUCK_TIMEOUT = 1.5           # seconds of no depth change before declaring stuck
STUCK_DEPTH_THRESHOLD = 0.02  # meters — min depth change to count as "moving"

# Reverse+steer timing
# Reverse with full steering lock pivots the car ~60-90°/sec depending on surface.
REVERSE_STEER_DURATION = 1.5  # seconds to reverse with steering (creates ~90° turn)
STOP_PAUSE = 0.2              # seconds to pause (stop motor) before reversing

# Reverse stuck detection — abort reverse if car isn't moving (hit rear wall)
REVERSE_STUCK_CHECK_AFTER = 0.5   # seconds into reverse before checking
REVERSE_STUCK_THRESHOLD = 0.03   # meters — min depth change to count as moving during reverse
MAX_FAILED_REVERSES = 2          # after this many failed reverses, try forward+turn instead
FORWARD_TURN_DURATION = 1.0      # seconds to drive forward with full steering


# ─── State machine ────────────────────────────────────────────────────
class State(enum.Enum):
    CRUISE = "cruise"         # driving forward
    STOPPING = "stopping"     # brief pause before reversing
    REVERSING = "reversing"   # backing up WITH steering (pivot away)
    FORWARD_TURN = "fwd_turn" # drive forward with full steering (when reverse fails)


# ─── Actuator helpers ────────────────────────────────────────────────
def init_hardware():
    """Initialize I2C and ServoKit."""
    print("Initializing I2C bus 0 (pins 27/28)...")
    i2c = busio.I2C(board.SCL_1, board.SDA_1)
    kit = ServoKit(channels=16, i2c=i2c)
    kit.servo[0].angle = STEERING_CENTER
    kit.continuous_servo[1].throttle = 0.0
    print("PCA9685 connected.")
    return kit


def send_command(kit, steering, throttle):
    """Send normalized steering [-1,1] and throttle [-1,1] to hardware."""
    angle = STEERING_CENTER + steering * STEERING_RANGE
    angle = max(STEERING_MIN, min(STEERING_MAX, angle))
    kit.servo[0].angle = angle
    kit.continuous_servo[1].throttle = max(-1.0, min(1.0, throttle))


def pick_turn_direction(left, right):
    """Pick turn direction: prefer clearer side, but randomize when similar."""
    if left > right * 1.3:
        return -1.0   # left is clearly more open
    elif right > left * 1.3:
        return 1.0    # right is clearly more open
    else:
        return random.choice([-1.0, 1.0])  # similar — pick randomly


def stop_car(kit):
    """Send neutral command."""
    kit.servo[0].angle = STEERING_CENTER
    kit.continuous_servo[1].throttle = 0.0


# ─── Depth analysis ─────────────────────────────────────────────────
def analyze_depth(depth_frame):
    """Analyze depth frame into left/center/right zone distances."""
    depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_frame.get_units()
    h, w = depth_m.shape

    top = int(h * ROI_TOP)
    bottom = int(h * ROI_BOTTOM)
    roi = depth_m[top:bottom, :]

    third = w // 3
    left_zone = roi[:, :third]
    center_zone = roi[:, third:2 * third]
    right_zone = roi[:, 2 * third:]

    def min_dist(zone):
        valid = zone[(zone > MIN_VALID_DEPTH) & (zone < MAX_VALID_DEPTH)]
        if len(valid) == 0:
            return float("inf")
        return float(np.percentile(valid, 5))

    left = min_dist(left_zone)
    center = min_dist(center_zone)
    right = min_dist(right_zone)
    closest = min(left, center, right)

    return left, center, right, closest


# ─── MJPEG stream (optional) ─────────────────────────────────────────
def start_stream_server(get_frame_func):
    """Start a background MJPEG server on port 8080."""
    from http.server import HTTPServer, BaseHTTPRequestHandler

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(b'<html><body style="margin:0;background:#000">'
                                 b'<img src="/stream" style="width:100%">'
                                 b'</body></html>')
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        jpeg = get_frame_func()
                        if jpeg is not None:
                            self.wfile.write(b'--frame\r\n'
                                             b'Content-Type: image/jpeg\r\n\r\n')
                            self.wfile.write(jpeg)
                            self.wfile.write(b'\r\n')
                        time.sleep(0.066)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, fmt, *args):
            pass

    server = HTTPServer(('0.0.0.0', 8080), Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print("MJPEG stream on http://0.0.0.0:8080")


# ─── Main loop ───────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Wander demo")
    parser.add_argument("--stream", action="store_true", help="Enable MJPEG stream on :8080")
    args = parser.parse_args()

    kit = init_hardware()

    # Graceful shutdown
    running = True
    def on_signal(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Start camera
    print("Starting RealSense pipeline (640x480 @ 30fps)...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Warm up camera
    print("Warming up camera...")
    for _ in range(10):
        pipeline.wait_for_frames()

    # Optional MJPEG stream
    latest_jpeg = [None]
    has_cv2 = False
    if args.stream:
        try:
            import cv2
            has_cv2 = True
            start_stream_server(lambda: latest_jpeg[0])
        except ImportError:
            print("WARNING: cv2 not available, stream disabled")

    # State machine
    state = State.CRUISE
    turn_direction = 1.0       # +1 = right, -1 = left
    maneuver_start = 0.0

    # Stuck detection
    prev_center = None
    depth_steady_since = None   # when center depth stopped changing

    # Reverse progress tracking
    reverse_start_depth = None  # closest depth when reverse began
    failed_reverses = 0         # consecutive reverses that didn't help

    tick_period = 1.0 / LOOP_HZ
    frame_count = 0

    print("\n=== WANDER DEMO STARTED ===")
    print("Throttle: {:.2f} | Stop at: {:.0f}cm | Stuck timeout: {:.1f}s".format(
        WANDER_THROTTLE, STOP_DISTANCE * 100, STUCK_TIMEOUT))
    print("Reverse+steer: {:.1f}s | ROI: {:.0f}%-{:.0f}%".format(
        REVERSE_STEER_DURATION, ROI_TOP * 100, ROI_BOTTOM * 100))
    print("Press Ctrl+C to stop.\n")

    try:
        while running:
            t0 = time.monotonic()
            now = t0

            # Read frames
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # Analyze depth
            left, center, right, closest = analyze_depth(depth_frame)

            # ── State machine ──
            steering = 0.0
            throttle = 0.0

            if state == State.CRUISE:
                if closest < STOP_DISTANCE:
                    # Obstacle in ANY zone — stop first, then reverse with steering
                    state = State.STOPPING
                    turn_direction = pick_turn_direction(left, right)
                    maneuver_start = now
                    steering = 0.0
                    throttle = 0.0
                    prev_center = None
                    depth_steady_since = None
                    print("  Obstacle at {:.2f}m (closest) — stopping, reverse+steer {} (L={:.2f} C={:.2f} R={:.2f})".format(
                        closest, "right" if turn_direction > 0 else "left", left, center, right))

                else:
                    # Clear of emergency stop — drive forward with avoidance steering
                    throttle = WANDER_THROTTLE

                    # Proportional avoidance: steer away from closer side
                    if closest < AVOIDANCE_DISTANCE:
                        # Steer away from the closer side
                        # Positive steering = turn right, negative = turn left
                        # If left is closer, steer right (positive); if right is closer, steer left (negative)
                        if left < AVOIDANCE_DISTANCE or right < AVOIDANCE_DISTANCE:
                            # How urgent: 1.0 at STOP_DISTANCE, 0.0 at AVOIDANCE_DISTANCE
                            urgency = 1.0 - (closest - STOP_DISTANCE) / (AVOIDANCE_DISTANCE - STOP_DISTANCE)
                            urgency = max(0.0, min(1.0, urgency))
                            if left < right:
                                steering = urgency * AVOIDANCE_GAIN   # steer right (away from left wall)
                            else:
                                steering = -urgency * AVOIDANCE_GAIN  # steer left (away from right wall)
                        elif center < AVOIDANCE_DISTANCE:
                            # Center blocked but sides open — steer toward more open side
                            urgency = 1.0 - (center - STOP_DISTANCE) / (AVOIDANCE_DISTANCE - STOP_DISTANCE)
                            urgency = max(0.0, min(1.0, urgency))
                            if left > right:
                                steering = -urgency * AVOIDANCE_GAIN  # steer left
                            else:
                                steering = urgency * AVOIDANCE_GAIN   # steer right
                        steering = max(-1.0, min(1.0, steering))
                    else:
                        steering = 0.0

                    # Stuck detection: depth not changing while driving
                    # Only check when obstacle is within 2m — at longer range,
                    # depth changes slowly and false-triggers stuck detection
                    if prev_center is not None and center < 2.0:
                        depth_change = abs(center - prev_center)
                        if depth_change < STUCK_DEPTH_THRESHOLD:
                            if depth_steady_since is None:
                                depth_steady_since = now
                            elif now - depth_steady_since > STUCK_TIMEOUT:
                                # Stuck! Stop then reverse with steering
                                state = State.STOPPING
                                turn_direction = pick_turn_direction(left, right)
                                maneuver_start = now
                                steering = 0.0
                                throttle = 0.0
                                prev_center = None
                                depth_steady_since = None
                                print("  STUCK (depth steady at {:.2f}m for {:.1f}s) — stopping, then reverse+steer {}".format(
                                    center, STUCK_TIMEOUT, "right" if turn_direction > 0 else "left"))
                        else:
                            depth_steady_since = None
                    else:
                        depth_steady_since = None
                    prev_center = center

            elif state == State.STOPPING:
                # Brief pause to stop momentum before reversing
                elapsed = now - maneuver_start
                steering = 0.0
                throttle = 0.0
                if elapsed > STOP_PAUSE:
                    if failed_reverses >= MAX_FAILED_REVERSES:
                        # Reverse keeps failing (rear wall) — try forward+turn
                        state = State.FORWARD_TURN
                        maneuver_start = now
                        # Turn opposite to reverse direction (steer away from front obstacle)
                        failed_reverses = 0
                        print("  Reverse failed {}x — forward+turn {} instead".format(
                            MAX_FAILED_REVERSES, "right" if turn_direction > 0 else "left"))
                    else:
                        state = State.REVERSING
                        maneuver_start = now
                        reverse_start_depth = closest
                        print("  Reversing with steering...")

            elif state == State.REVERSING:
                # Reverse WITH steering — pivot away from obstacle
                elapsed = now - maneuver_start

                # Check if reverse is actually working (depth increasing = moving away from front obstacle)
                # If depth hasn't increased after 0.5s, we've hit a rear wall
                if elapsed > REVERSE_STUCK_CHECK_AFTER and reverse_start_depth is not None:
                    depth_gain = closest - reverse_start_depth
                    if depth_gain < REVERSE_STUCK_THRESHOLD:
                        # Not moving — rear wall. Abort reverse.
                        failed_reverses += 1
                        state = State.STOPPING
                        maneuver_start = now
                        steering = 0.0
                        throttle = 0.0
                        print("  Reverse stuck (depth {:.2f}→{:.2f}m, no gain) — failed {} time(s)".format(
                            reverse_start_depth, closest, failed_reverses))
                        # Skip the rest of this tick
                        send_command(kit, steering, throttle)
                        dt = time.monotonic() - t0
                        sleep_time = tick_period - dt
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                        continue

                if elapsed > REVERSE_STEER_DURATION:
                    # Done reversing — check if clear now (all zones)
                    failed_reverses = 0  # successful reverse
                    if closest > STOP_DISTANCE:
                        state = State.CRUISE
                        steering = 0.0
                        throttle = WANDER_THROTTLE
                        prev_center = None
                        depth_steady_since = None
                        print("  Reverse+steer complete — clear ({:.2f}m), cruising".format(closest))
                    else:
                        # Still blocked — reverse+steer again, keep same direction
                        maneuver_start = now
                        reverse_start_depth = closest
                        print("  Still blocked ({:.2f}m) — reverse+steer {} again".format(
                            closest, "right" if turn_direction > 0 else "left"))
                else:
                    steering = turn_direction   # full lock while reversing
                    throttle = REVERSE_THROTTLE

            elif state == State.FORWARD_TURN:
                # Drive forward with full steering — escape when reverse fails
                elapsed = now - maneuver_start
                if elapsed > FORWARD_TURN_DURATION:
                    state = State.CRUISE
                    steering = 0.0
                    throttle = WANDER_THROTTLE
                    prev_center = None
                    depth_steady_since = None
                    print("  Forward+turn complete ({:.2f}m), cruising".format(closest))
                else:
                    steering = turn_direction
                    throttle = TURN_THROTTLE

            # Send to hardware
            send_command(kit, steering, throttle)

            # Build MJPEG frame (if streaming)
            if has_cv2:
                import cv2
                rgb = np.asanyarray(color_frame.get_data())
                h_img, w_img = rgb.shape[:2]

                # Draw ROI bounds
                roi_y1 = int(h_img * ROI_TOP)
                roi_y2 = int(h_img * ROI_BOTTOM)
                cv2.line(rgb, (0, roi_y1), (w_img, roi_y1), (0, 255, 255), 1)
                cv2.line(rgb, (0, roi_y2), (w_img, roi_y2), (0, 255, 255), 1)

                # Draw zone dividers
                t3 = w_img // 3
                cv2.line(rgb, (t3, roi_y1), (t3, roi_y2), (255, 255, 0), 1)
                cv2.line(rgb, (2 * t3, roi_y1), (2 * t3, roi_y2), (255, 255, 0), 1)

                # HUD
                state_name = state.value
                if state == State.CRUISE:
                    color = (0, 255, 0)
                elif state == State.STOPPING:
                    color = (0, 128, 255)
                elif state == State.FORWARD_TURN:
                    color = (255, 0, 255)
                else:
                    color = (0, 0, 255)

                cv2.putText(rgb, "[{}]".format(state_name),
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(rgb, "S:{:+.2f} T:{:+.2f}".format(steering, throttle),
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(rgb, "L:{:.2f} C:{:.2f} R:{:.2f}".format(left, center, right),
                            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Draw stop distance line (where center obstacle triggers stop)
                # Approximate: at STOP_DISTANCE, the obstacle fills roughly the center
                cv2.putText(rgb, "stop<{:.0f}cm".format(STOP_DISTANCE * 100),
                            (w_img - 150, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)

                _, jpeg = cv2.imencode('.jpg', rgb, [cv2.IMWRITE_JPEG_QUALITY, 65])
                latest_jpeg[0] = jpeg.tobytes()

            # Console output (every second)
            frame_count += 1
            if frame_count % LOOP_HZ == 0:
                elapsed_ms = (time.monotonic() - t0) * 1000
                print("  [{:>10s}] L={:.2f} C={:.2f} R={:.2f} | S={:+.2f} T={:+.2f} | {:.0f}ms".format(
                    state.value, left, center, right, steering, throttle, elapsed_ms))

            # Rate limit
            dt = time.monotonic() - t0
            sleep_time = tick_period - dt
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        print("\nStopping...")
        stop_car(kit)
        time.sleep(0.2)
        pipeline.stop()
        print("=== WANDER DEMO STOPPED ===")


if __name__ == "__main__":
    main()
