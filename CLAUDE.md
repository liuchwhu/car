# PilotNano — Self-Driving RC Car

## Project Overview

Self-driving RC car using **Jetson Nano + Tamiya XV-02 + PCA9685 + Intel RealSense D435**.
Algorithmic driving (obstacle avoidance, object following) — no behavioral cloning.

Python package: `pilotnano`, installed from `src/pilotnano/`.
Full plan: `PLAN.md`. Configs: `configs/`.

## Hardware

| Component | Details |
|-----------|---------|
| Computer | Jetson Nano (JetPack, Ubuntu 18.04, aarch64) |
| Camera | Intel RealSense D435 (USB 3.2, 640x480 RGB+Depth @ 30fps) |
| PWM driver | PCA9685 on I2C Bus 0 (pins 27/28: `board.SCL_1` / `board.SDA_1`) |
| Steering | PCA9685 ch0, position servo, 30°–120° (center=75°) |
| Throttle | PCA9685 ch1, ESC in continuous servo mode, throttle -1.0 to 1.0 |
| Chassis | Tamiya XV-02 |
| Battery | 2S LiPo |
| Power | Jetson powered by BEC direct from 2S (NOT through PCA9685) |

## Jetson SSH Access

- Host: `192.168.1.233`, user: `lc`, credentials in `.env` (gitignored)
- Use `expect` for SSH from macOS (no `sshpass` installed)
- Python: use `python3` (conda archiconda3, Python 3.7). `python` = system 2.7, avoid.
- librealsense2 v2.54.2 (apt, RSUSB backend). Python bindings built from source in `/home/lc/librealsense/build/`.
- opencv-python-headless, numpy, adafruit-servokit all installed in conda env.

## Development Environment

- macOS, Python 3.9.6 — must use `from __future__ import annotations` for `X | Y` unions
- venv: `.venv/`, install: `pip install -e ".[dev]"`
- Build backend: `setuptools.build_meta`
- Git remote: `git@github.com:liuchwhu/car.git`

## Architecture

```
Camera → Active Mode (wander/follow) → proposed ControlCommand
                                              │
                                    BehaviorManager.process()
                                              │
              DepthAnalyzer ──► DepthReport ──┤
                                              │
              Priority stack:                 │
              1. EmergencyStop (< 20cm)       │
              2. Recovery (stuck detection)   │
              3. ObstacleAvoidance (< 1m)     │
              4. Passthrough (use proposed)   │
                                              │
                                      Final ControlCommand → Actuator
```

## Implementation Status

- **Phases 1–5: DONE** — foundation, app, hardware drivers, calibration, behavior manager, wander mode
- **Phase 6: TODO** — object following (inference engine, object detector, PID, follow mode)
- **Phase 7: TODO** — tests + tooling

## Key Scripts

| Script | Purpose | Run on |
|--------|---------|--------|
| `scripts/test_servo.py` | Steering sweep + throttle pulse | Jetson |
| `scripts/test_hardware.py` | Normalized steering, dead zones, steering under throttle | Jetson |
| `scripts/test_camera.py` | D435 RGB+depth capture, 10 frames with stats | Jetson |
| `scripts/stream_camera.py` | MJPEG web stream (RGB + depth side-by-side) at `:8080` | Jetson |
| `scripts/find_center.py` | Interactive servo center discovery | Jetson |
| `scripts/find_limits.py` | Interactive steering limit discovery | Jetson |
| `scripts/calibrate.py` | Interactive + auto-calibration CLI | Jetson |

## Running on Jetson

All scripts must be run with `python3` (conda Python 3.7):
```bash
python3 ~/scripts/test_camera.py
python3 ~/scripts/stream_camera.py   # then open http://192.168.1.233:8080
```

## Conventions

- Config: OmegaConf YAML with merge chain (default → hardware → CLI overrides)
- Message bus: synchronous LocalBus (ROS2-style topic names for future migration)
- All steering/throttle values normalized to [-1, 1]
- Safety: BehaviorManager always sits between mode output and actuator
