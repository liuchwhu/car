# PilotNano - Self-Driving RC Car Project Scaffold

## Context
Scaffold a Python project for a self-driving RC car built on a Jetson Nano + Tamiya XV-02 chassis with PCA9685 PWM driver and Intel RealSense D435 camera. The project focuses on algorithmic autonomous driving — obstacle avoidance and object following using depth perception — with a behavior manager providing safety and decision arbitration. A lightweight message bus abstracts communication so ROS2 can be swapped in later.

## Hardware Notes
- **Battery**: 2S LiPo (7.4V nominal, 8.4V fully charged)
- **ESC**: Must be calibrated for 2S voltage range; throttle PWM mapping should account for lower voltage/speed ceiling
- **Power**: Jetson Nano requires 5V/2-4A — needs a BEC or separate 5V regulator from the 2S pack (do NOT power Jetson directly from 7.4V)
- **Speed**: 2S keeps top speed moderate, which is ideal for autonomous driving development
- **Servo limits**: The steering servo has physical angle limits — it cannot turn beyond a certain angle in each direction. These limits must be discovered during calibration and enforced in software (clamping) to prevent servo damage. The limits are saved as `steering_max_left` and `steering_max_right` in the calibration output file.

## Project Structure

```
pilotnano/
├── pyproject.toml                    # PEP 621, console entry point
├── Makefile                          # dev shortcuts (test, lint)
├── configs/
│   ├── default.yaml                  # top-level: loop_hz, log_level, sub-config refs
│   ├── hardware/
│   │   ├── jetson.yaml               # RealSense + PCA9685 settings (pins, PWM ranges, trim)
│   │   └── mock.yaml                 # Mock camera/actuator for desktop dev
│   ├── behavior/
│   │   └── default.yaml              # e-stop distance, stuck timeout, recovery maneuver params
│   ├── calibration/
│   │   ├── default.yaml              # auto-cal speeds, durations, tolerances
│   │   └── calibration_result.yaml   # auto-generated: saved calibration constants (do not edit)
│   └── follow/
│       └── default.yaml              # target class, follow distance, PID gains, lost-target policy
├── src/pilotnano/
│   ├── __init__.py
│   ├── app.py                        # CLI entry point, mode switching, main loop @ loop_hz
│   ├── config.py                     # OmegaConf load + merge (default → hardware → CLI)
│   ├── bus/
│   │   ├── __init__.py
│   │   ├── base.py                   # MessageBus ABC (publish, subscribe, unsubscribe)
│   │   ├── local.py                  # LocalBus: synchronous in-process dict[topic]→callbacks
│   │   └── messages.py               # Dataclasses: CameraFrame, ControlCommand + topic constants
│   ├── hardware/
│   │   ├── __init__.py               # Factory: create_camera(), create_actuator()
│   │   ├── base.py                   # Camera ABC (start/read/stop), Actuator ABC (start/send/stop)
│   │   ├── realsense.py              # pyrealsense2 pipeline, depth-to-color alignment
│   │   ├── pca9685.py                # adafruit ServoKit, normalized [-1,1] → PWM mapping
│   │   └── mock.py                   # MockCamera (synthetic frames), MockActuator (logs cmds)
│   ├── modes/
│   │   ├── __init__.py               # Factory: create_mode()
│   │   ├── base.py                   # DrivingMode ABC (start/step/stop)
│   │   ├── manual.py                 # Display camera feed, car driven by RC remote directly
│   │   ├── wander.py                 # Autonomous wandering: drive forward + obstacle avoidance
│   │   └── follow.py                 # Object following: detect target, PID to follow at distance
│   ├── behavior/
│   │   ├── __init__.py
│   │   ├── manager.py                # BehaviorManager: priority-based behavior arbitration
│   │   ├── base.py                   # Behavior ABC (evaluate → ControlCommand | None)
│   │   ├── emergency_stop.py         # Depth < threshold → full stop (highest priority)
│   │   ├── obstacle_avoidance.py     # Steer toward open space when obstacle within range
│   │   ├── recovery.py               # Stuck detection → reverse+turn state machine
│   │   └── passthrough.py            # Forwards the active mode's command (lowest priority)
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── depth_analyzer.py         # Depth map → obstacle zones (left/center/right distances)
│   │   └── object_detector.py        # YOLO/MobileNet-SSD via ONNX Runtime → bounding boxes
│   ├── inference/
│   │   ├── __init__.py
│   │   └── engine.py                 # ONNX Runtime session (auto-selects TensorRT>CUDA>CPU)
│   ├── control/
│   │   ├── __init__.py
│   │   └── pid.py                    # PID controller for steering and throttle
│   ├── calibration/
│   │   ├── __init__.py
│   │   ├── auto_calibrator.py        # Orchestrates all auto-calibration routines
│   │   ├── steering_calibrator.py    # Auto steering trim + steering-to-turn-rate curve
│   │   ├── throttle_calibrator.py    # Auto throttle-to-speed curve via depth deltas
│   │   └── motion_estimator.py       # Optical flow + depth deltas → actual velocity/turn rate
│   └── utils/
│       ├── __init__.py
│       └── logging.py                # Logging config
├── scripts/
│   └── calibrate.py                  # Interactive + auto-calibration CLI
├── tests/
│   ├── conftest.py                   # Fixtures: mock_camera, mock_actuator, local_bus, sample_cfg
│   ├── test_bus.py
│   ├── test_hardware_mock.py
│   ├── test_behavior.py
│   ├── test_depth_analyzer.py
│   ├── test_pid.py
│   ├── test_object_detector.py
│   └── test_calibration.py
└── models/
    └── .gitkeep                      # Place pretrained ONNX models here (yolo-nano.onnx, etc.)
```

## Key Design Decisions

| Decision | Choice | Why |
|----------|--------|-----|
| Config library | OmegaConf (not Hydra) | Lighter, no CLI magic needed, same YAML merge |
| Message bus | Synchronous LocalBus | 20Hz loop fits in 50ms budget; async adds complexity with no benefit now |
| Behavior safety | Priority-based behavior manager | Always-on safety layer between decision-making and actuators; prevents crashes regardless of driving mode |
| Object detection | Pretrained YOLO-Nano or MobileNet-SSD (ONNX) | No custom training needed — download a pretrained model, convert to ONNX, deploy |
| Inference | ONNX Runtime (auto provider) | Portable across Jetson (TensorRT EP) and desktop (CPU) |
| Control | PID controllers | Simple, tunable, no ML needed — appropriate for following and steering tasks |

---

## Implementation Plan

### Phase 1: Foundation

**Goal**: Build the skeleton that everything else plugs into — config loading, message passing, and hardware abstractions. After this phase, you can run `pilotnano --mode manual` on your laptop with mock hardware and see the main loop ticking.

**Why start here**: Every subsequent phase depends on config, the bus, and hardware interfaces. Getting these right first means we never have to refactor the core wiring later. The mock hardware layer is critical — it lets us develop and test all driving modes on a laptop without touching the Jetson or the car.

**What we build**:

1. **`pyproject.toml`** — Project packaging with PEP 621 metadata. Defines `pilotnano` as the console entry point. Core dependencies: numpy, opencv-python, omegaconf, pyrealsense2, adafruit-circuitpython-servokit, onnxruntime.

2. **`configs/`** — YAML files for every tunable parameter. `default.yaml` sets the main loop rate (20Hz), log level, and references which hardware sub-config to use. `hardware/jetson.yaml` holds RealSense resolution/FPS, PCA9685 I2C address, servo/ESC channel numbers, PWM microsecond ranges (1000-2000μs), steering trim offset, and 2S-specific throttle limits. `hardware/mock.yaml` mirrors the same keys but with `type: mock`.

3. **`src/pilotnano/config.py`** — A single `load_config()` function that uses OmegaConf to load `default.yaml`, then merges the referenced hardware sub-config on top, then applies any CLI dot-notation overrides (e.g., `hardware.actuator.steering_trim=0.05`). No framework magic — just explicit `OmegaConf.load()` and `OmegaConf.merge()` calls.

4. **`src/pilotnano/bus/`** — The message bus abstraction. `messages.py` defines dataclasses (`CameraFrame`, `ControlCommand`) and string topic constants (`"camera/frame"`, `"control/command"`, etc.) using ROS2-style naming so a future `ROS2Bus` can map them 1:1. `base.py` defines the `MessageBus` ABC with `publish()`, `subscribe()`, and `unsubscribe()`. `local.py` implements `LocalBus` — a synchronous in-process pub/sub using `dict[str, list[Callable]]`. When you `publish()`, it calls all subscribers sequentially in the caller's thread. No threading, no queues — the 20Hz main loop is our only scheduler.

5. **`src/pilotnano/hardware/base.py`** — Abstract base classes for `Camera` (start/read/stop, returns `CameraFrame`) and `Actuator` (start/send/stop, accepts `ControlCommand`). Both support context manager (`with camera:` / `with actuator:`). These interfaces are the contract — every real and mock implementation must follow them.

6. **`src/pilotnano/hardware/mock.py`** — `MockCamera` returns synthetic frames (solid color gradient with a frame counter overlay, includes a fake depth map with configurable obstacle placement). `MockActuator` logs commands to stdout and stores them in a list (useful for test assertions). These let us run the full pipeline on a laptop.

7. **`src/pilotnano/hardware/__init__.py`** — Factory functions `create_camera(cfg)` and `create_actuator(cfg)` that read `cfg.camera.type` / `cfg.actuator.type` and return the right concrete class. Simple `if/elif` — two implementations don't justify a plugin registry.

8. **`src/pilotnano/modes/base.py` + `manual.py`** — `DrivingMode` ABC defines the `start()/step()/stop()` lifecycle that the main loop calls. `ManualMode` is the simplest mode: each `step()` reads a camera frame and displays it with OpenCV `imshow`. The car is driven by the RC remote directly (standard RC wiring). This mode exists purely for verifying the camera works and eyeballing the feed.

**Milestone**: `pilotnano --mode manual --hardware mock` runs on your laptop, showing synthetic frames at 20Hz in an OpenCV window.

---

### Phase 2: Main App + Real Hardware Drivers

**Goal**: Wire up the entry point and make the car's actual hardware respond. After this phase, you can SSH into the Jetson, run `pilotnano --mode manual`, and see live RealSense RGB+depth on screen while the servo/ESC are ready to accept commands.

**Why now**: Phase 1 gave us interfaces and mocks. Now we write the real implementations and the main app that ties everything together. We do this before the behavior manager because we need to verify the hardware works before building safety logic on top of it.

**What we build**:

9. **`src/pilotnano/app.py`** — The `main()` function that is the `pilotnano` console entry point. It parses CLI args (`--mode`, `--config`, plus any OmegaConf overrides), calls `load_config()`, creates the `LocalBus`, creates Camera and Actuator via factories, creates the requested `DrivingMode`, then runs a `while mode.running:` loop at `cfg.loop_hz`. The loop calls `mode.step()` and sleeps for the remainder of the tick. Ctrl+C triggers `mode.stop()` via a `try/finally` block.

10. **`src/pilotnano/hardware/realsense.py`** — `RealSenseCamera` wraps the `pyrealsense2` pipeline. `start()` configures and enables RGB + depth streams at the resolution/FPS from config, optionally creates an `rs.align` object to align depth frames to the RGB frame. `read()` calls `pipeline.wait_for_frames()`, extracts numpy arrays, and returns a `CameraFrame`. `stop()` cleanly shuts down the pipeline. Handles USB 3.0 vs 2.0 gracefully (logs a warning if bandwidth is limited).

11. **`src/pilotnano/hardware/pca9685.py`** — `PCA9685Actuator` wraps Adafruit ServoKit. `start()` initializes `ServoKit(channels=16, address=cfg.i2c_address)`, configures pulse width ranges from config, and loads the saved calibration file (`configs/calibration/calibration_result.yaml`) if it exists — this provides steering trim, steering limits, and throttle/steering lookup curves. `send()` maps normalized steering [-1, 1] to servo angle and throttle [-1, 1] to ESC pulse width, applying the steering trim offset and **clamping steering to the discovered physical limits** (`steering_max_left`, `steering_max_right`) to prevent servo damage. If calibration curves are available, it translates desired physical units (speed, turn rate) into correct PWM values. `stop()` sends a neutral command (steering center, throttle zero) as a safety measure.

12. **`src/pilotnano/utils/logging.py`** — Sets up Python `logging` with a consistent format showing timestamp, level, and module name. Reads `log_level` from config.

13. **`scripts/calibrate.py`** — Interactive CLI tool for servo and ESC calibration. Walks you through: setting steering left/center/right positions (maps physical limits to PWM values), arming the ESC (2S-specific procedure: neutral → full throttle → neutral), and setting throttle range. Writes the calibrated values to a YAML file that can be referenced from the main config. This is essential for the 2S setup since PWM ranges vary between ESCs.

**Milestone**: On the Jetson with everything wired up, `pilotnano --mode manual` shows live camera feed. Running `scripts/calibrate.py` in manual mode verifies the servo sweeps left-right and the ESC responds.

---

### Phase 3: Auto-Calibration

**Goal**: Use the D435 camera as a measurement instrument to automatically calibrate the car's steering and throttle. After this phase, the car knows exactly what steering angle produces what turn rate, and what throttle value produces what speed — no guesswork, no manual tuning.

**Why now**: Accurate calibration is essential for the PID controllers in object following (Phase 5). Without it, `steering=0.5` is a meaningless number — we don't know if it produces a gentle curve or a sharp turn. Auto-calibration turns arbitrary PWM values into physically meaningful units (degrees/second, meters/second). It also eliminates the tedious manual calibration step — the car calibrates itself in ~30 seconds.

**What we build**:

14. **`src/pilotnano/calibration/motion_estimator.py`** — `MotionEstimator` measures the car's actual motion using the D435. Two methods:
- **Speed from depth**: When facing a wall/surface, measure the rate of change of center depth between frames. `speed = Δdepth / Δtime`. Accurate to ~1cm at D435 ranges.
- **Turn rate from optical flow**: Compute sparse optical flow (OpenCV `calcOpticalFlowPyrLK`) between consecutive RGB frames. The dominant horizontal flow direction and magnitude indicate the car's yaw rate. When driving forward, lateral flow asymmetry indicates steering bias (drift).
- Both methods work without any additional sensors — the D435 is the only instrument needed.

15. **`src/pilotnano/calibration/steering_calibrator.py`** — `SteeringCalibrator` runs two auto-calibration routines:
- **Trim calibration**: Drives forward slowly at a wall (`throttle=0.15`, `steering=0.0`). Measures drift via optical flow — if the scene shifts right, the car is drifting left. Adjusts `steering_trim` in small increments, re-tests, converges on the trim value that produces straight-line driving. Takes ~10 seconds, 3-5 iterations.
- **Steering curve**: Sweeps through steering values (-1.0 to 1.0 in steps of 0.2) while driving slowly forward. At each value, measures the actual turn rate from optical flow. Builds a lookup table `{commanded_steering: actual_turn_rate_deg_per_sec}`. This lets the PID convert a desired turn rate into the correct steering command. Takes ~20 seconds.

16. **`src/pilotnano/calibration/throttle_calibrator.py`** — `ThrottleCalibrator` measures actual speed at different throttle values:
- Faces a flat surface (wall) from ~3 meters. Commands a throttle value for 1 second, measures approach speed from depth deltas. Stops, reverses to starting position. Repeats at different throttle values (0.1, 0.2, 0.3, ... up to max safe speed).
- Builds a lookup table `{commanded_throttle: actual_speed_m_per_sec}`. This lets the PID command real speeds instead of arbitrary throttle values.
- Also measures the ESC's dead zone (minimum throttle that actually produces motion — important for 2S where it might be higher than expected).
- Takes ~30 seconds.

17. **`src/pilotnano/calibration/auto_calibrator.py`** — `AutoCalibrator` orchestrates all calibration routines in sequence. Runs as `pilotnano --mode calibrate` or `scripts/calibrate.py --auto`. Sequence:
1. Check for existing calibration file (`configs/calibration/calibration_result.yaml`). If it exists and is valid, skip calibration and log "Using saved calibration from <timestamp>". Use `--force-calibrate` flag to override.
2. Steering trim calibration → measures `steering_trim`.
3. Steering curve calibration → measures lookup table `{commanded_steering: actual_turn_rate_deg_per_sec}`.
4. Steering limit discovery → slowly sweeps steering toward each extreme, detects physical limit by monitoring servo current draw or depth-based motion stall. Saves `steering_max_left` and `steering_max_right` (the usable range, not the servo's full range). These limits are enforced as clamps in `PCA9685Actuator.send()`.
5. Throttle speed calibration → measures lookup table `{commanded_throttle: actual_speed_m_per_sec}`.
6. Saves ALL results to `configs/calibration/calibration_result.yaml` — a single persistent file containing steering_trim, steering_curve, steering_limits, throttle_curve, ESC dead zone, and a timestamp. This file is loaded by `PCA9685Actuator` at startup.
7. Writes a human-readable `calibration_report.json` with all measured values and pass/fail status.
- **Persistent calibration**: The result file is saved once and reused on every subsequent run. No need to re-calibrate every time you start the car. Re-run calibration only when something changes (new tires, different surface, battery type).
- The lookup tables are loaded by `PCA9685Actuator` at startup and used to translate desired physical values (turn rate, speed) into the correct PWM commands.

18. **`configs/calibration/default.yaml`** — Configuration for auto-calibration:
```yaml
steering_trim:
  test_throttle: 0.15       # gentle forward speed during trim test
  test_duration: 1.0        # seconds per trim test
  max_iterations: 10        # convergence limit
  tolerance: 0.02           # acceptable drift (normalized flow)
steering_curve:
  test_throttle: 0.15
  step_duration: 0.5        # seconds at each steering value
  steps: [-1.0, -0.6, -0.2, 0.0, 0.2, 0.6, 1.0]
throttle_curve:
  target_distance: 3.0      # meters — start distance from wall
  test_duration: 1.0        # seconds at each throttle value
  steps: [0.1, 0.15, 0.2, 0.25, 0.3, 0.4]
```

**Milestone**: Place the car 3 meters from a wall. Run `pilotnano --mode calibrate`. The car automatically: discovers its steering limits, finds its steering trim, sweeps steering values to build a turn-rate curve, tests throttle values to build a speed curve. All results are saved to `configs/calibration/calibration_result.yaml`. On the next startup, the car loads the saved calibration instantly — no re-calibration needed. Use `--force-calibrate` to re-run if hardware changes. Subsequent modes (wander, follow) use these curves for accurate control.

---

### Phase 4: Behavior Manager + Depth Perception

**Goal**: Build the safety and decision-arbitration layer that sits between any driving mode and the actuators. After this phase, the car has an always-on safety system: emergency stop when something is too close, and automatic recovery when stuck. Every driving mode routes through this layer.

**Why now**: Before we enable any autonomous driving, we need a safety net. Building it now means every subsequent mode (wander, follow) gets safety for free. It also introduces the depth analyzer — the D435's key advantage over a regular webcam.

**What we build**:

19. **`src/pilotnano/perception/depth_analyzer.py`** — `DepthAnalyzer` processes the depth frame into actionable information. Divides the depth image into zones (left third, center third, right third), computes the minimum reliable distance in each zone (filtering out zero/invalid depth pixels), and returns a `DepthReport` dataclass with `left_dist`, `center_dist`, `right_dist`, and `closest_dist`. All thresholds and zone boundaries are configurable. This is the shared perception component that emergency stop, recovery, obstacle avoidance, and object following all build on.

20. **`src/pilotnano/behavior/base.py`** — `Behavior` ABC that all behaviors implement. Key method: `evaluate(frame: CameraFrame, depth_report: DepthReport, proposed_cmd: ControlCommand) -> ControlCommand | None`. Returns a `ControlCommand` to override the proposed action, or `None` to abstain (let lower-priority behaviors decide). Each behavior also has a `priority` (int, lower = higher priority) and a `name` for logging.

21. **`src/pilotnano/behavior/emergency_stop.py`** — `EmergencyStopBehavior`. Highest priority. If `depth_report.closest_dist < e_stop_distance` (configurable, default 0.2m / ~8 inches), returns `ControlCommand(steering=0, throttle=0)` — full stop. Publishes an alert to the bus. This is a hard safety boundary that no other behavior can override. Uses a small hysteresis (resume threshold slightly higher than stop threshold) to prevent flickering.

22. **`src/pilotnano/behavior/recovery.py`** — `RecoveryBehavior`. Second priority. Implements a state machine for getting unstuck:
- **MONITORING** → detects "stuck" condition: throttle has been positive but `depth_report.center_dist` hasn't changed for N consecutive frames (configurable, default 1 second / 20 frames). This catches situations where the car is pushing against a wall.
- **REVERSING** → sends `throttle=-0.3` (gentle reverse) for a configurable duration (default 1 second).
- **TURNING** → sends `steering=±1.0` (full lock toward the side with most depth clearance, determined from `depth_report.left_dist` vs `depth_report.right_dist`) + gentle forward throttle for a configurable duration (default 0.5 seconds).
- **RESUMING** → returns to MONITORING, hands control back to the active driving mode.
- All durations, throttle values, and thresholds are in `configs/behavior/default.yaml`.

23. **`src/pilotnano/behavior/passthrough.py`** — `PassthroughBehavior`. Lowest priority. Simply returns the `proposed_cmd` unchanged. This is the fallback — if no higher-priority behavior intervenes, the active mode's command goes through to the actuator.

24. **`src/pilotnano/behavior/manager.py`** — `BehaviorManager` orchestrates the priority stack. Holds a sorted list of `Behavior` instances. On each tick, `process(frame, proposed_cmd) -> ControlCommand`:
1. Runs `DepthAnalyzer` on the frame's depth data to get a `DepthReport`.
2. Iterates through behaviors in priority order (highest first).
3. The first behavior to return a non-`None` command wins — that command is sent to the actuator.
4. Logs which behavior is currently in control (useful for debugging: "emergency_stop ACTIVE" vs "passthrough").

The manager is instantiated in `app.py` and injected into all driving modes.

25. **`configs/behavior/default.yaml`** — Configuration for all behavior parameters:
```yaml
emergency_stop:
  distance: 0.20          # meters — stop if anything closer
  resume_distance: 0.35   # meters — resume when clear
obstacle_avoidance:
  trigger_distance: 1.0   # meters — start steering away
  steer_gain: 0.8         # how aggressively to steer (0-1)
recovery:
  stuck_timeout: 1.0      # seconds of no progress before triggering
  reverse_duration: 1.0   # seconds to reverse
  reverse_throttle: -0.3
  turn_duration: 0.5      # seconds to turn
  turn_throttle: 0.2
```

**Data flow with behavior manager**:
```
Camera → Active Mode (wander/follow) → proposed ControlCommand
                                              │
                                              ▼
                                    BehaviorManager.process()
                                              │
              DepthAnalyzer ──► DepthReport ──┤
                                              │
              Priority stack:                 │
              1. EmergencyStop (< 20cm)  ─────┤
              2. Recovery (stuck?)       ─────┤
              3. ObstacleAvoidance (< 1m) ────┤
              4. Passthrough (use proposed) ──┤
                                              │
                                              ▼
                                      Final ControlCommand → Actuator
```

**Milestone**: On the Jetson, push the car toward a wall — emergency stop kicks in. Test with mock hardware on laptop by simulating decreasing depth values.

---

### Phase 5: Obstacle Avoidance + Wander Mode

**Goal**: Make the car drive autonomously by wandering forward and steering around obstacles using depth data. No ML, no training data — just depth perception and reactive control. This is the first fully autonomous mode.

**Why now**: With the behavior manager and depth analyzer in place, obstacle avoidance is straightforward — it's just another behavior in the priority stack. The wander mode provides a simple "drive forward" policy that the behavior stack makes safe. This gives you an autonomous car with minimal complexity.

**What we build**:

26. **`src/pilotnano/behavior/obstacle_avoidance.py`** — `ObstacleAvoidanceBehavior`. Third priority (after e-stop and recovery, before passthrough). When `depth_report.center_dist < trigger_distance` (configurable, default 1.0m), steers toward the side with more clearance: compares `left_dist` vs `right_dist`, produces a steering command proportional to the imbalance (controlled by `steer_gain`). Throttle is reduced proportionally to proximity — the closer the obstacle, the slower the car goes. If both sides are blocked, it returns `None` and lets recovery handle it (reverse + turn). This is the active avoidance layer — unlike emergency stop (which halts), this steers around obstacles while keeping the car moving.

27. **`src/pilotnano/modes/wander.py`** — `WanderMode` is the simplest autonomous driving mode. Each `step()`: reads a camera frame, creates a `ControlCommand` with `steering=0.0` (straight) and `throttle=cfg.wander_throttle` (configurable, default 0.3 — gentle forward), passes it through the `BehaviorManager`, and sends the result to the actuator. The mode itself just says "go forward" — the behavior stack handles everything else (avoid obstacles, stop for walls, recover from stuck). This is intentionally dumb — the intelligence lives in the behaviors.

**Milestone**: Place the car in an open room with furniture. Run `pilotnano --mode wander`. The car drives forward, steers around chair legs and walls, reverses out of corners, and keeps exploring indefinitely. No training, no data collection — just depth + reactive behaviors.

---

### Phase 6: Object Following

**Goal**: Make the car detect and follow a specific object (person, ball, another car) at a set distance using a pretrained object detector and the D435's depth stream. After this phase, you have a "follow me" car.

**Why now**: The behavior manager is handling safety. The inference engine (for loading ONNX models) is needed here for the object detector. The depth analyzer provides range to the target. This is the most compelling demo capability — a car that follows you around.

**What we build**:

28. **`src/pilotnano/inference/engine.py`** — `InferenceEngine` wraps an ONNX Runtime `InferenceSession`. On init, it auto-detects available execution providers and prefers TensorRT (fastest on Jetson) > CUDA > CPU. Loads the ONNX model, reads input/output tensor names from metadata. `predict()` takes a preprocessed numpy array, runs inference, and returns raw outputs. On the Jetson Nano with TensorRT, inference takes ~10-15ms per frame for a small detection model — within our 50ms budget.

29. **`src/pilotnano/perception/object_detector.py`** — `ObjectDetector` wraps the inference engine for object detection specifically. Takes a pretrained YOLO-Nano or MobileNet-SSD ONNX model (placed in `models/` directory). `detect(rgb_frame) -> list[Detection]` where `Detection` is a dataclass with `class_name`, `confidence`, `bbox (x1, y1, x2, y2)`. Handles preprocessing (resize to model input, normalize) and postprocessing (NMS, confidence filtering) internally. The target class to follow is configurable (e.g., `"person"`, `"sports ball"`).

30. **`src/pilotnano/control/pid.py`** — `PIDController` with configurable `Kp`, `Ki`, `Kd`, output clamping, and anti-windup. Two instances are used in follow mode: one for steering (error = horizontal offset of target from frame center) and one for throttle (error = current distance minus target distance). Gains are in `configs/follow/default.yaml`.

31. **`src/pilotnano/modes/follow.py`** — `FollowMode` is the object-following driving mode. Each `step()`:
1. Read camera frame (RGB + depth).
2. Run `ObjectDetector.detect()` on the RGB frame.
3. Filter for the target class, pick the highest-confidence detection.
4. If target found:
   - Compute bearing: `(bbox_center_x - frame_center_x) / frame_width` → normalized error [-1, 1].
   - Compute range: read depth value at bbox center from the aligned depth frame.
   - PID steering: steer to center the target in frame.
   - PID throttle: accelerate/decelerate to maintain target distance (configurable, default 1.5m).
   - Create `ControlCommand(steering, throttle)`.
5. If target lost:
   - Configurable policy: `stop` (halt and wait), `search` (slow rotation to scan), or `last_known` (drive toward last known position for N seconds then stop).
6. Pass command through `BehaviorManager` (so e-stop + obstacle avoidance still protect the car).
7. Send to actuator.

32. **`configs/follow/default.yaml`** — Configuration for follow mode:
```yaml
detector:
  model_path: models/yolo-nano.onnx
  target_class: person
  confidence_threshold: 0.5
follow:
  target_distance: 1.5    # meters — desired distance to target
  max_throttle: 0.4       # cap speed while following
  lost_target_policy: stop # stop | search | last_known
  lost_target_timeout: 3.0 # seconds before giving up search
pid_steering:
  kp: 0.8
  ki: 0.0
  kd: 0.1
pid_throttle:
  kp: 0.5
  ki: 0.05
  kd: 0.1
```

**Milestone**: Download a pretrained YOLO-Nano ONNX model, place it in `models/`. Run `pilotnano --mode follow`. Stand in front of the car — it detects you, maintains 1.5m distance, and follows as you walk. Step behind a wall — the car stops (or slowly searches). Walk toward the car — it backs up. Obstacle avoidance prevents it from crashing into furniture while following.

---

### Phase 7: Tests + Developer Tooling

**Goal**: Add automated tests and quality-of-life tools for iterating on the project. After this phase, the project is robust, testable, and pleasant to work with.

**Why last**: Tests and tooling are important but they don't block any functionality. By building them after the core pipeline works end-to-end, we can write tests that reflect the actual architecture rather than guessing.

**What we build**:

33. **`tests/`** — `conftest.py` provides shared fixtures: `mock_camera`, `mock_actuator`, `local_bus`, `sample_config`, and `depth_report_factory` (for generating test DepthReports). Individual test files:
- `test_bus.py` — LocalBus subscribe/publish/unsubscribe, multiple subscribers, topic isolation.
- `test_hardware_mock.py` — MockCamera returns valid CameraFrame with correct shape, MockActuator records commands.
- `test_behavior.py` — EmergencyStop triggers at threshold, Recovery state machine transitions (MONITORING→REVERSING→TURNING→RESUMING), ObstacleAvoidance steers toward open side, BehaviorManager respects priority ordering, Passthrough forwards commands unchanged.
- `test_depth_analyzer.py` — Zone splitting, invalid pixel filtering, min-distance computation with known depth maps.
- `test_pid.py` — PID output correctness, anti-windup, output clamping.
- `test_object_detector.py` — Detector loads an ONNX model, returns Detection dataclasses with correct fields (can use a tiny test model).

34. **`Makefile`** — Developer shortcuts: `make test` (pytest), `make lint` (ruff check), `make format` (ruff format), `make wander` (runs pilotnano in wander mode), `make follow` (runs pilotnano in follow mode).

**Milestone**: `pytest tests/` passes on both laptop and Jetson. `make test && make lint` gives a clean bill of health.

---

## Libraries

| Component | Library | Install scope |
|-----------|---------|--------------|
| Camera | `pyrealsense2` | Jetson + dev |
| Servo/ESC | `adafruit-circuitpython-servokit` | Jetson only |
| Config | `omegaconf` | All |
| Inference | `onnxruntime` (or `onnxruntime-gpu`) | All |
| Image processing | `opencv-python`, `numpy` | All |
| Object detection | Pretrained YOLO-Nano / MobileNet-SSD (ONNX) | Downloaded, not trained |

## Verification

1. **Desktop (no hardware)**: `pilotnano --mode manual --hardware mock` — shows synthetic camera frames
2. **Jetson (hardware)**: `pilotnano --mode manual` — shows live RealSense feed, servo responds via calibrate script
3. **Obstacle avoidance**: `pilotnano --mode wander` — car drives forward, steers around furniture, reverses out of corners
4. **Object following**: `pilotnano --mode follow` — car follows a person at 1.5m distance, stops when target lost
5. **Tests**: `pytest tests/` — all pass with mock hardware

---

## What's Next: Projects After the Scaffold

Once the base project is working, the architecture supports many extensions:

### 1. Behavioral Cloning (End-to-End ML Driving)
- **What**: Add data collection mode (gamepad driving while recording camera + labels), train a PilotNet CNN to imitate your driving, deploy as an autopilot mode. The full collect → train → drive pipeline.
- **Add**: `modes/collect.py`, `modes/autopilot.py`, `data/recorder.py`, `data/dataset.py`, `data/augmentation.py`, `model/pilotnet.py`, `model/export.py`, `perception/preprocessor.py`, `scripts/train.py`
- **When useful**: When you want the car to learn a specific track/route that's hard to describe with rules

### 2. Lane Following with Computer Vision
- **What**: Use classical CV (Canny edge detection, Hough line transform, or color thresholding) to detect lane lines or track edges, then use the PID controller to steer along them. No neural network needed.
- **Add**: `perception/lane_detector.py`, `modes/lane_follow.py`
- **Depth data used**: No (RGB only)

### 3. Waypoint Navigation with Visual Odometry
- **What**: Drive to a sequence of waypoints using the RealSense for visual odometry (estimating position from frame-to-frame motion). No GPS needed — suitable for indoor or small outdoor areas.
- **Add**: `perception/visual_odometry.py`, `planning/waypoint_planner.py`, `modes/navigate.py`
- **Depth data used**: Yes — stereo depth improves visual odometry accuracy

### 4. SLAM + Autonomous Exploration
- **What**: Build a map of the environment while driving (Simultaneous Localization and Mapping), then autonomously explore unmapped areas. Uses depth + RGB for mapping.
- **Add**: Integration with ORB-SLAM3 or RTAB-Map. The bus architecture makes it easy to add a SLAM node that subscribes to camera frames and publishes a map/pose.
- **Depth data used**: Yes — essential for 3D mapping

### 5. Multi-Modal Driving (RGB + Depth Fusion)
- **What**: Train a neural network that takes both RGB and depth as input for more robust autonomous driving. The depth channel helps the model understand 3D scene geometry, especially in challenging lighting.
- **Add**: Modify PilotNet to accept 4-channel input (RGB + depth) or use a dual-encoder architecture.
- **Depth data used**: Yes — depth as an additional input channel to the neural network

### 6. ROS2 Migration
- **What**: Swap the `LocalBus` for a `ROS2Bus` adapter, turning each component into a ROS2 node. This enables distributed processing, RViz visualization, rosbag recording, and integration with the broader ROS ecosystem.
- **Add**: `bus/ros2.py` — maps topic strings to ROS2 topics, wraps `rclpy.Publisher` and `rclpy.Subscription`. No business logic changes needed.
