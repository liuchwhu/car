# PilotNano - Self-Driving RC Car Project Scaffold

## Context
Scaffold a Python project for a self-driving RC car built on a Jetson Nano + Tamiya XV-02 chassis with PCA9685 PWM driver and Intel RealSense D435 camera. The project should support behavioral cloning (collect data, train, drive autonomously) and be structured to grow into a modular perception-planning-control pipeline. A lightweight message bus abstracts communication so ROS2 can be swapped in later.

## Hardware Notes
- **Battery**: 2S LiPo (7.4V nominal, 8.4V fully charged)
- **ESC**: Must be calibrated for 2S voltage range; throttle PWM mapping should account for lower voltage/speed ceiling
- **Power**: Jetson Nano requires 5V/2-4A — needs a BEC or separate 5V regulator from the 2S pack (do NOT power Jetson directly from 7.4V)
- **Speed**: 2S keeps top speed moderate, which is ideal for autonomous driving development

## Project Structure

```
pilotnano/
├── pyproject.toml                    # PEP 621, console entry point, optional [train] deps
├── Makefile                          # dev shortcuts (test, lint, export)
├── configs/
│   ├── default.yaml                  # top-level: loop_hz, log_level, sub-config refs
│   ├── hardware/
│   │   ├── jetson.yaml               # RealSense + PCA9685 settings (pins, PWM ranges, trim)
│   │   └── mock.yaml                 # Mock camera/actuator for desktop dev
│   ├── model/
│   │   └── pilotnet.yaml             # input_shape, lr, epochs, batch_size
│   └── collection/
│       └── default.yaml              # output_dir, image format, save_depth flag
├── src/pilotnano/
│   ├── __init__.py
│   ├── app.py                        # CLI entry point, mode switching, main loop @ loop_hz
│   ├── config.py                     # OmegaConf load + merge (default → hardware → model → CLI)
│   ├── bus/
│   │   ├── __init__.py
│   │   ├── base.py                   # MessageBus ABC (publish, subscribe, unsubscribe)
│   │   ├── local.py                  # LocalBus: synchronous in-process dict[topic]→callbacks
│   │   └── messages.py               # Dataclasses: CameraFrame, ControlCommand, ModelOutput + topic constants
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
│   │   ├── collect.py                # Gamepad→actuator + recorder (camera frame + labels to disk)
│   │   └── autopilot.py              # Camera→preprocess→inference→actuator
│   ├── perception/
│   │   ├── __init__.py
│   │   └── preprocessor.py           # Crop, resize(200x66), normalize, BGR→RGB, HWC→CHW
│   ├── model/
│   │   ├── __init__.py
│   │   ├── pilotnet.py               # PilotNet CNN (PyTorch): 5 conv + 4 FC → (steering, throttle)
│   │   └── export.py                 # torch.onnx.export helper
│   ├── inference/
│   │   ├── __init__.py
│   │   └── engine.py                 # ONNX Runtime session (auto-selects TensorRT>CUDA>CPU)
│   ├── data/
│   │   ├── __init__.py
│   │   ├── recorder.py               # Writes frames/*.jpg + labels.csv per run
│   │   ├── dataset.py                # PyTorch Dataset loading recorded runs
│   │   └── augmentation.py           # Horizontal flip, brightness, shadow, translation
│   └── utils/
│       ├── __init__.py
│       └── logging.py                # Logging config
├── scripts/
│   ├── train.py                      # Offline training (runs on workstation, not Jetson)
│   ├── export_onnx.py                # Checkpoint → ONNX export
│   ├── visualize_data.py             # Browse frames with steering/throttle overlay
│   └── calibrate.py                  # Interactive servo/ESC range calibration
├── tests/
│   ├── conftest.py                   # Fixtures: mock_camera, mock_actuator, local_bus, sample_cfg
│   ├── test_bus.py
│   ├── test_hardware_mock.py
│   ├── test_recorder.py
│   ├── test_model.py
│   └── test_inference.py
└── notebooks/
    └── explore_data.ipynb
```

## Key Design Decisions

| Decision | Choice | Why |
|----------|--------|-----|
| Config library | OmegaConf (not Hydra) | Lighter, no CLI magic needed, same YAML merge |
| Message bus | Synchronous LocalBus | 20Hz loop fits in 50ms budget; async adds complexity with no benefit now |
| Data format | JPG images + CSV labels | Human-browsable, debuggable, small dataset volumes |
| Model | PilotNet (DAVE-2 variant) | Proven for RC cars, tiny enough for Jetson Nano |
| Inference | ONNX Runtime (auto provider) | Portable across Jetson (TensorRT EP) and desktop (CPU) |
| Actuator control | Gamepad for collect mode | Simpler than reading RC PWM; software controls PCA9685 directly |

---

## Implementation Plan

### Phase 1: Foundation

**Goal**: Build the skeleton that everything else plugs into — config loading, message passing, and hardware abstractions. After this phase, you can run `pilotnano --mode manual` on your laptop with mock hardware and see the main loop ticking.

**Why start here**: Every subsequent phase depends on config, the bus, and hardware interfaces. Getting these right first means we never have to refactor the core wiring later. The mock hardware layer is critical — it lets us develop and test all driving modes on a laptop without touching the Jetson or the car.

**What we build**:

1. **`pyproject.toml`** — Project packaging with PEP 621 metadata. Defines `pilotnano` as the console entry point. Core dependencies (numpy, opencv, omegaconf, pyrealsense2, servokit, onnxruntime) go in `[dependencies]`. Training-only libraries (torch, torchvision, onnx, matplotlib, tqdm) go in `[project.optional-dependencies.train]` so the Jetson doesn't need to install them.

2. **`configs/`** — YAML files for every tunable parameter. `default.yaml` sets the main loop rate (20Hz), log level, and references which hardware/model/collection sub-config to use. `hardware/jetson.yaml` holds RealSense resolution/FPS, PCA9685 I2C address, servo/ESC channel numbers, PWM microsecond ranges (1000-2000μs), steering trim offset, and 2S-specific throttle limits. `hardware/mock.yaml` mirrors the same keys but with `type: mock`. `model/pilotnet.yaml` defines input shape (3x66x200), learning rate, batch size, epochs. `collection/default.yaml` sets output directory, image format (jpg), and whether to save depth frames.

3. **`src/pilotnano/config.py`** — A single `load_config()` function that uses OmegaConf to load `default.yaml`, then merges the referenced hardware/model/collection sub-configs on top, then applies any CLI dot-notation overrides (e.g., `hardware.actuator.steering_trim=0.05`). No framework magic — just explicit `OmegaConf.load()` and `OmegaConf.merge()` calls.

4. **`src/pilotnano/bus/`** — The message bus abstraction. `messages.py` defines dataclasses (`CameraFrame`, `ControlCommand`, `ModelOutput`) and string topic constants (`"camera/frame"`, `"control/command"`, etc.) using ROS2-style naming so a future `ROS2Bus` can map them 1:1. `base.py` defines the `MessageBus` ABC with `publish()`, `subscribe()`, and `unsubscribe()`. `local.py` implements `LocalBus` — a synchronous in-process pub/sub using `dict[str, list[Callable]]`. When you `publish()`, it calls all subscribers sequentially in the caller's thread. No threading, no queues — the 20Hz main loop is our only scheduler.

5. **`src/pilotnano/hardware/base.py`** — Abstract base classes for `Camera` (start/read/stop, returns `CameraFrame`) and `Actuator` (start/send/stop, accepts `ControlCommand`). Both support context manager (`with camera:` / `with actuator:`). These interfaces are the contract — every real and mock implementation must follow them.

6. **`src/pilotnano/hardware/mock.py`** — `MockCamera` returns synthetic frames (solid color gradient with a frame counter overlay). `MockActuator` logs commands to stdout and stores them in a list (useful for test assertions). These let us run the full pipeline on a laptop.

7. **`src/pilotnano/hardware/__init__.py`** — Factory functions `create_camera(cfg)` and `create_actuator(cfg)` that read `cfg.camera.type` / `cfg.actuator.type` and return the right concrete class. Simple `if/elif` — two implementations don't justify a plugin registry.

8. **`src/pilotnano/modes/base.py` + `manual.py`** — `DrivingMode` ABC defines the `start()/step()/stop()` lifecycle that the main loop calls. `ManualMode` is the simplest mode: each `step()` reads a camera frame and displays it with OpenCV `imshow`. The car is driven by the RC remote directly (standard RC wiring). This mode exists purely for verifying the camera works and eyeballing the feed.

**Milestone**: `pilotnano --mode manual --hardware mock` runs on your laptop, showing synthetic frames at 20Hz in an OpenCV window.

---

### Phase 2: Main App + Real Hardware Drivers

**Goal**: Wire up the entry point and make the car's actual hardware respond. After this phase, you can SSH into the Jetson, run `pilotnano --mode manual`, and see live RealSense RGB+depth on screen while the servo/ESC are ready to accept commands.

**Why now**: Phase 1 gave us interfaces and mocks. Now we write the real implementations and the main app that ties everything together. We do this before data collection because we need to verify the hardware works before trusting it to record training data.

**What we build**:

9. **`src/pilotnano/app.py`** — The `main()` function that is the `pilotnano` console entry point. It parses CLI args (`--mode`, `--config`, plus any OmegaConf overrides), calls `load_config()`, creates the `LocalBus`, creates Camera and Actuator via factories, creates the requested `DrivingMode`, then runs a `while mode.running:` loop at `cfg.loop_hz`. The loop calls `mode.step()` and sleeps for the remainder of the tick. Ctrl+C triggers `mode.stop()` via a `try/finally` block.

10. **`src/pilotnano/hardware/realsense.py`** — `RealSenseCamera` wraps the `pyrealsense2` pipeline. `start()` configures and enables RGB + depth streams at the resolution/FPS from config, optionally creates an `rs.align` object to align depth frames to the RGB frame. `read()` calls `pipeline.wait_for_frames()`, extracts numpy arrays, and returns a `CameraFrame`. `stop()` cleanly shuts down the pipeline. Handles USB 3.0 vs 2.0 gracefully (logs a warning if bandwidth is limited).

11. **`src/pilotnano/hardware/pca9685.py`** — `PCA9685Actuator` wraps Adafruit ServoKit. `start()` initializes `ServoKit(channels=16, address=cfg.i2c_address)` and configures pulse width ranges from config (important for 2S ESC calibration). `send()` maps normalized steering [-1, 1] to servo angle and throttle [-1, 1] to ESC pulse width, applying the steering trim offset from config. `stop()` sends a neutral command (steering center, throttle zero) as a safety measure.

12. **`src/pilotnano/utils/logging.py`** — Sets up Python `logging` with a consistent format showing timestamp, level, and module name. Reads `log_level` from config.

**Milestone**: On the Jetson with everything wired up, `pilotnano --mode manual` shows live camera feed. Running `scripts/calibrate.py` (placeholder — will be fleshed out in Phase 3) lets you verify the servo sweeps left-right and the ESC responds.

---

### Phase 3: Data Collection

**Goal**: Enable the core training data workflow — drive the car manually with a gamepad while the system records synchronized camera frames and steering/throttle labels to disk. After this phase, you can do laps around a track and build up a dataset.

**Why now**: You can't train a model without data. Data collection is the most important capability for behavioral cloning. We build it early so you can start accumulating driving data while we work on the training pipeline.

**What we build**:

13. **`src/pilotnano/data/recorder.py`** — `Recorder` writes synchronized data to disk. Each recording session creates a timestamped directory (e.g., `data/runs/2025-01-15_14-30-00/`) containing a `frames/` subdirectory with numbered `000000_rgb.jpg` (and optionally `000000_depth.npz`) files, a `labels.csv` with columns `frame_id,timestamp,steering,throttle`, and a `metadata.json` with the config snapshot, start time, and frame count. JPG format keeps file sizes manageable (~30KB per frame at 640x480). CSV labels are human-readable and trivially loadable with numpy or pandas.

14. **`src/pilotnano/modes/collect.py`** — `CollectMode` is the data collection driving mode. Each `step()`: reads the gamepad (via pygame joystick API), maps stick axes to a `ControlCommand` (left stick horizontal → steering, right trigger → throttle), sends the command to the actuator (so you're physically driving the car through the PCA9685), reads a camera frame, and passes both to the `Recorder`. The gamepad simultaneously controls the car and provides the training labels — this is the key insight that makes behavioral cloning simple. A button on the gamepad toggles recording on/off so you can skip bad segments.

15. **`scripts/calibrate.py`** — Interactive CLI tool for servo and ESC calibration. Walks you through: setting steering left/center/right positions (maps physical limits to PWM values), arming the ESC (2S-specific procedure: neutral → full throttle → neutral), and setting throttle range. Writes the calibrated values to a YAML file that can be referenced from the main config. This is essential for the 2S setup since PWM ranges vary between ESCs.

**Milestone**: Drive the XV-02 around a track with a gamepad, see `data/runs/<timestamp>/` fill up with images and a labels.csv. Inspect the data with `ls` and a CSV viewer to confirm steering/throttle values look correct.

---

### Phase 4: Model + Training Pipeline

**Goal**: Build the neural network and the training pipeline so you can turn collected driving data into an autonomous driving model. After this phase, you can train a PilotNet model on your workstation and export it as an ONNX file ready for the Jetson.

**Why now**: By this point you should have collected several recording sessions. The model and training pipeline transform that raw data into an autopilot. We build the preprocessor first because it's shared between training and inference — ensuring the model sees exactly the same image transformations at both stages.

**What we build**:

16. **`src/pilotnano/perception/preprocessor.py`** — `Preprocessor` transforms raw 640x480 BGR camera frames into the tensor format the model expects. Pipeline: crop the top portion (removes sky/ceiling — irrelevant for an RC car on the ground), resize to 200x66 pixels (matching NVIDIA PilotNet input), convert BGR → RGB, normalize pixel values to [-1, 1], transpose HWC → CHW. All parameters (crop region, target size) come from config so they're easy to tune. This same class is used during both training and inference to guarantee consistency.

17. **`src/pilotnano/model/pilotnet.py`** — `PilotNet` is a PyTorch `nn.Module` implementing NVIDIA's DAVE-2 architecture, adapted for our RC car. Architecture: 5 convolutional layers (24→36→48→64→64 filters, mix of 5x5 and 3x3 kernels with strided convolutions for downsampling) followed by 4 fully connected layers (1152→100→50→10→2). Dropout (0.3) after the first two FC layers for regularization with small datasets. Output is 2 values: steering and throttle, passed through `tanh` to constrain to [-1, 1]. Total parameters: ~250K — tiny enough for real-time inference on Jetson Nano.

18. **`src/pilotnano/model/export.py`** — Utility function `export_to_onnx()` that takes a trained PilotNet checkpoint and exports it to ONNX format using `torch.onnx.export()` with opset 11. Validates the exported model with `onnx.checker.check_model()`. This ONNX file is what gets deployed to the Jetson.

19. **`src/pilotnano/data/dataset.py`** — `PilotNetDataset` is a PyTorch `Dataset` that loads recorded runs. It scans run directories for `labels.csv` files, builds a flat list of `(image_path, steering, throttle)` tuples, and in `__getitem__` loads the image, applies augmentations (if training), applies the `Preprocessor` transform, and returns `(tensor, [steering, throttle])`. Supports optional filtering to reduce the over-representation of straight-line driving (a common problem in behavioral cloning datasets).

20. **`src/pilotnano/data/augmentation.py`** — `DrivingAugmentation` applies training-time data augmentations to increase dataset diversity and reduce overfitting. Augmentations: random horizontal flip (mirrors the image and negates the steering value), random brightness/contrast adjustment (simulates lighting changes), random shadow overlay (simulates partial shadows on the track), and small random horizontal translation (with proportional steering adjustment to teach recovery). Each augmentation has a configurable probability.

21. **`scripts/train.py`** — Offline training script that runs on a workstation with a GPU (not on the Jetson). Usage: `python scripts/train.py --data ./data/runs`. Loads config, discovers all runs, builds the dataset with train/val split (85/15), creates DataLoaders, instantiates PilotNet, and runs a training loop with MSE loss on (steering, throttle), Adam optimizer, and cosine LR schedule. Logs train/val loss per epoch, saves the best checkpoint by validation loss, and exports the best model to ONNX at the end.

**Milestone**: After collecting 10-20 minutes of driving data across several sessions, run `python scripts/train.py --data ./data/runs` on your workstation. Training converges in ~30 epochs, producing `best.onnx` (~1MB). Validation loss shows the model learned the steering/throttle mapping.

---

### Phase 5: Inference + Autopilot Mode

**Goal**: Deploy the trained model to the Jetson and make the car drive itself. After this phase, you have a fully autonomous RC car running behavioral cloning.

**Why now**: We have all the pieces — the model, the preprocessing pipeline, the hardware drivers. This phase connects them into the autonomous driving loop: camera → preprocess → neural network → actuator.

**What we build**:

22. **`src/pilotnano/inference/engine.py`** — `InferenceEngine` wraps an ONNX Runtime `InferenceSession`. On init, it auto-detects available execution providers and prefers TensorRT (fastest on Jetson) > CUDA > CPU. Loads the ONNX model, reads input/output tensor names from metadata. `predict()` takes a preprocessed numpy array, runs inference, and returns `(steering, throttle)`. On the Jetson Nano with TensorRT, inference takes ~5-10ms per frame — well within our 50ms (20Hz) budget.

23. **`src/pilotnano/modes/autopilot.py`** — `AutopilotMode` is the autonomous driving mode. `start()` loads the `InferenceEngine` with the specified ONNX model path and initializes the `Preprocessor`. Each `step()`: reads a camera frame, preprocesses the RGB image into a model input tensor, runs inference to get steering and throttle predictions, creates a `ControlCommand`, sends it to the actuator, and publishes the command to the bus (for logging/display). The car drives itself in a closed loop at 20Hz.

24. **`scripts/export_onnx.py`** — Standalone script for converting a PyTorch checkpoint to ONNX. Usage: `python scripts/export_onnx.py --checkpoint best.pth --output model.onnx`. This is a convenience wrapper around the `export_to_onnx()` utility for when you want to re-export without re-training.

**Milestone**: Copy `best.onnx` to the Jetson, run `pilotnano --mode autopilot --model best.onnx`. The car drives autonomously around the track it was trained on. Expect some wobble and corrections — this is normal for behavioral cloning and can be improved with more data and augmentation.

---

### Phase 6: Tests + Developer Tooling

**Goal**: Add automated tests and quality-of-life tools for iterating on the project. After this phase, the project is robust, testable, and pleasant to work with.

**Why last**: Tests and tooling are important but they don't block any functionality. By building them after the core pipeline works end-to-end, we can write tests that reflect the actual architecture rather than guessing.

**What we build**:

25. **`tests/`** — `conftest.py` provides shared fixtures: `mock_camera`, `mock_actuator`, `local_bus`, and `sample_config` (loaded from `configs/hardware/mock.yaml`). Individual test files verify: `test_bus.py` — LocalBus subscribe/publish/unsubscribe, multiple subscribers, topic isolation. `test_hardware_mock.py` — MockCamera returns valid CameraFrame with correct shape, MockActuator records commands. `test_recorder.py` — records a few frames, verifies directory structure, CSV content, and image files exist. `test_model.py` — instantiates PilotNet, forward pass with random input, output shape is (B, 2), values in [-1, 1]. `test_inference.py` — exports PilotNet to ONNX in /tmp, loads with InferenceEngine, runs predict, verifies output shape.

26. **`scripts/visualize_data.py`** — Data browsing tool. Opens a collected run and displays frames with steering/throttle values overlaid as text and a visual indicator (steering as a rotated line, throttle as a bar). Arrow keys navigate between frames. Plots a histogram of steering/throttle distributions to diagnose dataset imbalance (too much straight driving). Optionally lets you mark and delete bad frames.

27. **`notebooks/explore_data.ipynb`** — Jupyter notebook for interactive data exploration. Load a run, display sample frames, plot steering/throttle over time, visualize augmentations, and inspect model predictions vs ground truth. Useful for debugging training issues.

28. **`Makefile`** — Developer shortcuts: `make test` (pytest), `make lint` (ruff check), `make format` (ruff format), `make train` (runs train.py with default args), `make export` (runs export_onnx.py), `make collect` (runs pilotnano in collect mode), `make drive` (runs pilotnano in autopilot mode).

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
| Training | `torch`, `torchvision`, `onnx` | Workstation only (optional dep) |
| Gamepad input | `pygame` | Jetson (for collect mode) |

## Verification

1. **Desktop (no hardware)**: `pilotnano --mode manual --hardware mock` — shows synthetic camera frames
2. **Jetson (hardware)**: `pilotnano --mode manual` — shows live RealSense feed, servo responds via calibrate script
3. **Data collection**: `pilotnano --mode collect` — drive with gamepad, verify `data/runs/<timestamp>/` has images + labels.csv
4. **Training**: `python scripts/train.py --data ./data/runs` — trains PilotNet, produces best.onnx
5. **Autopilot**: `pilotnano --mode autopilot --model best.onnx` — car drives autonomously
6. **Tests**: `pytest tests/` — all pass with mock hardware

---

## What's Next: Projects After the Scaffold

Once the base project is working (you can collect data, train, and drive autonomously), the architecture supports many extensions. Here are concrete projects roughly ordered by complexity:

### 1. Behavioral Cloning Refinement
- **What**: Improve the basic autopilot by collecting more diverse data (different speeds, lighting, track layouts), tuning augmentations, and experimenting with model architectures (add batch normalization, try MobileNet backbone, etc.)
- **Leverage**: Existing collect → train → drive pipeline, just iterate on data quality and model design
- **Depth data used**: No (RGB only)

### 2. Lane Following with Computer Vision
- **What**: Use classical CV (Canny edge detection, Hough line transform, or color thresholding) to detect lane lines or track edges, then use a PID controller to steer along them. No neural network needed.
- **Leverage**: Perception module — add a `lane_detector.py` alongside the preprocessor. Control module — add a `pid_controller.py`. Wire them in a new `LaneFollowMode`.
- **Depth data used**: No (RGB only)

### 3. Obstacle Avoidance with Depth
- **What**: Use the RealSense D435's depth stream to detect obstacles in the car's path. Set a depth threshold (e.g., anything closer than 1 meter), find the closest obstacle's position (left/center/right), and steer away from it.
- **Leverage**: The `CameraFrame` already includes depth. Add an `obstacle_detector.py` in perception that processes the depth map. Can be combined with behavioral cloning (override steering when an obstacle is close) or as a standalone safety layer.
- **Depth data used**: Yes — this is the first project that uses the D435's depth capability

### 4. Object Following (Follow-the-Leader)
- **What**: Detect and track a specific object (a person, another car, a colored ball) and drive to follow it at a set distance. Uses object detection for bearing and the depth map for range.
- **Leverage**: Add an object detector (YOLO-Nano or MobileNet-SSD via TensorRT) in perception. Use depth at the detected bounding box center to estimate distance. A PID controller maintains target distance and centers the object in frame.
- **Depth data used**: Yes — depth provides range to the target, enabling distance-keeping

### 5. Waypoint Navigation with Visual Odometry
- **What**: Drive to a sequence of waypoints using the RealSense for visual odometry (estimating position from frame-to-frame motion). No GPS needed — suitable for indoor or small outdoor areas.
- **Leverage**: Add a `visual_odometry.py` module using OpenCV feature matching or the RealSense's built-in tracking (D435i has IMU). Planning module gets a waypoint list and generates trajectories.
- **Depth data used**: Yes — stereo depth improves visual odometry accuracy

### 6. SLAM + Autonomous Exploration
- **What**: Build a map of the environment while driving (Simultaneous Localization and Mapping), then autonomously explore unmapped areas. Uses depth + RGB for mapping.
- **Leverage**: Integrate with ORB-SLAM3 or RTAB-Map (both support RealSense). The bus architecture makes it easy to add a SLAM node that subscribes to camera frames and publishes a map/pose. Planning module uses the map to choose frontiers to explore.
- **Depth data used**: Yes — essential for 3D mapping

### 7. Multi-Modal Driving (RGB + Depth Fusion)
- **What**: Train a neural network that takes both RGB and depth as input for more robust autonomous driving. The depth channel helps the model understand 3D scene geometry, especially in challenging lighting.
- **Leverage**: Modify PilotNet to accept 4-channel input (RGB + depth) or use a dual-encoder architecture. The recorder already optionally saves depth frames. Update the dataset loader to include depth.
- **Depth data used**: Yes — depth as an additional input channel to the neural network

### 8. ROS2 Migration
- **What**: Swap the `LocalBus` for a `ROS2Bus` adapter, turning each component into a ROS2 node. This enables distributed processing, RViz visualization, rosbag recording, and integration with the broader ROS ecosystem.
- **Leverage**: The bus abstraction was designed exactly for this. Write a `ROS2Bus` class that maps topic strings to ROS2 topics, wraps `rclpy.Publisher` and `rclpy.Subscription`. No business logic changes needed.
- **Depth data used**: N/A — infrastructure change, not a feature
