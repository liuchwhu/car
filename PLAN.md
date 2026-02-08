# PilotNano - Self-Driving RC Car Project Scaffold

## Context
Scaffold a Python project for a self-driving RC car built on a Jetson Nano + Tamiya XV-02 chassis with PCA9685 PWM driver and Intel RealSense D435 camera. The project should support behavioral cloning (collect data, train, drive autonomously) and be structured to grow into a modular perception-planning-control pipeline. A lightweight message bus abstracts communication so ROS2 can be swapped in later.

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

## Implementation Order

### Phase 1: Foundation (files 1-8)
1. `pyproject.toml` - project metadata, dependencies, entry point
2. `configs/` - all YAML config files
3. `src/pilotnano/config.py` - OmegaConf loader
4. `src/pilotnano/bus/` - messages, base ABC, LocalBus
5. `src/pilotnano/hardware/base.py` - Camera + Actuator ABCs
6. `src/pilotnano/hardware/mock.py` - mock implementations
7. `src/pilotnano/hardware/__init__.py` - factory functions
8. `src/pilotnano/modes/base.py` + `manual.py` - base mode + manual mode

### Phase 2: Main App + Hardware (files 9-13)
9. `src/pilotnano/app.py` - entry point with main loop
10. `src/pilotnano/hardware/realsense.py` - RealSense D435 driver
11. `src/pilotnano/hardware/pca9685.py` - PCA9685 servo/ESC driver
12. `src/pilotnano/utils/logging.py` - logging setup

### Phase 3: Data Collection (files 13-15)
13. `src/pilotnano/data/recorder.py` - frame + label recording
14. `src/pilotnano/modes/collect.py` - collection mode
15. `scripts/calibrate.py` - servo calibration tool

### Phase 4: Model + Training (files 16-21)
16. `src/pilotnano/perception/preprocessor.py` - image preprocessing
17. `src/pilotnano/model/pilotnet.py` - PilotNet CNN
18. `src/pilotnano/model/export.py` - ONNX export
19. `src/pilotnano/data/dataset.py` - PyTorch Dataset
20. `src/pilotnano/data/augmentation.py` - augmentations
21. `scripts/train.py` - training script

### Phase 5: Inference + Autopilot (files 22-24)
22. `src/pilotnano/inference/engine.py` - ONNX Runtime wrapper
23. `src/pilotnano/modes/autopilot.py` - autonomous driving mode
24. `scripts/export_onnx.py` - export script

### Phase 6: Tests + Extras (files 25-31)
25. `tests/conftest.py` + all test files
26. `scripts/visualize_data.py` - data browser
27. `notebooks/explore_data.ipynb`
28. `Makefile`

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

1. **Desktop (no hardware)**: `pilotnano --mode manual --config configs/hardware/mock.yaml` - should show synthetic camera frames
2. **Jetson (hardware)**: `pilotnano --mode manual` - should show live RealSense feed, verify servo responds to calibrate script
3. **Data collection**: `pilotnano --mode collect` - drive with gamepad, verify `data/runs/<timestamp>/` has images + labels.csv
4. **Training**: `python scripts/train.py --data ./data/runs` - trains PilotNet, exports best.onnx
5. **Autopilot**: `pilotnano --mode autopilot --model best.onnx` - car drives autonomously
6. **Tests**: `pytest tests/` - all pass with mock hardware

## Post-Implementation

- Initialize git repo, commit, push to `https://github.com/liuchwhu/car.git`
