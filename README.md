<div align="center" style="line-height: 1;">
<h1 style="text-align: center;">MuJoCo Tools: A Toolbox For MuJoCo</h1>
  <a href="https://mujoco-tool.readthedocs.io/en/latest/?badge=latest"><img alt="Docs" 
    src="https://readthedocs.org/projects/mujoco-tool/badge/?version=latest"/></a>
  <a href="https://www.lnsgroup.cc/"><img alt="Homepage"
    src="https://img.shields.io/badge/Homepage-LnsGroup-671372"/></a>
  <a href="https://github.com/ShanningZhuang/mujoco_tools/blob/main/docs/images/wechat_qr_code.jpeg?raw=true"><img alt="Wechat"
    src="https://img.shields.io/badge/WeChat-Shanning_Zhuang-brightgreen?logo=wechat&logoColor=white"/></a>
  <a href="https://twitter.com/ShanningZhuang"><img alt="Twitter Follow"
    src="https://img.shields.io/badge/Twitter-Shanning_Zhuang-white?logo=x&logoColor=white"/></a>
  <a href="https://github.com/ShanningZhuang/mujoco_tools/blob/main/LICENSE"><img alt="Code License"
    src="https://img.shields.io/badge/Code_License-MIT-f5de53?&color=f5de53"/></a>
<br>
<img src="https://github.com/ShanningZhuang/mujoco_tools/blob/main/docs/images/mujoco_tools_logo_jpg.jpg?raw=true" width="20%" alt="MuJoCo Tools" />
  <br>
</div>

[English](README.md) | [中文](README_CN.md)

A comprehensive toolkit for MuJoCo simulation, visualization, and data processing.

## Todo List

### Refactor

- [] Define the unified data format (txt npy npz)
- [] In recorder, the video that output the result of the activation is obviously not good.

### Completed Features
- [x] Basic MuJoCo model loading and simulation
- [x] Command-line interface with comprehensive options
- [x] 3D model motion trajectory rendering
- [x] Multi-camera view support
- [x] Customizable resolution (supports up to 4K)
- [x] Adjustable playback speed and frame rate
- [x] Joint positions (qpos) and velocities (qvel) recording
- [x] Body positions (xpos) and orientations (xquat) recording
- [x] Support for multiple data formats (.npy/.txt/.csv)
- [x] Basic data analysis and processing utilities

### In Progress
- [ ] Add tests scripts to test the package
- [ ] Muscle activation heatmap display
- [ ] Tendon path points recording
- [ ] Sensor data recording
- [ ] Support for .mot trajectory format
- [ ] Advanced data analysis tools
- [ ] Documentation improvements
- [ ] Unit tests
- [ ] Example scripts for common use cases

## Installation

```bash
# Install from source
git clone https://github.com/yourusername/mujoco_tools.git
cd mujoco_tools
pip install -e .
# Install from PyPI
pip install mujoco-tools
# Install from github
pip install git+https://github.com/ShanningZhuang/mujoco_tools.git
```

## Project Structure

```
mujoco_tools/
├── mujoco_tools/          # Main package
│   ├── cli.py            # Command-line interface
│   ├── mujoco_loader.py  # MuJoCo model loading utilities
│   ├── player.py         # Visualization player
│   ├── recorder.py       # Data recording utilities
│   ├── tools.py          # General utilities
│   └── data_processor.py # Data processing utilities
├── models/               # Test models
└── examples/             # Example scripts
```

## Usage

### 1. Command Line Interface

```bash
mujoco-tools -m <model.xml> [options]
```

#### Required Arguments:
- `-m, --model`: Path to MuJoCo XML model file
- `--mode`: Simulation mode (kinematics: runs mj.fwd_position, dynamics: runs mj.step) [default: kinematics]

#### Input Data Options:
- `-d, --data`: Input data type and path (e.g., "qpos data/qpos.npy ctrl data/ctrl.npy") or Directly input the path of npz
- `--input_data_freq`: Frequency of input data [default: 50]

#### Output Path Options:
- `--output_path`: Output path [default: logs]
- `--output_prefix`: Output prefix [default: output]

#### Visualization Options:
- `--record_video`: Enable video recording
- `--width`: Video width in pixels [default: 1920]
- `--height`: Video height in pixels [default: 1080]
- `--fps`: Video framerate [default: 50]
- `--output_video_freq`: Frequency of output video [default: 50]
- `--camera`: Camera name [default: Free]
- `--flags`: Custom vision flags (e.g., "mjVIS_ACTUATOR mjVIS_ACTIVATION")

#### Recording Options:
- `--record_data`: Enable data recording
- `--format`: Output format (npy/txt/csv) [default: npy]
- `--datatype`: Data types to record (space-separated: qpos qvel xpos xquat sensor tendon) [default: qpos]
- `--output_data_freq`: Frequency of output data [default: 50]

### 2. Bash Script Usage

Create a bash script for configuration:

```bash
#!/bin/bash

# Default settings
MODEL_PATH="models/humanoid/humanoid.xml"
DATA_PATH="qpos data/qpos.npy"
MODE="kinematics"
OUTPUT="output/video.mp4"
RESOLUTION="1080p"
FPS=50
CAMERA="side"
RECORD_DATA=1
DATA_FORMAT="npy"
RECORD_TYPES="qpos qvel xpos"

# Build command
CMD="mujoco-tools \\
    -m \"$MODEL_PATH\" \\
    -d \"$DATA_PATH\" \\
    --mode \"$MODE\" \\
    -o \"$OUTPUT\" \\
    --resolution \"$RESOLUTION\" \\
    --fps \"$FPS\" \\
    --camera \"$CAMERA\""

# Add recording options
if [ "$RECORD_DATA" -eq 1 ]; then
    CMD+=" --record"
    CMD+=" --format \"$DATA_FORMAT\""
    CMD+=" --datatype \"$RECORD_TYPES\""
fi

# Execute command
eval "$CMD"
```

### 3. Python Module Usage

```python
# Direct module import
from mujoco_tools import MujocoLoader, Player, Recorder

# Command line usage
python -m mujoco_tools.cli -m /path/to/model.xml -d 'qpos /path/to/data.npy'
python -m mujoco_tools.mujoco_loader -m /path/to/model.xml
mujoco-tools -m /home/zsn/research/mujoco_tools/logs/model/Arm_Hand/mj_vision_manipulation_high_cube.xml -d "act /home/zsn/research/mujoco_tools/logs/arm_hand/2025_02_08_23_07_44_HandCube_act.txt qpos /home/zsn/research/mujoco_tools/logs/arm_hand/2025_02_08_23_07_44_HandCube_qpos.txt" --mode kinematics --input_data_freq 500 --record_video --camera "record_camera_2" --width 1920 --height 1080 --output_prefix stage6 --flags "mjVIS_ACTUATOR mjVIS_ACTIVATION"  --activation_map
 --activation_shape "10 9"
```

## Data Format

The default output format is `.npy` (or `.npz` for multiple arrays). Data is stored in `(time, data)` format.

## Development

### Setup Development Environment
```bash
# Clone the repository
git clone https://github.com/yourusername/mujoco_tools.git
cd mujoco_tools

# Install development dependencies
pip install -e .
```

### Publishing
The package is automatically published to PyPI when a new release is created on GitHub. To publish a new version:

1. Update version in `setup.py`
2. Create and push a new tag:
```bash
git tag v0.1.0  # Use appropriate version
git push origin v0.1.0
```
3. Create a new release on GitHub using the tag
4. The GitHub Action will automatically build and publish to PyPI

### Running Tests
```bash
pytest tests/
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

To contribute:
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

[MIT License](LICENSE) 
