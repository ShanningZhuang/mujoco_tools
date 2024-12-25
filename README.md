# MuJoCo Tools

[English](README.md) | [中文](README_CN.md)

A comprehensive toolkit for MuJoCo simulation, visualization, and data processing.

## Todo List

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
- `-d, --data`: Input data type and path (e.g., "qpos data/qpos.npy ctrl data/ctrl.npy")
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
```

## Data Format

The default output format is `.npy` (or `.npz` for multiple arrays). Data is stored in `(time, data)` format.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

[MIT License](LICENSE) 
