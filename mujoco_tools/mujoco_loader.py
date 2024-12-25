"""
MuJoCo Model Loader and Viewer

This script provides functionality to load and visualize MuJoCo physics simulation models.
It supports both active and passive viewing modes:
- Active viewer: Runs independently in a separate thread
- Passive viewer: Requires manual synchronization

Usage:
    python -m mujoco_tools.mujoco_loader --model path/to/model.xml [--active_viewer|--passive_viewer]

Arguments:
    --model: Path to the MuJoCo XML model file
    --passive_viewer: Enable passive viewer mode
    --active_viewer: Enable active viewer mode
"""

import mujoco
import mujoco.viewer
import numpy as np
import argparse
from .tools import load_model_from_path

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Load a MuJoCo model file')
    parser.add_argument('--model', type=str, default="../models/humanoid/humanoid.xml",
                       help='Path to the MuJoCo XML model file')
    parser.add_argument('--passive_viewer', action='store_true',
                       help='Whether to use passive viewer')
    parser.add_argument('--active_viewer', action='store_true',
                       help='Whether to use active viewer')
    
    args = parser.parse_args()
    model, data = load_model_from_path(args.model)
    mujoco.mj_step(model, data)
    if args.passive_viewer:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.sync()
    if args.active_viewer:
        viewer = mujoco.viewer.launch(model, data)
    breakpoint()
