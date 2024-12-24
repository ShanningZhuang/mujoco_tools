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
