#!/usr/bin/env python3
"""
Command-line interface for MuJoCo tools
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, Any
import mujoco
import numpy as np
from tqdm import tqdm
from mujoco_tools.recorder import VideoRecorder, StateRecorder
from mujoco_tools.player import MujocoPlayer
def parse_data_arg(data_str: str) -> Dict[str, str]:
    """Parse data argument string into a dictionary
    Example: "qpos data/qpos.npy ctrl data/ctrl.npy" -> {"qpos": "data/qpos.npy", "ctrl": "data/ctrl.npy"}
    """
    if not data_str:
        return {}
        
    parts = data_str.split()
    if len(parts) % 2 != 0:
        raise ValueError("Data argument must be pairs of type and path")
        
    return dict(zip(parts[::2], parts[1::2]))

def parse_vision_flags(flags_str: str) -> Dict[str, bool]:
    """Parse vision flags string into a dictionary
    Example: "mjVIS_ACTUATOR mjVIS_ACTIVATION" -> {"mjVIS_ACTUATOR": True, "mjVIS_ACTIVATION": True}
    """
    if not flags_str:
        return {}
        
    flags = {}
    for flag in flags_str.split():
        if not hasattr(mujoco.mjtVisFlag, flag):
            print(f"Warning: Unknown vision flag '{flag}'")
            continue
        flags[flag] = True
    return flags

class TestPlayer(MujocoPlayer):
    def play_trajectory(self, data, input_data_freq):
        """Play trajectory and notify all recorders"""
        # If no data provided, initialize with zeros for ctrl
        if not data:
            data = {'ctrl': np.zeros((1000, self.model.nu))}  # Default 100 timesteps
        else:
            for key, value in data.items():
                data[key] = np.load(value)
        # Calculate total frames using the first key in data dictionary
        first_key = next(iter(data))
        total_frames = len(range(0, len(data[first_key])))

        input_time_step = int(1 / (self.model.opt.timestep * input_data_freq))
        # Initialize data
        self.data.qpos = self.model.key_qpos[0]
        # Main playback loop with progress bar
        with tqdm(total=total_frames, desc="Playing trajectory", unit="frame") as pbar:
            for i in range(0, len(data[first_key])):
                for key, value in data.items():
                    # Safely set attributes instead of using eval
                    setattr(self.data, key, value[i])
                # Forward the simulation
                if self.mode == 'kinematics':
                    mujoco.mj_fwdPosition(self.model, self.data)
                elif self.mode == 'dynamics':
                    for _ in range(input_time_step):
                        mujoco.mj_step(self.model, self.data)
                # Notify all recorders
                for recorder in self.recorders:
                    output_time_step = int(input_data_freq / recorder.output_data_freq)
                    if i % output_time_step == 0:
                        recorder.record_frame(self.model, self.data)
                pbar.update(1)
                
def main():
    parser = argparse.ArgumentParser(description='MuJoCo visualization and recording tool')
    
    # Required arguments
    parser.add_argument('-m', '--model', type=str, required=True,
                      help='Path to MuJoCo XML model file')
    parser.add_argument('--mode', choices=['kinematics', 'dynamics'], default='kinematics',
                      help='Simulation mode (kinematics: runs mj.fwd_position, dynamics: runs mj.step)')
    parser.add_argument('--input_data_freq', type=int, default=50,
                      help='Frequency of input data')    
    parser.add_argument('--output_path', type=str, default='logs',
                      help='Output path')
    parser.add_argument('--output_prefix', type=str, default='output',
                      help='Output prefix')
    
    # Input data
    parser.add_argument('-d', '--data', type=str,
                      help='Input data type and path (e.g. "qpos data/qpos.npy ctrl data/ctrl.npy")')
        
    # Visualization options
    parser.add_argument('--record_video', action='store_true',
                      help='Enable video recording')    
    parser.add_argument('--width', type=int, default=1920,
                      help='Video width in pixels')
    parser.add_argument('--height', type=int, default=1080,
                      help='Video height in pixels')
    parser.add_argument('--fps', type=int, default=50,
                      help='Video framerate')
    parser.add_argument('--output_video_freq', type=int, default=50,
                      help='Frequency of output video')
    parser.add_argument('--camera', type=str, default='Free',
                      help='Camera name')
    parser.add_argument('--flags', type=str,
                      help='Custom vision flags (e.g. "mjVIS_ACTUATOR mjVIS_ACTIVATION")')
    
    # Recording options
    parser.add_argument('--record_data', action='store_true',
                      help='Enable data recording')
    parser.add_argument('--format', choices=['npy', 'txt', 'csv'], default='npy',
                      help='Output format for recorded data')
    parser.add_argument('--datatype', type=str, default='qpos',
                      help='Data types to record (space-separated: qpos qvel xpos xquat sensor tendon)')
    parser.add_argument('--output_data_freq', type=int, default=50,
                      help='Frequency of output data')
    
    args = parser.parse_args()
    
    # Initialize player
    player = TestPlayer(
        model_path=args.model,
        mode=args.mode,
        input_data_freq=args.input_data_freq,
        output_path=args.output_path,
        output_prefix=args.output_prefix
    )
    
    # Setup VideoRecorder if needed
    if args.record_video:
        video_recorder = VideoRecorder(
            camera_name=args.camera,
            width=args.width,
            height=args.height,
            fps=args.fps,
            vision_flags=parse_vision_flags(args.flags),
            output_video_freq=args.output_video_freq
        )
        player.add_recorder(video_recorder)
    
    # Setup recorder if needed
    if args.record_data:
        datatypes = set(args.datatype.split())
        recorder = StateRecorder(
            model=player.model,
            output_format=args.format,
            datatypes=datatypes,
            output_data_freq=args.output_data_freq
        )
        player.add_recorder(recorder)
    
    # Load data and play trajectory
    data_files = parse_data_arg(args.data)
    player.play_trajectory(
        data_files,
        input_data_freq=args.input_data_freq
    )
    player.save_data()

if __name__ == '__main__':
    sys.exit(main()) 