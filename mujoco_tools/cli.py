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

from .player import MujocoPlayer
from .recorder import VideoRecorder, StateRecorder

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
    parser.add_argument('--init_qpos', type=int,
                      help='Initialize first frame with key_qpos[init_qpos] from the model')
        
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
    parser.add_argument('--activation_map', action='store_true',
                      help='Enable activation map')
    parser.add_argument('--activation_shape', type=str, default='35 20',
                      help='Activation map shape (rows cols)')
    
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
    player = MujocoPlayer(
        model_path=args.model,
        mode=args.mode,
        input_data_freq=args.input_data_freq,
        output_path=args.output_path,
        output_prefix=args.output_prefix,
        input_data=args.data,
        init_qpos=args.init_qpos
    )
    
    # Setup VideoRecorder if needed
    if args.record_video:
        video_recorder = VideoRecorder(
            camera_name=args.camera,
            width=args.width,
            height=args.height,
            fps=args.fps,
            vision_flags=parse_vision_flags(args.flags),
            activation_map=args.activation_map,
            activation_shape=tuple(map(int, args.activation_shape.split())),
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
    player.play_trajectory()
    player.save_data()

if __name__ == '__main__':
    sys.exit(main()) 