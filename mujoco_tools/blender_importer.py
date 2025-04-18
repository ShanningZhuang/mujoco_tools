#!/usr/bin/env python3
"""
Command-line interface for importing MuJoCo models and animation data into Blender
"""

import os
import numpy as np
import mujoco
from pathlib import Path

from mujoco_tools.player import MujocoPlayer
from mujoco_tools.recorder import BlenderRecorder

def main():
    # Set default values directly
    model_path = '/home/zsn/research/blender_project/blender_mujoco_chengtian/assets/ant.xml'
    input_data = 'qpos /home/zsn/research/blender_project/blender_mujoco_chengtian/ant_qpos.txt'
    mode = 'kinematics'
    input_data_freq = 50
    output_path = None
    output_prefix = None
    
    # Initialize player
    player = MujocoPlayer(
        model_path=model_path,
        mode=mode,
        input_data_freq=input_data_freq,
        output_path=output_path,
        output_prefix=output_prefix,
        input_data=input_data
    )
    
    # Initialize the blender recorder
    blender_recorder = BlenderRecorder(
        output_data_freq=input_data_freq,
        record_types=["geom"]
    )
    
    # Add recorder to player and run animation
    player.add_recorder(blender_recorder)
    player.play_trajectory()

    print(f"Blender import completed. Objects created from model: {model_path}")

main()