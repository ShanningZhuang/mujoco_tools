import mujoco
import mujoco.viewer
import numpy as np
import os
import cv2
import matplotlib.pyplot as plt
from pathlib import Path
import imageio
from tqdm import tqdm

class VideoRecorder:
    def __init__(self, camera_name='lateral_camera_angle', width=1920, height=1080, fps=50, vision_flags=None, output_video_freq=50):
        """Initialize video recorder with rendering settings"""
        self.camera_name = camera_name
        self.fps = fps
        self.output_data_freq = output_video_freq
        self.setup_renderer(width, height)
        # Initialize video writer as None
        self.video_writer = None
        self.output_path = None
        
    def setup_renderer(self, width, height):
        """Set up the MuJoCo renderer with given settings"""
        self.height = height
        self.width = width
        self.rgb_renderer = None
        self.scene_option = mujoco.MjvOption()
        # setattr(self.scene_option, 'flags', [mujoco.mjtVisFlag.mjVIS_ACTUATOR, mujoco.mjtVisFlag.mjVIS_ACTIVATION])
        
    def initialize(self, output_path, output_prefix):
        """Initialize video writer"""
        self.output_path = f'{output_path}/{output_prefix}_video.mp4'
        frame_size = (self.width, self.height)
        fourcc = cv2.VideoWriter_fourcc(*'h264')
        self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, frame_size)
        print(f"Started recording to {self.output_path}")
        
    def record_frame(self, model, data):
        """Record a single frame directly to video file"""
        if self.rgb_renderer is None:
            self.rgb_renderer = mujoco.Renderer(model, width=self.width, height=self.height)
        # Create scene and camera
        self.rgb_renderer.update_scene(data, camera=self.camera_name, scene_option=self.scene_option)
        frame = self.rgb_renderer.render()
        # Convert from RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.video_writer.write(frame)
        
    def save(self, output_path, output_prefix='video'):
        """Finish recording and release video writer"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            print(f"Video saved successfully to {self.output_path}")
        
class StateRecorder:
    def __init__(self, model, output_format='npy', datatypes=None, output_data_freq=50):
        """Initialize state recorder for positions and orientations"""
        self.datatypes = datatypes
        self.output_format = output_format
        self.output_data_freq = output_data_freq
        self.reset()
    
    def initialize(self, output_path, output_prefix):
        """Initialize state recorder"""
        self.output_path = output_path
        self.output_prefix = output_prefix
        
    def reset(self):
        """Reset recorded data"""
        self.recorded_data = {datatype: [] for datatype in self.datatypes}
        
    def record_frame(self, model, data):
        """Record position and orientation data for the current frame"""
        for datatype in self.datatypes:
            self.recorded_data[datatype].append(getattr(data, datatype).copy())
            
    def tendon_waypoint(self, model, data):
        """
        Record tendon waypoint xpos
        ten_wrapadr is the start address of tendon's path
        ten_wrapnum is the number of wrap points in path
        wrap_xpos is the Cartesian 3D points in all paths
        return:
            waypoint_xpos: list of numpy arrays, (ntendon, num_waypoints, 6)
        """
        ten_wrapadr = data.ten_wrapadr
        ten_wrapnum = data.ten_wrapnum
        wrap_xpos = data.wrap_xpos.reshape(-1,3)
        waypoint_xpos = []
        for i in range(model.ntendon):
            start = ten_wrapadr[i]
            end = start + ten_wrapnum[i]
            waypoint_xpos.append(wrap_xpos[start:end,:])
        return waypoint_xpos
            
    def save(self, output_path, output_prefix='state'):
        """Save recorded state data"""
        for datatype in self.datatypes:
            np.save(f'{self.output_path}/{self.output_prefix}_{datatype}.npy', np.array(self.recorded_data[datatype]))
