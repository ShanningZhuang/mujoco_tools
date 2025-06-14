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
    def __init__(self, camera_name='lateral_camera_angle', width=1920, height=1080, fps=50, vision_flags=None, 
                 output_video_freq=50, activation_map=False, activation_shape=(35, 20)):
        """Initialize video recorder with rendering settings
        
        Args:
            camera_name: Camera name or space-separated camera names
            width: Video width
            height: Video height
            fps: Frames per second
            vision_flags: MuJoCo vision flags
            output_video_freq: Output video frequency
            activation_map: Whether to include activation visualization in the recording
            activation_shape: Shape to use for activation visualization (rows, cols)
        """
        # Convert camera_name string to list if space-separated
        self.camera_names = camera_name.split() if isinstance(camera_name, str) else camera_name
        self.fps = fps
        self.output_data_freq = output_video_freq
        # Width per camera will be total width divided by number of cameras
        self.camera_width = width
        self.camera_height = height
        self.vision_flags = vision_flags
        self.setup_renderer(self.camera_width, self.camera_height)
        # Initialize video writer as None
        self.video_writer = None
        self.output_path = None
        # Activation map settings
        self.activation_map = activation_map
        self.activation_shape = activation_shape
        
    def setup_renderer(self, camera_width, camera_height):
        """Set up the MuJoCo renderer with given settings"""
        self.video_height = camera_height 
        self.video_width = camera_width * len(self.camera_names)
        self.rgb_renderer = None
        self.scene_option = mujoco.MjvOption()
        for flag, value in self.vision_flags.items():
            self.scene_option.flags[getattr(mujoco.mjtVisFlag, flag)] = value
        
    def initialize(self, output_path, output_prefix):
        """Initialize video writer"""
        self.output_path = f'{output_path}/{output_prefix}_video.mp4'
        frame_size = (self.video_width, self.video_height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, frame_size)
        print(f"Started recording to {self.output_path}")
        
    def record_frame(self, model, data):
        """Record frames from multiple cameras and concatenate them
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        frames = []
        if self.rgb_renderer is None:
            self.rgb_renderer = mujoco.Renderer(model, width=self.camera_width, height=self.camera_height)
            
        for camera_name in self.camera_names:
            # Create scene and camera
            self.rgb_renderer.update_scene(data, camera=camera_name, scene_option=self.scene_option)
            frame = self.rgb_renderer.render()
            frames.append(frame)
        
        # Concatenate frames horizontally
        combined_frame = np.concatenate(frames, axis=1)
        
        # Add activation map if enabled
        if self.activation_map:
            combined_frame = self._add_activation_map(combined_frame, data.act)
        
        # Convert from RGB to BGR for OpenCV
        combined_frame = cv2.cvtColor(combined_frame, cv2.COLOR_RGB2BGR)
        self.video_writer.write(combined_frame)
        
    def _add_activation_map(self, sim_frame, activation):
        """Add activation map visualization to the simulation frame
        
        Args:
            sim_frame: Simulation frame (numpy array)
            activation: Activation array to visualize
            
        Returns:
            numpy.ndarray: Combined frame with activation visualization
        """
        # Make a copy of the activation array
        activation = activation.copy()
        
        # Calculate the total elements in the target shape
        target_elements = self.activation_shape[0] * self.activation_shape[1]
        
        # If the activation array is smaller than the target shape, pad with zeros
        if activation.size < target_elements:
            # Create a new array with the right number of elements, filled with zeros
            padded_activation = np.zeros(target_elements)
            
            # Copy the actual activation values to the beginning of the padded array
            padded_activation[:activation.size] = activation.flatten()
            
            # Use the padded activation for visualization
            activation = padded_activation
        
        # Generate activation visualization
        activation_frame = self.visualize_activation(activation, shape=self.activation_shape)
        
        # Resize activation frame to match the height of the original frame
        # while preserving aspect ratio
        act_height = sim_frame.shape[0]
        act_aspect_ratio = activation_frame.shape[1] / activation_frame.shape[0]
        act_width = int(act_height * act_aspect_ratio)
        # Force act_width to be even
        act_width = act_width if act_width % 2 == 0 else act_width + 1
        activation_frame_resized = cv2.resize(activation_frame, (act_width, act_height))
        
        # Calculate how much space we have for the simulation frame
        total_width = self.video_width
        sim_width = total_width - act_width
        
        # If the simulation frame is wider than the available space, crop it from the center
        if sim_frame.shape[1] > sim_width:
            # Crop from the center
            center_x = sim_frame.shape[1] // 2
            half_width = sim_width // 2
            left_bound = max(0, center_x - half_width)
            right_bound = min(sim_frame.shape[1], center_x + half_width)
            # Extract the center region
            sim_frame = sim_frame[:, left_bound:right_bound]
        
        # Concatenate with the activation visualization
        combined_frame = np.concatenate([sim_frame, activation_frame_resized], axis=1)
        
        return combined_frame
        
    def save(self, output_path, output_prefix='video'):
        """Finish recording and release video writer"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            print(f"Video saved successfully to {self.output_path}")
        
    def visualize_activation(self, activation, shape=(35, 20), title='Muscle Activation', figsize=(20, 35), dpi=30.86):
        """Convert muscle activation array into a visualization image
        
        Args:
            activation: Numpy array of activation values (will be reshaped to shape)
            shape: Tuple (rows, cols) to reshape the activation array
            title: Title for the visualization
            figsize: Figure size in inches
            dpi: Dots per inch for the figure
            
        Returns:
            numpy.ndarray: RGB image of the visualization
        """
        import matplotlib.pyplot as plt
        
        plt.figure(layout='constrained', facecolor='white', figsize=figsize, dpi=dpi)
        
        # Define the color gradient: light pink to muscle red
        actuator_color = np.array([255, 227, 224]) / 255.0  # Light pink
        muscle_color = np.array([239, 99, 81]) / 255.0      # Muscle red
        cmap = plt.cm.colors.LinearSegmentedColormap.from_list("custom_cmap", [actuator_color, muscle_color])
        
        # Set up the plot
        plt.imshow(activation.reshape(shape), cmap=cmap, vmax=1, vmin=0)
        
        # Remove ticks and labels
        plt.gca().tick_params(axis='both', which='both', bottom=False, top=False, 
                             labelbottom=False, right=False, left=False, labelleft=False)
        
        # Add grid
        plt.grid(True, which='both', axis='both', color='black', linestyle='-', linewidth=4)
        
        # Set grid lines to be in between cells
        plt.gca().set_xticks(np.arange(-0.5, shape[1] + 0.5, 1) - 0.005)
        plt.gca().set_yticks(np.arange(-0.5, shape[0] + 0.5, 1) - 0.01)
        
        plt.gca().set_axisbelow(False)
        plt.title(title, fontsize=96, fontweight='bold')
        
        # Render figure to numpy array
        fig = plt.gcf()
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        
        plt.close()
        return img

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
