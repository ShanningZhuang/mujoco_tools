import mujoco
import numpy as np
import os
from tqdm import tqdm
from .data_processor import InputDataProcessor
class MujocoPlayer:
    def __init__(self, model_path, mode='kinematics', input_data_freq=500, output_path=None, output_prefix=None, input_data=None):
        """Initialize MuJoCo player with model and optional recorders"""
        if mode not in ['kinematics', 'dynamics']:
            raise ValueError("Mode must be either 'kinematics' or 'dynamics'")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.mode = mode
        self.input_data_freq = input_data_freq
        self.output_path = output_path
        if output_path:
            os.makedirs(output_path, exist_ok=True)
        self.output_prefix = output_prefix
        self.recorders = []
        data_processor = InputDataProcessor(input_data)
        self.input_data = data_processor.process()
        
    def add_recorder(self, recorder):
        """Add a recorder to the player"""
        if self.input_data_freq % recorder.output_data_freq != 0:
            raise ValueError("Input data frequency must be divisible by recorder output data frequency")
        recorder.initialize(self.output_path, self.output_prefix)
        self.recorders.append(recorder)
        
    def play_trajectory(self):
        """Play trajectory and notify all recorders"""
        # If no data provided, initialize with zeros for ctrl
        if not self.input_data:
            data = {'ctrl': np.zeros((1000, self.model.nu))}  # Default 1000 timesteps
        else:
            data = self.input_data
        # Calculate total frames using the first key in data dictionary
        first_key = next(iter(data))
        total_frames = len(range(0, len(data[first_key])))

        input_time_step = int(1 / (self.model.opt.timestep * self.input_data_freq))
        # Main playback loop with progress bar
        with tqdm(total=total_frames, desc="Playing trajectory", unit="frame") as pbar:
            for i in range(0, len(data[first_key])):
                for key, value in data.items():
                    # Safely set attributes instead of using eval
                    key = key.split('.')[-1]
                    setattr(self.data, key, value[i])
                # Forward the simulation
                if self.mode == 'kinematics':
                    mujoco.mj_fwdPosition(self.model, self.data)
                elif self.mode == 'dynamics':
                    for _ in range(input_time_step):
                        mujoco.mj_step(self.model, self.data)
                # Notify all recorders
                for recorder in self.recorders:
                    output_time_step = int(self.input_data_freq / recorder.output_data_freq)
                    if i % output_time_step == 0:
                        recorder.record_frame(self.model, self.data)
                pbar.update(1)
                
    def save_data(self):
        """Save data from all recorders"""
        # Add timestamp to output prefix
        for recorder in self.recorders:
            recorder.save(self.output_path,self.output_prefix)

    def play_one_step(self, step_index=None, data_dict=None):
        """Play a single step of the trajectory and return model and data
        
        Args:
            step_index: Index of the step to play. If None, uses the next available step.
            data_dict: Optional dictionary with data to use instead of self.input_data
        
        Returns:
            tuple: (model, data) after playing the step
        """
        # Use provided data or fall back to input_data
        if data_dict is None:
            if not self.input_data:
                # Create default data if none provided
                data_dict = {'ctrl': np.zeros((1, self.model.nu))}
            else:
                data_dict = self.input_data
        
        # Determine which step to play
        if step_index is None:
            # Use an internal counter if no step index provided
            if not hasattr(self, '_current_step'):
                self._current_step = 0
            step_index = self._current_step
            self._current_step += 1
        
        # Check if the step is within bounds
        first_key = next(iter(data_dict))
        if step_index >= len(data_dict[first_key]):
            raise IndexError(f"Step index {step_index} out of bounds (max: {len(data_dict[first_key])-1})")
        
        # Set data values from dictionary
        for key, value in data_dict.items():
            # Handle both direct attribute names and nested attribute paths
            attr_parts = key.split('.')
            if len(attr_parts) > 1:
                # For attributes like 'self.data.qpos'
                setattr(self.data, attr_parts[-1], value[step_index])
            else:
                # For direct attributes
                setattr(self.data, key, value[step_index])
        
        # Forward the simulation
        if self.mode == 'kinematics':
            mujoco.mj_fwdPosition(self.model, self.data)
        elif self.mode == 'dynamics':
            input_time_step = int(1 / (self.model.opt.timestep * self.input_data_freq))
            for _ in range(input_time_step):
                mujoco.mj_step(self.model, self.data)
        
        # Return the model and data
        return self.model, self.data