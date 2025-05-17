import pytest
import os
import tempfile
import numpy as np
from unittest.mock import patch, MagicMock
from mujoco_tools.player import MujocoPlayer
from mujoco_tools.recorder import VideoRecorder, StateRecorder
from mujoco_tools.data_processor import InputDataProcessor

class TestIntegration:
    @patch.object(InputDataProcessor, 'process')
    def test_basic_workflow(self, mock_process, fixture_model_path, sample_data, tmp_path):
        """Test the complete workflow with MujocoPlayer and recorders"""
        # Set up the player with kinematics mode
        output_path = str(tmp_path / "output")
        os.makedirs(output_path, exist_ok=True)
        
        # Prepare input data string
        input_str = f"qpos {sample_data['qpos_path']} ctrl {sample_data['ctrl_path']}"
        
        # Mock the data processor to return sample data
        mock_process.return_value = {
            'qpos': sample_data['qpos'],
            'ctrl': sample_data['ctrl']
        }
        
        # Create player
        player = MujocoPlayer(
            model_path=fixture_model_path,
            mode='kinematics',
            input_data_freq=100,
            output_path=output_path,
            output_prefix="test_run",
            input_data=input_str
        )
        
        # Create and add recorders
        # For testing, we can use minimal settings
        video_recorder = VideoRecorder(
            camera_name='side',  # Use a camera that exists in the model
            width=320,  # Smaller resolution for faster tests
            height=240,
            fps=30,
            output_video_freq=20,
            vision_flags={'mjVIS_TEXTURE': 1}
        )
        player.add_recorder(video_recorder)
        
        # Create a mock state recorder
        mock_state_recorder = MagicMock(spec=StateRecorder)
        mock_state_recorder.output_data_freq = 50
        player.add_recorder(mock_state_recorder)
        
        # Run the simulation (wrapped in try-except for proper cleanup)
        try:
            player.play_trajectory()
            player.save_data()
            
            # Verify outputs and that methods were called
            assert os.path.exists(os.path.join(output_path, "test_run_video.mp4"))
            mock_state_recorder.initialize.assert_called_once()
            mock_state_recorder.save.assert_called_once_with(output_path, "test_run")
        except Exception as e:
            # Cleanup in case of failure
            player.save_data()  # Ensure files are properly closed
            raise e 