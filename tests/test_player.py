import pytest
import numpy as np
import os
import mujoco
from unittest.mock import MagicMock, patch
from mujoco_tools.player import MujocoPlayer
from mujoco_tools.data_processor import InputDataProcessor

class TestMujocoPlayer:
    @patch.object(InputDataProcessor, 'process', return_value={})
    def test_init_kinematics_mode(self, mock_process, fixture_model_path):
        """Test initializing player in kinematics mode"""
        player = MujocoPlayer(fixture_model_path, mode='kinematics')
        assert player.mode == 'kinematics'
        assert isinstance(player.model, mujoco.MjModel)
        assert isinstance(player.data, mujoco.MjData)
        assert player.recorders == []
    
    @patch.object(InputDataProcessor, 'process', return_value={})
    def test_init_dynamics_mode(self, mock_process, fixture_model_path):
        """Test initializing player in dynamics mode"""
        player = MujocoPlayer(fixture_model_path, mode='dynamics')
        assert player.mode == 'dynamics'
    
    def test_init_invalid_mode(self, fixture_model_path):
        """Test error on invalid mode"""
        with pytest.raises(ValueError) as excinfo:
            MujocoPlayer(fixture_model_path, mode='invalid')
        assert "Mode must be either 'kinematics' or 'dynamics'" in str(excinfo.value)
    
    @patch.object(InputDataProcessor, 'process', return_value={})
    def test_init_with_output_path(self, mock_process, fixture_model_path, tmp_path):
        """Test initialization with output path"""
        output_path = str(tmp_path / "output")
        player = MujocoPlayer(fixture_model_path, output_path=output_path)
        assert player.output_path == output_path
        assert os.path.exists(output_path)
    
    @patch.object(InputDataProcessor, 'process', return_value={})
    def test_add_recorder_valid(self, mock_process, fixture_model_path):
        """Test adding a valid recorder"""
        player = MujocoPlayer(fixture_model_path, input_data_freq=500)
        
        # Create a mock recorder
        mock_recorder = MagicMock()
        mock_recorder.output_data_freq = 50  # Divisible by input_data_freq
        
        player.add_recorder(mock_recorder)
        assert player.recorders == [mock_recorder]
        mock_recorder.initialize.assert_called_once()
    
    @patch.object(InputDataProcessor, 'process', return_value={})
    def test_add_recorder_invalid_freq(self, mock_process, fixture_model_path):
        """Test error when adding recorder with invalid frequency"""
        player = MujocoPlayer(fixture_model_path, input_data_freq=500)
        
        # Create a mock recorder with non-divisible frequency
        mock_recorder = MagicMock()
        mock_recorder.output_data_freq = 33  # Not divisible by input_data_freq
        
        with pytest.raises(ValueError) as excinfo:
            player.add_recorder(mock_recorder)
        assert "Input data frequency must be divisible by recorder output data frequency" in str(excinfo.value)
    
    @patch('mujoco.mj_fwdPosition')
    def test_play_trajectory_kinematics(self, mock_mj_fwdPosition, fixture_model_path, sample_data):
        """Test playing trajectory in kinematics mode"""
        # Setup player with sample data
        input_str = f"qpos {sample_data['qpos_path']} ctrl {sample_data['ctrl_path']}"
        player = MujocoPlayer(fixture_model_path, mode='kinematics', input_data=input_str)
        
        # Add a mock recorder
        mock_recorder = MagicMock()
        mock_recorder.output_data_freq = 50
        player.add_recorder(mock_recorder)
        
        # Play trajectory
        player.play_trajectory()
        
        # Verify mj_fwdPosition was called for each frame
        assert mock_mj_fwdPosition.call_count == 100  # Number of frames in sample data
        
        # Verify recorder was called appropriate number of times (every output_time_step)
        assert mock_recorder.record_frame.call_count == 10  # 500 / 50 = 10
    
    @pytest.mark.xfail(reason="Bug in MujocoPlayer.play_trajectory() when using dynamics mode with different input data frequency")
    @patch.object(InputDataProcessor, 'process')
    def test_play_trajectory_dynamics(self, mock_process, fixture_model_path, sample_data):
        """Test playing trajectory in dynamics mode"""
        # Setup player with sample data
        input_str = f"qpos {sample_data['qpos_path']} ctrl {sample_data['ctrl_path']}"
        
        # Mock the data processor to return sample data
        mock_process.return_value = {
            'qpos': sample_data['qpos'],
            'ctrl': sample_data['ctrl']
        }
        
        # Create a MagicMock for mj_step
        with patch.object(mujoco, 'mj_step') as mock_mj_step:
            # Create player with mocked data
            player = MujocoPlayer(fixture_model_path, mode='dynamics', input_data=input_str)
            
            # Play trajectory
            player.play_trajectory()
            
            # Verify mj_step was called appropriate number of times
            # For dynamics mode, it should be called at least once per frame
            assert mock_mj_step.call_count > 0
    
    @patch.object(InputDataProcessor, 'process', return_value={})
    def test_save_data(self, mock_process, fixture_model_path, tmp_path):
        """Test saving data from all recorders"""
        output_path = str(tmp_path / "output")
        player = MujocoPlayer(fixture_model_path, output_path=output_path)
        
        # Add mock recorders
        mock_recorder1 = MagicMock()
        mock_recorder2 = MagicMock()
        player.recorders = [mock_recorder1, mock_recorder2]
        
        # Save data
        player.save_data()
        
        # Verify save was called on each recorder
        mock_recorder1.save.assert_called_once_with(output_path, None)
        mock_recorder2.save.assert_called_once_with(output_path, None) 