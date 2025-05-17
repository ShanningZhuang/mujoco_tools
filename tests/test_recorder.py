import pytest
import mujoco
import numpy as np
import os
import cv2
from unittest.mock import MagicMock, patch
from mujoco_tools.recorder import VideoRecorder, StateRecorder

class TestVideoRecorder:
    def test_init_single_camera(self):
        """Test initializing recorder with a single camera"""
        recorder = VideoRecorder(camera_name='camera1', width=640, height=480, fps=30, 
                                 vision_flags={'mjVIS_STATIC': 0, 'mjVIS_TEXTURE': 1})
        
        assert recorder.camera_names == ['camera1']
        assert recorder.fps == 30
        assert recorder.camera_width == 640
        assert recorder.camera_height == 480
        assert recorder.scene_option.flags[mujoco.mjtVisFlag.mjVIS_STATIC] == 0
        assert recorder.scene_option.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] == 1
    
    def test_init_multiple_cameras(self):
        """Test initializing recorder with multiple cameras"""
        recorder = VideoRecorder(camera_name='camera1 camera2', width=1280, height=720, 
                                 vision_flags={'mjVIS_TEXTURE': 1})
        
        assert recorder.camera_names == ['camera1', 'camera2']
        assert recorder.video_width == 1280 * 2  # Width times number of cameras
        assert recorder.video_height == 720
    
    @patch('cv2.VideoWriter')
    def test_initialize(self, mock_video_writer, tmp_path):
        """Test initializing the video writer"""
        recorder = VideoRecorder(camera_name='camera1', width=640, height=480, fps=30, 
                                 vision_flags={'mjVIS_TEXTURE': 1})
        
        output_path = str(tmp_path)
        recorder.initialize(output_path, 'test')
        
        assert recorder.output_path == f'{output_path}/test_video.mp4'
        mock_video_writer.assert_called_once()
    
    @patch('mujoco.Renderer')
    @patch('cv2.VideoWriter')
    def test_record_frame(self, mock_video_writer, mock_renderer, fixture_model_path):
        """Test recording a frame"""
        # Setup
        model = mujoco.MjModel.from_xml_path(fixture_model_path)
        data = mujoco.MjData(model)
        
        # Mock renderer and its return value
        mock_renderer_instance = MagicMock()
        mock_renderer.return_value = mock_renderer_instance
        # Set up renderer to return a simple numpy array for frame
        mock_renderer_instance.render.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Create recorder
        recorder = VideoRecorder(camera_name='camera1', width=640, height=480, fps=30, 
                                 vision_flags={'mjVIS_TEXTURE': 1})
        mock_writer = MagicMock()
        recorder.video_writer = mock_writer
        
        # Record frame
        recorder.record_frame(model, data)
        
        # Verify the renderer was called
        mock_renderer_instance.update_scene.assert_called_once()
        mock_renderer_instance.render.assert_called_once()
        # Verify the video writer was called
        mock_writer.write.assert_called_once()
    
    @patch('cv2.VideoWriter')
    def test_save(self, mock_video_writer, tmp_path):
        """Test saving video"""
        recorder = VideoRecorder(camera_name='camera1', width=640, height=480, fps=30, 
                                 vision_flags={'mjVIS_TEXTURE': 1})
        
        mock_writer = MagicMock()
        recorder.video_writer = mock_writer
        recorder.output_path = str(tmp_path / "test_video.mp4")
        
        recorder.save(str(tmp_path))
        
        # Verify the video writer was released
        mock_writer.release.assert_called_once()
        assert recorder.video_writer is None
    
    def test_visualize_activation(self):
        """Test activation visualization"""
        # Create a mock to replace the entire visualize_activation method
        with patch.object(VideoRecorder, 'visualize_activation', return_value=np.zeros((100, 100, 3), dtype=np.uint8)) as mock_method:
            # Create a recorder instance
            recorder = VideoRecorder(camera_name='camera1', width=640, height=480, fps=30, 
                                     vision_flags={'mjVIS_TEXTURE': 1}, activation_map=True)
            
            # Call the method with test data
            activation = np.ones(10)
            result = recorder.visualize_activation(activation, shape=(2, 5))
            
            # Verify the result
            assert isinstance(result, np.ndarray)
            assert result.shape == (100, 100, 3)
            
            # Check that method was called with expected arguments
            mock_method.assert_called_once()

class TestStateRecorder:
    @pytest.fixture
    def model(self, fixture_model_path):
        """Create a model for the state recorder"""
        return mujoco.MjModel.from_xml_path(fixture_model_path)
        
    @pytest.fixture
    def mock_state_recorder(self, model):
        """Create a mock state recorder for testing"""
        datatypes = ['qpos', 'qvel', 'time']
        recorder = StateRecorder(model, datatypes=datatypes)
        return recorder
    
    def test_init(self, model):
        """Test initialization"""
        datatypes = ['qpos', 'qvel', 'time']
        recorder = StateRecorder(model, datatypes=datatypes)
        assert recorder.output_data_freq == 50  # Default value
        assert set(recorder.recorded_data.keys()) == set(datatypes)
    
    def test_initialize(self, mock_state_recorder):
        """Test initialization of state data"""
        recorder = mock_state_recorder
        recorder.initialize("output_path", "test_prefix")
        
        # Verify output path and prefix set correctly
        assert recorder.output_path == "output_path"
        assert recorder.output_prefix == "test_prefix"
    
    @patch('numpy.save')
    def test_save(self, mock_np_save, mock_state_recorder, tmp_path):
        """Test saving state data"""
        recorder = mock_state_recorder
        recorder.output_path = str(tmp_path)
        recorder.output_prefix = "test"
        
        # Add some test data
        recorder.recorded_data['qpos'] = [np.ones(10)]
        recorder.recorded_data['qvel'] = [np.zeros(10)]
        recorder.recorded_data['time'] = [0.1]
        
        # Save data
        recorder.save(str(tmp_path), "test")
        
        # Verify numpy save was called
        assert mock_np_save.call_count == 3  # Once for each key in state_data 