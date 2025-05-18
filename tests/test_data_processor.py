import pytest
import numpy as np
import os
from mujoco_tools.data_processor import InputDataProcessor, parse_data_arg

class TestParseDataArg:
    def test_empty_string(self):
        """Test that empty string returns empty dict"""
        result = parse_data_arg("")
        assert result == {}
    
    def test_valid_input(self, sample_data):
        """Test parsing valid input string"""
        input_str = f"qpos {sample_data['qpos_path']} ctrl {sample_data['ctrl_path']}"
        result = parse_data_arg(input_str)
        
        assert "qpos" in result
        assert "ctrl" in result
        assert isinstance(result["qpos"], np.ndarray)
        assert isinstance(result["ctrl"], np.ndarray)
        np.testing.assert_array_equal(result["qpos"], sample_data["qpos"])
        np.testing.assert_array_equal(result["ctrl"], sample_data["ctrl"])
    
    def test_txt_file_parsing(self, sample_data):
        """Test parsing text file input"""
        input_str = f"sample {sample_data['txt_path']}"
        result = parse_data_arg(input_str)
        
        assert "sample" in result
        assert isinstance(result["sample"], np.ndarray)
        assert result["sample"].shape[0] == 10  # 10 lines in the sample file
        assert result["sample"].shape[1] == 5   # 5 values per line
    
    def test_invalid_format(self):
        """Test error on invalid file format"""
        with pytest.raises(ValueError) as excinfo:
            parse_data_arg("sample invalid.jpg")
        assert "Unsupported file format" in str(excinfo.value)
    
    def test_uneven_pairs(self):
        """Test error on uneven pairs"""
        with pytest.raises(ValueError) as excinfo:
            parse_data_arg("qpos path1 ctrl")
        assert "Data argument must be pairs" in str(excinfo.value)

class TestInputDataProcessor:
    def test_init_invalid_input(self):
        """Test initialization with invalid input"""
        with pytest.raises(ValueError) as excinfo:
            InputDataProcessor(123).process()
        assert "Input must be a string" in str(excinfo.value)
    
    def test_process_npz(self, sample_data):
        """Test processing NPZ file"""
        processor = InputDataProcessor(sample_data["npz_path"])
        result = processor.process()
        
        assert "qpos" in result
        assert "ctrl" in result
        assert isinstance(result["qpos"], np.ndarray)
        assert isinstance(result["ctrl"], np.ndarray)
        np.testing.assert_array_equal(result["qpos"], sample_data["qpos"])
        np.testing.assert_array_equal(result["ctrl"], sample_data["ctrl"])
    
    def test_process_string_input(self, sample_data):
        """Test processing string input"""
        input_str = f"qpos {sample_data['qpos_path']} ctrl {sample_data['ctrl_path']}"
        processor = InputDataProcessor(input_str)
        result = processor.process()
        
        assert "qpos" in result
        assert "ctrl" in result
        assert isinstance(result["qpos"], np.ndarray)
        assert isinstance(result["ctrl"], np.ndarray)
        np.testing.assert_array_equal(result["qpos"], sample_data["qpos"])
        np.testing.assert_array_equal(result["ctrl"], sample_data["ctrl"])
    
    def test_empty_input(self):
        """Test with empty input"""
        # This test is expected to fail due to a bug in the implementation
        # The InputDataProcessor.__init__ method returns None for empty strings
        # instead of setting self.input_str to an empty string
        processor = InputDataProcessor("")
        result = processor.process()
        assert result == {} 