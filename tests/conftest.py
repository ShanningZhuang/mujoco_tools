import os
import pytest
import numpy as np
import mujoco
from pathlib import Path

# Define the path to the fixture model
@pytest.fixture
def fixture_model_path():
    """Path to a simple test model XML"""
    # Use the local example model 
    model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "examples", "models", "humanoid", "humanoid.xml")
    # Verify the file exists
    assert os.path.exists(model_path), f"Model file not found at {model_path}"
    return model_path

@pytest.fixture
def test_data_path():
    """Path to test data directory"""
    return os.path.join(os.path.dirname(__file__), "test_data")

@pytest.fixture
def sample_data(test_data_path):
    """Generate and save sample data for testing"""
    # Create directory if it doesn't exist
    os.makedirs(test_data_path, exist_ok=True)
    
    # Sample data: 100 frames of random data
    frames = 100
    qpos = np.random.random((frames, 28))  # Example dimensions for humanoid
    ctrl = np.random.random((frames, 21))  # Example dimensions for humanoid
    
    # Save to files
    qpos_path = os.path.join(test_data_path, "qpos.npy")
    ctrl_path = os.path.join(test_data_path, "ctrl.npy")
    
    np.save(qpos_path, qpos)
    np.save(ctrl_path, ctrl)
    
    # Create a sample txt file
    txt_path = os.path.join(test_data_path, "sample.txt")
    with open(txt_path, "w") as f:
        for i in range(10):
            f.write(",".join([str(x) for x in np.random.random(5)]) + "\n")
    
    # Create a test npz file
    npz_path = os.path.join(test_data_path, "test_data.npz")
    np.savez(npz_path, qpos=qpos, ctrl=ctrl)
    
    return {
        "qpos_path": qpos_path,
        "ctrl_path": ctrl_path,
        "txt_path": txt_path,
        "npz_path": npz_path,
        "qpos": qpos,
        "ctrl": ctrl
    }

@pytest.fixture
def output_path(tmp_path):
    """Create a temporary directory for test output files"""
    return str(tmp_path / "output") 