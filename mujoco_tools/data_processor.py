import numpy as np
from typing import Dict

def parse_data_arg(data_str: str) -> Dict[str, str]:
    """Parse data argument string into a dictionary
    Example: "qpos data/qpos.npy ctrl data/ctrl.npy" -> {"qpos": "data/qpos.npy", "ctrl": "data/ctrl.npy"}
    """
    if not data_str:
        return {}
        
    parts = data_str.split()
    if len(parts) % 2 != 0:
        raise ValueError("Data argument must be pairs of type and path")
    data_name = parts[::2]
    data_path = parts[1::2]
    data_array = []
    
    for path in data_path:
        if path.endswith('.npy'):
            data_array.append(np.load(path))
        elif path.endswith('.txt'):
            # Read the text file and process each line
            with open(path, 'r') as f:
                lines = f.readlines()
            # Process each line: split by comma, convert to float
            processed_data = [
                [float(num) for num in line.strip().split(',')]
                for line in lines
            ]
            # Convert to numpy array
            data_array.append(np.array(processed_data))
        else:
            raise ValueError(f"Unsupported file format for {path}. Must be .npy or .txt")
            
    return dict(zip(data_name, data_array))
class InputDataProcessor:
    def __init__(self, input_str: str):
        """Initialize with a string input
        
        Args:
            input_str: Either a path to .npz file or string in format "type1 path1 type2 path2"
        """
        if not input_str:
            self.input_str = ""
            return None
        if not isinstance(input_str, str):
            raise ValueError("Input must be a string")
        self.input_str = input_str
        
    def process(self) -> Dict[str, np.ndarray]:
        """Process the input string into a standardized dictionary format
        
        Returns:
            dict: Dictionary mapping data types to their corresponding numpy arrays
        """
        if self.input_str.endswith('.npz'):
            return self._process_npz(self.input_str)
        return parse_data_arg(self.input_str)
    
    def _process_npz(self, npz_path: str) -> Dict[str, np.ndarray]:
        """Extract all arrays from a .npz file into a dictionary
        
        Args:
            npz_path: Path to the .npz file
            
        Returns:
            dict: Dictionary mapping array names to numpy arrays
        """
        key_list = ["qpos", "qvel", "ctrl","self.data.qpos","self.data.qvel","self.data.ctrl"]
        with np.load(npz_path) as data:
            return {key: data[key] for key in key_list if key in data}