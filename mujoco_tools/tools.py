import mujoco
import numpy as np
from typing import Optional, Tuple, Union, Dict

def load_model_from_path(model_path: str) -> Tuple[mujoco.MjModel, mujoco.MjData]:
    """
    Load a MuJoCo model from a path and initialize its data.
    
    Args:
        model_path (str): Path to the MuJoCo XML model file
        
    Returns:
        Tuple[mujoco.MjModel, mujoco.MjData]: Model and data objects
    """
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    return model, data

def get_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> np.ndarray:
    """
    Get joint position(s) by joint name.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        joint_name: Name of the joint
        
    Returns:
        np.ndarray: Joint position(s)
    """
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_adr = model.jnt_qposadr[joint_id]
    joint_type = model.jnt_type[joint_id]
    
    if joint_type == mujoco.mjtJoint.mjJNT_FREE:
        return data.qpos[joint_adr:joint_adr+7]  # xyz + quaternion
    elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
        return data.qpos[joint_adr:joint_adr+4]  # quaternion
    else:
        return data.qpos[joint_adr]  # single value

def set_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, value: np.ndarray):
    """
    Set joint position(s) by joint name.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        joint_name: Name of the joint
        value: Position value(s) to set
    """
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_adr = model.jnt_qposadr[joint_id]
    joint_type = model.jnt_type[joint_id]
    
    if joint_type == mujoco.mjtJoint.mjJNT_FREE:
        data.qpos[joint_adr:joint_adr+7] = value
    elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
        data.qpos[joint_adr:joint_adr+4] = value
    else:
        data.qpos[joint_adr] = value

def get_body_pose(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get body position and orientation (quaternion).
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        body_name: Name of the body
        
    Returns:
        Tuple[np.ndarray, np.ndarray]: Position (xyz) and orientation (quaternion)
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    xpos = data.xpos[body_id].copy()
    xquat = data.xquat[body_id].copy()
    return xpos, xquat

def apply_force_to_body(model: mujoco.MjModel, data: mujoco.MjData, body_name: str, 
                       force: np.ndarray, point: Optional[np.ndarray] = None):
    """
    Apply force to a body at specified point (or center of mass if not specified).
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        body_name: Name of the body
        force: Force vector [fx, fy, fz]
        point: Point of application in world coordinates (optional)
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if point is None:
        data.xfrc_applied[body_id, :3] = force
    else:
        mujoco.mj_applyFT(model, data, force, [0, 0, 0], point, body_id, data.xfrc_applied)

def get_sensor_data(model: mujoco.MjModel, data: mujoco.MjData, sensor_name: str) -> np.ndarray:
    """
    Get sensor reading by name.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        sensor_name: Name of the sensor
        
    Returns:
        np.ndarray: Sensor reading
    """
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    sensor_adr = model.sensor_adr[sensor_id]
    sensor_dim = model.sensor_dim[sensor_id]
    return data.sensordata[sensor_adr:sensor_adr+sensor_dim]

def reset_simulation(model: mujoco.MjModel, data: mujoco.MjData):
    """
    Reset simulation to initial state.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
    """
    mujoco.mj_resetData(model, data)

def forward_kinematics(model: mujoco.MjModel, data: mujoco.MjData):
    """
    Compute forward kinematics.
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
    """
    mujoco.mj_forward(model, data)

def get_jacobian(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get body Jacobian (translational and rotational).
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        body_name: Name of the body
        
    Returns:
        Tuple[np.ndarray, np.ndarray]: Translational and rotational Jacobians
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
    return jacp, jacr

def launch_passive_viewer(model: mujoco.MjModel, data: mujoco.MjData):
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.sync()

def launch_viewer(model: mujoco.MjModel, data: mujoco.MjData):
    viewer = mujoco.viewer.launch(model, data)