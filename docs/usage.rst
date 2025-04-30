Usage
=====

.. _installation:

Installation
------------

To use MuJoCo Tools, first install it using pip:

.. code-block:: console

   # Install from PyPI
   pip install mujoco-tools
   
   # Or install from github
   pip install git+https://github.com/ShanningZhuang/mujoco_tools.git
   
   # Or install from source
   git clone https://github.com/ShanningZhuang/mujoco_tools.git
   cd mujoco_tools
   pip install -e .

Command Line Usage
-----------------

MuJoCo Tools provides a convenient command-line interface:

.. code-block:: console

   mujoco-tools -m <model.xml> [options]

Required Arguments:
^^^^^^^^^^^^^^^^^^

* ``-m, --model``: Path to MuJoCo XML model file
* ``--mode``: Simulation mode (kinematics: runs mj.fwd_position, dynamics: runs mj.step) [default: kinematics]

Input Data Options:
^^^^^^^^^^^^^^^^^

* ``-d, --data``: Input data type and path (e.g., "qpos data/qpos.npy ctrl data/ctrl.npy") or directly input the path of npz
* ``--input_data_freq``: Frequency of input data [default: 50]

Visualization Options:
^^^^^^^^^^^^^^^^^^^^

* ``--record_video``: Enable video recording
* ``--width``: Video width in pixels [default: 1920]
* ``--height``: Video height in pixels [default: 1080]
* ``--fps``: Video framerate [default: 50]
* ``--camera``: Camera name [default: Free]
* ``--flags``: Custom vision flags (e.g., "mjVIS_ACTUATOR mjVIS_ACTIVATION")

Python Module Usage
------------------

You can also use MuJoCo Tools as a Python module:

.. code-block:: python

   # Direct module import
   from mujoco_tools import MujocoPlayer, VideoRecorder, StateRecorder
   
   # Load a model
   from mujoco_tools.mujoco_loader import MujocoLoader
   loader = MujocoLoader(model_path='path/to/model.xml')
   
   # Create a player for visualization
   player = MujocoPlayer(loader.model, loader.data)

