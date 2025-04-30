Advanced Usage
==============

This section covers advanced usage patterns and examples for MuJoCo Tools.

Custom Data Recording
--------------------

You can customize what data is recorded during simulations:

.. code-block:: python

    from mujoco_tools import MujocoPlayer, StateRecorder
    from mujoco_tools.mujoco_loader import MujocoLoader
    
    # Load model
    loader = MujocoLoader(model_path='path/to/model.xml')
    
    # Create state recorder with custom settings
    recorder = StateRecorder(
        model=loader.model,
        data=loader.data,
        output_path='logs',
        output_prefix='custom_recording',
        record_types=['qpos', 'qvel', 'xpos', 'xquat'],
        format='npy'
    )
    
    # Run simulation with recording
    for i in range(1000):
        loader.step()
        recorder.record()
    
    # Save recorded data
    recorder.save()

Custom Visualization
-------------------

Customize visualization with different camera views and rendering options:

.. code-block:: python

    from mujoco_tools import MujocoPlayer, VideoRecorder
    from mujoco_tools.mujoco_loader import MujocoLoader
    import mujoco as mj
    
    # Load model
    loader = MujocoLoader(model_path='path/to/model.xml')
    
    # Create player with custom settings
    player = MujocoPlayer(
        model=loader.model,
        data=loader.data,
        width=1920,
        height=1080,
        camera_name='side'  # Use a specific camera
    )
    
    # Add custom visualization flags
    player.add_vis_flag(mj.mjtVisFlag.mjVIS_JOINT)
    player.add_vis_flag(mj.mjtVisFlag.mjVIS_ACTUATOR)
    
    # Create video recorder
    recorder = VideoRecorder(
        model=loader.model,
        data=loader.data,
        output_path='videos/custom.mp4',
        width=1920,
        height=1080,
        fps=60
    )
    
    # Run simulation with recording
    for i in range(1000):
        loader.step()
        player.render()
        recorder.record()
    
    # Finalize video
    recorder.close()

Bash Script Examples
-------------------

Creating a sophisticated bash script for batch processing:

.. code-block:: bash

    #!/bin/bash
    
    # Define variables
    MODEL_DIR="models"
    OUTPUT_DIR="results"
    RESOLUTIONS=("720p" "1080p" "4K")
    CAMERAS=("free" "side" "top")
    
    # Create output directory if it doesn't exist
    mkdir -p "$OUTPUT_DIR"
    
    # Process each model
    for MODEL in "$MODEL_DIR"/*.xml; do
      MODEL_NAME=$(basename "$MODEL" .xml)
      
      # Process each camera
      for CAMERA in "${CAMERAS[@]}"; do
        # Process each resolution
        for RES in "${RESOLUTIONS[@]}"; do
          OUTPUT="${OUTPUT_DIR}/${MODEL_NAME}_${CAMERA}_${RES}.mp4"
          
          echo "Processing $MODEL_NAME with camera $CAMERA at $RES"
          
          # Build and run command
          mujoco-tools \
            -m "$MODEL" \
            --camera "$CAMERA" \
            --resolution "$RES" \
            --record_video \
            --output_path "$OUTPUT" \
            --record_data \
            --format "npy" \
            --datatype "qpos qvel xpos xquat"
        done
      done
    done
    
    echo "All processing complete!" 