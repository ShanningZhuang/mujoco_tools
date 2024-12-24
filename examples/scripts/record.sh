#!/bin/bash

# Default values
MODEL_PATH="../models/humanoid/humanoid.xml"
MODE="kinematics"
INPUT_DATA_FREQ=50
OUTPUT_PATH="logs"
OUTPUT_PREFIX="test"
DATA="qpos logs/input_qpos.npy ctrl logs/input_ctrl.npy"

# Video settings
CAMERA="side_all"
WIDTH=1920
HEIGHT=1080
FPS=50
OUTPUT_VIDEO_FREQ=50

# Data recording settings
FORMAT="npy"
DATATYPES="qpos qvel xpos xquat"
OUTPUT_DATA_FREQ=50

# Vision flags (optional)
VISION_FLAGS="mjVIS_ACTUATOR mjVIS_ACTIVATION"

# Build command
CMD="python cli.py \
    --model \"$MODEL_PATH\" \
    --mode \"$MODE\" \
    --input_data_freq \"$INPUT_DATA_FREQ\" \
    --output_path \"$OUTPUT_PATH\" \
    --output_prefix \"$OUTPUT_PREFIX\" \
    --data \"$DATA\""



# Add video recording options
CMD+=" --record_video"
CMD+=" --camera \"$CAMERA\""
CMD+=" --width \"$WIDTH\""
CMD+=" --height \"$HEIGHT\""
CMD+=" --fps \"$FPS\""
CMD+=" --output_video_freq \"$OUTPUT_VIDEO_FREQ\""
[[ -n "$VISION_FLAGS" ]] && CMD+=" --flags \"$VISION_FLAGS\""

# Add data recording options
CMD+=" --record_data"
CMD+=" --format \"$FORMAT\""
CMD+=" --datatype \"$DATATYPES\""
CMD+=" --output_data_freq \"$OUTPUT_DATA_FREQ\""

# Execute the command
echo "$CMD"
eval "$CMD"