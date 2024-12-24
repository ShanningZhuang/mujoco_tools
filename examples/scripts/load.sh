#!/bin/bash

# Default values
MODEL_PATH="../models/humanoid/humanoid.xml"

# Build command
CMD="python load.py \
    --model \"$MODEL_PATH\""

# Execute the command
echo "$CMD"
eval "$CMD"
