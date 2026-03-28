#!/bin/bash

# 1. Define the single log file where everything will be appended
LOG_FILE="$(pwd)/run_logs/simulation.log"

# 2. Ensure log directory exists
mkdir -p "$(pwd)/run_logs"

# 3. Set the required robot model
export TURTLEBOT3_MODEL=burger

echo "================================================="
echo "Starting LQR Controller Simulation..."
echo "All runtime logs are being appended to: $LOG_FILE"
echo "================================================="

# 4. Launch the simulation and pipe terminal output directly to the single file
# Use tee -a to append to the file while still showing output in the terminal
ros2 launch nav2_bringup tb3_simulation_launch.py \
    params_file:="$(pwd)/config/nav2_params.yaml" \
    headless:=False 2>&1 | tee -a "$LOG_FILE"
