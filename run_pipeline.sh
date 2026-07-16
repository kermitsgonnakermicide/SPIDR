#!/bin/bash
# Unified run script for hexapod_nav pipeline
# Usage: ./run_pipeline.sh [sim|real] [world_name]

set -e

WORKSPACE="/home/daksh/spooder_ws"
MODE=${1:-real}
WORLD=${2:-test_world}

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=== Hexapod 3D Navigation Pipeline ==="
echo "Mode: $MODE"
if [ "$MODE" = "sim" ]; then
    echo "World: $WORLD"
fi

if [ "$MODE" = "sim" ]; then
    echo "Starting simulation pipeline..."
    echo "--------------------------------------------"
    echo "This launches:"
    echo "  1. Gazebo (spooder_gazebo simulation)"
    echo "  2. Robot spawn + ros2_control"
    echo "  3. EKF localization"
    echo "  4. SLAM Toolbox + Nav2"
    echo "  5. hexapod_nav pipeline (OctoMap → Terrain → Cost → Foothold → Gait)"
    echo "  6. RViz (hexapod_nav sim.rviz)"
    echo "--------------------------------------------"
    ros2 launch hexapod_nav simulation.launch.py world:=$WORLD
else
    echo "Starting real robot pipeline..."
    echo "--------------------------------------------"
    echo "Requires: OAK-D Lite camera connected via USB"
    echo "This launches:"
    echo "  1. OAK-D camera driver"
    echo "  2. OctoMap server"
    echo "  3. hexapod_nav pipeline (OctoMap → Terrain → Cost → Foothold → Gait)"
    echo "  4. Nav2 (body-level path planning only)"
    echo "--------------------------------------------"
    ros2 launch hexapod_nav full_pipeline.launch.py
fi