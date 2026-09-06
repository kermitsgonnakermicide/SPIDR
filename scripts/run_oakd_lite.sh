#!/usr/bin/env bash
# run_oakd_lite.sh - Canonical entry point for the Spooder hexapod simulation/hardware.
# Usage: ./run_oakd_lite.sh [mode] [options]
#
# Modes:
#   sim        - Full Spooder simulation (Gazebo + OAK-D Lite + perception + Nav2)
#   hardware   - Run OAK-D Lite driver only on real hardware (no Gazebo, no Nav2)
#   sim_only   - Just Gazebo + Spooder robot, no perception or Nav2
#
# Options:
#   --record        Enable ROS 2 bag recording
#   --no-octomap    Skip OctoMap terrain pipeline
#   --headless      Run Gazebo headless (no GUI)
#   --world NAME    Gazebo world name (default: plain_world)
#
# Examples:
#   ./run_oakd_lite.sh                    # Full Spooder simulation
#   ./run_oakd_lite.sh sim --record       # Full sim + record bag
#   ./run_oakd_lite.sh hardware           # Just OAK-D driver on real hw
#   ./run_oakd_lite.sh sim --headless     # Headless full simulation
#   ./run_oakd_lite.sh sim_only --headless  # Just Gazebo, headless

set -eo pipefail
# Disable -u (unbound variable check) to avoid issues with ROS setup scripts
# that reference env vars like AMENT_TRACE_SETUP_FILES before they are set.

cd "$(dirname "${BASH_SOURCE[0]}")"

# Source ROS 2 Jazzy and workspace
source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Default values
MODE="sim"
RECORD="false"
USE_OCTOMAP="true"
HEADLESS="false"
WORLD="plain_world"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        sim|hardware|sim_only)
            MODE="$1"
            shift
            ;;
        --record)
            RECORD="true"
            shift
            ;;
        --no-octomap)
            USE_OCTOMAP="false"
            shift
            ;;
        --headless)
            HEADLESS="true"
            shift
            ;;
        --world)
            WORLD="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [mode] [options]"
            echo ""
            echo "Modes:"
            echo "  sim        Full Spooder simulation (Gazebo + OAK-D Lite + Nav2)"
            echo "  hardware   Run OAK-D Lite driver on real hardware (no sim/Nav2)"
            echo "  sim_only   Just Gazebo + Spooder robot"
            echo ""
            echo "Options:"
            echo "  --record        Enable ROS 2 bag recording"
            echo "  --no-octomap    Skip OctoMap terrain pipeline"
            echo "  --headless      Run Gazebo headless (no GUI)"
            echo "  --world NAME    Gazebo world name (default: plain_world)"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

# Build packages if needed
if [ ! -d "install/spooder_perception" ]; then
    echo "Building spooder_perception package..."
    colcon build --symlink-install --packages-select spooder_perception
    source install/setup.bash
fi

# Map mode to launch file
case "$MODE" in
    sim)
        echo "================================================================"
        echo "  Starting full Spooder simulation"
        echo "    Gazebo + Spooder hexapod + OAK-D Lite + Nav2"
        echo "    World: $WORLD   Record: $RECORD   OctoMap: $USE_OCTOMAP   Headless: $HEADLESS"
        echo "================================================================"
        ros2 launch spooder_perception oakd_bringup.launch.py \
            mode:=simulation \
            record:="$RECORD" \
            use_octomap:="$USE_OCTOMAP" \
            use_sim_time:=true \
            world:="$WORLD" \
            headless:="$HEADLESS"
        ;;
    hardware)
        echo "================================================================"
        echo "  Starting OAK-D Lite hardware driver only"
        echo "  (No Gazebo, no Nav2 — just the camera on real hardware)"
        echo "================================================================"
        ros2 launch spooder_perception oakd_driver.launch.py
        ;;
    sim_only)
        echo "================================================================"
        echo "  Starting Spooder in Gazebo (no perception, no Nav2)"
        echo "    World: $WORLD   Headless: $HEADLESS"
        echo "================================================================"
        ros2 launch spooder_gazebo 01_sim_world.launch.py \
            world:="$WORLD" \
            headless:="$HEADLESS"
        ;;
esac
