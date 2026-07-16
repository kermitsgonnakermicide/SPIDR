#!/usr/bin/env bash
set -Ee -o pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

LAUNCH_PIDS=()

log() {
    printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

cleanup() {
    log "Shutting down all Spooder processes..."
    for pid in "${LAUNCH_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${LAUNCH_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

launch_background() {
    log "Starting: $*"
    setsid "$@" &
    LAUNCH_PIDS+=("$!")
}

source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Kill old processes
if [ "${SPOODER_SKIP_CLEANUP:-0}" != "1" ]; then
    log "Stopping old ROS/Gazebo/RViz processes"
    pkill -9 -f "ros|gz|gazebo|rviz|ruby" || true
    sleep 2
fi
ros2 daemon stop || true
ros2 daemon start
sleep 1

# Build packages
log "Building Spooder packages"
colcon build --symlink-install --packages-select spooder_description spooder_gazebo spooder_navigation spooder_control spooder_perception hexapod_nav
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Use hexapod_nav pipeline (set USE_HEXAPOD_NAV=1 to enable)
USE_HEXAPOD_NAV="${USE_HEXAPOD_NAV:-0}"

# 1. Start Gazebo with world
launch_background ros2 launch spooder_gazebo 01_sim_world.launch.py headless:=false
log "Waiting for Gazebo to load..."
sleep 8

# 2. Spawn robot + EKF + controllers (+ skip old gait controller if hexapod_nav)
SPAWN_ARGS=(spawn_x:=0.0)
if [ "$USE_HEXAPOD_NAV" = "1" ]; then
    SPAWN_ARGS+=(use_hexapod_nav:=true)
fi
launch_background ros2 launch spooder_gazebo 02_robot_spawn.launch.py "${SPAWN_ARGS[@]}"
log "Waiting for robot spawn + EKF + controllers..."
sleep 15

# 3. Start SLAM (provides map -> spooder/odom TF)
launch_background ros2 launch spooder_navigation slam.launch.py
log "Waiting for SLAM to initialize..."
sleep 5

# 4. Start Nav2 (costmaps, planner, controller)
launch_background ros2 launch spooder_navigation navigation.launch.py
log "Waiting for Nav2 to activate..."
sleep 5

# 5. Start hexapod_nav OctoMap + terrain pipeline (if enabled)
if [ "$USE_HEXAPOD_NAV" = "1" ]; then
    log "Starting hexapod_nav OctoMap + terrain pipeline..."
    launch_background ros2 launch hexapod_nav simulation.launch.py
    sleep 8
fi

# 6. Start RViz (foreground - closing it stops everything)
log "Starting RViz..."
exec ros2 launch spooder_gazebo 04_viz.launch.py
