#!/bin/bash
set -Ee -o pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

LAUNCH_PIDS=()

log() {
    printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

cleanup() {
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

log "Stopping old processes"
pkill -9 -f "ros|gz|gazebo|rviz|ruby|python3" || true
sleep 2
ros2 daemon stop || true
ros2 daemon start
sleep 1

log "Building packages"
colcon build --symlink-install --packages-select spooder_description spooder_gazebo spooder_navigation spooder_control spooder_perception spooder_foothold
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

log "Launching foothold_terrain world"
launch_background ros2 launch spooder_gazebo 01_sim_world.launch.py world:=foothold_terrain headless:=false
sleep 5

log "Spawning robot"
launch_background ros2 launch spooder_gazebo 02_robot_spawn.launch.py spawn_x:=1.0 spawn_y:=0.0
sleep 10

log "Starting SLAM"
launch_background ros2 launch spooder_navigation slam.launch.py
sleep 5

log "Starting perception (OctoMap)"
launch_background ros2 launch spooder_perception perception.launch.py
sleep 6

log "Starting Nav2"
launch_background ros2 launch spooder_navigation navigation.launch.py
sleep 5

log "Starting foothold optimizer"
launch_background ros2 run spooder_foothold foothold_optimizer
sleep 2

log "Starting RViz"
setsid ros2 launch spooder_gazebo 04_viz.launch.py &
RVIZ_PID="$!"
LAUNCH_PIDS+=("$RVIZ_PID")
sleep 3
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    log "RViz exited early, check error above."
    wait "$RVIZ_PID"
fi

sleep 4
log "Startup complete. Robot at (1,0) facing foothold terrain sections."
log "Use 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' to drive."
log "Watch /spooder/foothold_candidates and /spooder/foothold_selected in RViz."
wait "$RVIZ_PID"
