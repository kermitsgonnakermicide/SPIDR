#!/bin/bash
set -Ee -o pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

LAUNCH_PIDS=()
log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
cleanup() {
    for pid in "${LAUNCH_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    done
    sleep 1
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
[ -f "install/setup.bash" ] && source install/setup.bash

log "Stopping old processes"
pkill -9 -f "ros|gz|gazebo|rviz|ruby" || true
sleep 1
ros2 daemon stop || true
ros2 daemon start
sleep 1

log "Building packages"
colcon build --symlink-install --packages-select spooder_description spooder_gazebo spooder_navigation spooder_control spooder_perception spooder_foothold
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

log "Launching foothold_terrain world with robot at (1,0)"
launch_background ros2 launch spooder_gazebo 01_sim_world.launch.py world:=foothold_terrain headless:=false
sleep 5

launch_background ros2 launch spooder_gazebo 02_robot_spawn.launch.py spawn_x:=1.0 spawn_y:=0.0
sleep 12

log "Starting SLAM"
launch_background ros2 launch spooder_navigation slam.launch.py
sleep 5

log "Starting OctoMap + terrain"
launch_background ros2 launch spooder_perception perception.launch.py
sleep 6

log "Starting Nav2 + OctoMap planner"
launch_background ros2 launch spooder_navigation navigation.launch.py
sleep 5

log "Starting foothold optimizer (Phase 2: multi-objective costs)"
launch_background ros2 run spooder_foothold foothold_optimizer
sleep 2

log "Starting RViz"
setsid ros2 launch spooder_gazebo 04_viz.launch.py &
LAUNCH_PIDS+=("$!")
sleep 3

log "=== STARTUP COMPLETE ==="
log "Robot at (1,0) facing terrain sections (stepped platforms at x=2-4, y=-1)"
log ""
log "  To set a 3D goal ON TOP OF THE 0.25m STEP:"
log "    ros2 topic pub /spooder/goal_3d geometry_msgs/PoseStamped \\"
log "      '{header: {frame_id: map}, pose: {position: {x: 4.0, y: -0.5, z: 0.25}, orientation: {w: 1}}}'"
log ""
log "  Or set a 2D goal via RViz (z=0, body lift inferred from terrain)"
log "  Teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
log ""
log "  Visualize: /spooder/foothold_candidates (green points, map frame)"
log "             /spooder/foothold_selected (red spheres, map frame)"
log "             /spooder/target_body_height (Float32)"
log ""
log "  Speed: stride_amp=0.3 (was 0.07), gait_speed=6 (was 4) => ~4x faster"
log "  New cost families: geometric + stability + kinematic + energy"
log ""

wait
