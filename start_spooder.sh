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
# Localhost Gazebo transport discovery (avoids hung ros_gz_sim create)
export GZ_IP="${GZ_IP:-127.0.0.1}"

# Use the OctoMap/foothold-aware hexapod pipeline by default.
USE_HEXAPOD_NAV="${USE_HEXAPOD_NAV:-1}"
HEX_PREFIX="$(ros2 pkg prefix hexapod_nav)"
HEX_NAV_PARAMS="$HEX_PREFIX/share/hexapod_nav/config/nav2_params.yaml"
HEX_RVIZ_CONFIG="$HEX_PREFIX/share/hexapod_nav/rviz/sim.rviz"

# World selection: set SPOODER_WORLD env var or use first argument.
# Available: test_world, plain_world, rough_terrain, cave_world, maze_world, foothold_terrain
SPOODER_WORLD="${1:-${SPOODER_WORLD:-plain_world}}"

# 1. Start Gazebo with world
log "World: $SPOODER_WORLD"
launch_background ros2 launch spooder_gazebo 01_sim_world.launch.py world:=$SPOODER_WORLD headless:=false
log "Waiting for Gazebo to load..."
sleep 8

# 2. Spawn robot + EKF + controllers (+ skip old gait controller if hexapod_nav)
SPAWN_ARGS=(spawn_x:=0.0 spawn_y:=0.6 spawn_z:=3.0)
if [ "$USE_HEXAPOD_NAV" = "1" ]; then
    SPAWN_ARGS+=(use_hexapod_nav:=true)
fi
launch_background ros2 launch spooder_gazebo 02_robot_spawn.launch.py "${SPAWN_ARGS[@]}"
log "Waiting for robot spawn + EKF + controllers..."
# 02_robot_spawn: create@8s, joint_state@14s, controller@16s, ekf@18s
sleep 22

# 3. Static map->odom transform (SLAM disabled; lidar removed)
# Nav2 needs a map->odom TF; we publish a static identity transform so
# the global frame is effectively the odom frame. This means the costmap
# drifts with odometry, but the robot can still be commanded via /goal_pose.
launch_background ros2 run tf2_ros static_transform_publisher -- --frame-id map --child-frame-id spooder/odom --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
log "Waiting for static transform..."
sleep 2

# 4. Start hexapod_nav OctoMap + terrain pipeline before Nav2 consumes /projected_map.
if [ "$USE_HEXAPOD_NAV" = "1" ]; then
    log "Starting hexapod_nav OctoMap + terrain pipeline..."
    launch_background ros2 launch hexapod_nav simulation.launch.py
    sleep 8
fi

# 5. Start Nav2 (costmaps, planner, controller)
NAV_ARGS=(use_sim_time:=true)
if [ "$USE_HEXAPOD_NAV" = "1" ]; then
    NAV_ARGS+=(params_file:="$HEX_NAV_PARAMS")
fi
launch_background ros2 launch spooder_navigation navigation.launch.py "${NAV_ARGS[@]}"
log "Waiting for Nav2 to activate..."
sleep 5

# 5b. Start Rerun.io bridge (replaces RViz)
launch_background ros2 run hexapod_nav rerun_bridge

# 6. Start RViz (foreground - closing it stops everything)
log "Starting RViz..."
RVIZ_ARGS=(use_sim_time:=true)
if [ "$USE_HEXAPOD_NAV" = "1" ]; then
    RVIZ_ARGS+=(rviz_config:="$HEX_RVIZ_CONFIG")
fi
exec ros2 launch spooder_gazebo 04_viz.launch.py "${RVIZ_ARGS[@]}"
