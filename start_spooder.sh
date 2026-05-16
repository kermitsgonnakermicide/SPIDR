#!/usr/bin/env bash
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

require_ros_pkg() {
    local pkg="$1"
    local apt_pkg="$2"

    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        log "Missing ROS package: $pkg"
        log "Install it with: sudo apt-get install -y $apt_pkg"
        exit 1
    fi
}

require_ros_pkg gz_ros2_control ros-jazzy-gz-ros2-control
require_ros_pkg controller_manager ros-jazzy-controller-manager
require_ros_pkg position_controllers ros-jazzy-position-controllers
require_ros_pkg joint_state_broadcaster ros-jazzy-joint-state-broadcaster


if [ "${SPOODER_SKIP_CLEANUP:-0}" = "1" ]; then
    log "Skipping old process cleanup"
else
    log "Stopping old ROS/Gazebo/RViz processes"
    pkill -9 -f "ros|gz|gazebo|rviz|ruby|python3" || true
    sleep 2
fi
ros2 daemon stop || true
ros2 daemon start
sleep 1

if [ -d "src/octomap_mapping/octomap_server" ]; then
    log "Building local OctoMap source packages"
    colcon build --symlink-install --packages-select octomap_server octomap_mapping --cmake-args -DBUILD_TESTING=OFF
    source install/setup.bash
fi

if ! ros2 pkg prefix octomap_server >/dev/null 2>&1; then
    log "Missing ROS package: octomap_server"
    log "Install it with: sudo apt-get install -y ros-jazzy-octomap-server ros-jazzy-octomap-ros"
    log "Or clone source with: git clone --branch ros2 https://github.com/OctoMap/octomap_mapping.git src/octomap_mapping"
    exit 1
fi

log "Building Spooder packages"
colcon build --symlink-install --packages-select spooder_description spooder_gazebo spooder_navigation spooder_control spooder_perception
source install/setup.bash


export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CLOUD_TOPIC="${SPOODER_CLOUD_TOPIC:-/camera/points}"
OPTIMIZED_CLOUD_TOPIC="${SPOODER_OPTIMIZED_CLOUD_TOPIC:-/camera/points/optimized}"

launch_background ros2 launch spooder_gazebo 01_sim_world.launch.py headless:=false
sleep 5

launch_background ros2 launch spooder_gazebo 02_robot_spawn.launch.py spawn_x:=1.0
sleep 10 # Wait for spawner and EKF to stabilize

launch_background ros2 launch spooder_navigation slam.launch.py
sleep 5

launch_background ros2 launch spooder_perception perception.launch.py cloud_topic:="$CLOUD_TOPIC" optimized_cloud_topic:="$OPTIMIZED_CLOUD_TOPIC"
sleep 6

launch_background ros2 launch spooder_navigation navigation.launch.py
sleep 5

log "Starting RViz"
setsid ros2 launch spooder_gazebo 04_viz.launch.py &
RVIZ_PID="$!"
LAUNCH_PIDS+=("$RVIZ_PID")
sleep 3
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    log "RViz exited during startup; check the RViz error above."
    wait "$RVIZ_PID"
fi

sleep 4 # Let Nav2 subscribe to the OctoMap terrain layer before driving.

log "Startup complete. RViz is running; close RViz or press Ctrl-C to stop Spooder."
wait "$RVIZ_PID"
