#!/usr/bin/env bash
# Lidar-free, no-SLAM start script. Headless test mode.
set -E
cd "$(dirname "${BASH_SOURCE[0]}")"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GZ_IP=127.0.0.1

LOG_DIR=/tmp/spooder_run
mkdir -p $LOG_DIR

# Kill old
pkill -9 -f "ros2|gz sim|controller_manager|gait|ekf|octomap|foothold" || true
sleep 2

# 1. Gazebo
echo "[$(date +%H:%M:%S)] Starting Gazebo headless"
ros2 launch spooder_gazebo 01_sim_world.launch.py world:=plain_world headless:=true > $LOG_DIR/01_gz.log 2>&1 &
echo $! > $LOG_DIR/01_gz.pid
sleep 12

# 2. Robot spawn
echo "[$(date +%H:%M:%S)] Spawning robot"
ros2 launch spooder_gazebo 02_robot_spawn.launch.py spawn_x:=0.0 spawn_y:=0.6 spawn_z:=1.0 use_hexapod_nav:=true > $LOG_DIR/02_spawn.log 2>&1 &
echo $! > $LOG_DIR/02_spawn.pid
sleep 25

# 3. Static map->odom transform (lidar/SLAM removed)
echo "[$(date +%H:%M:%S)] Static map->odom transform"
ros2 run tf2_ros static_transform_publisher -- --frame-id map --child-frame-id spooder/odom --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 > $LOG_DIR/03_tf.log 2>&1 &
echo $! > $LOG_DIR/03_tf.pid
sleep 2

# 4. hexapod_nav pipeline
echo "[$(date +%H:%M:%S)] Starting hexapod_nav pipeline"
ros2 launch hexapod_nav simulation.launch.py > $LOG_DIR/04_hex.log 2>&1 &
echo $! > $LOG_DIR/04_hex.pid
sleep 10

# 5. Nav2 (lidar-free config)
echo "[$(date +%H:%M:%S)] Starting Nav2"
ros2 launch spooder_navigation navigation.launch.py use_sim_time:=true > $LOG_DIR/05_nav.log 2>&1 &
echo $! > $LOG_DIR/05_nav.pid
sleep 8

echo "[$(date +%H:%M:%S)] All launched. Logs in $LOG_DIR"
ls -la $LOG_DIR/*.log
