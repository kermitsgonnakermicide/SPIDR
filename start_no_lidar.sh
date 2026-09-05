#!/usr/bin/env bash
# Lidar-free, no-SLAM start script. Headless test mode.
set -E
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

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
# spawn_z=0.30: legs start horizontal but the body has only ~16cm to fall
# before the gait controller takes over (was 1.0m, which fully pancaked
# the robot during the bringup delay and the gait never recovered).
ros2 launch spooder_gazebo 02_robot_spawn.launch.py spawn_x:=0.0 spawn_y:=0.6 spawn_z:=0.30 use_hexapod_nav:=true > $LOG_DIR/02_spawn.log 2>&1 &
echo $! > $LOG_DIR/02_spawn.pid
sleep 25

# 3. Static map->odom transform (lidar/SLAM removed).
# Use the sim-aware helper script: the stock CLI static_transform_publisher
# latches its /tf_static message at node construction time using
# ``node.get_clock().now()``, which returns 0 (NOT the sim clock) when /clock
# has not yet been received. That stamp-0 latched transform then shows up
# as a far-future message relative to sim-time, triggering tf2_buffer
# "Detected jump back in time" flooding during the first ~6 s of bringup.
# sim_static_tf.py waits for a real /clock message before publishing.
echo "[$(date +%H:%M:%S)] Static map->odom transform"
python3 "$SCRIPT_DIR/sim_static_tf.py" \
  --frame-id map --child-frame-id spooder/odom \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --use-sim-time --wait-timeout 30 > $LOG_DIR/03_tf.log 2>&1 &
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
