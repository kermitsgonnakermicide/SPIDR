#!/usr/bin/env bash
# Non-headless Gazebo + Foxglove bringup. Wrapper around the canonical
# scripts/run_oakd_lite.sh sim entry, with two additions on top:
#   1. foxglove_bridge on ws://localhost:8765  (so you can connect from
#      foxglove.dev or the desktop app and load spooder_walk.foxglove).
#   2. After boot settles, fires one sample /goal_pose so the gait kicks in.
#
# Logs: /tmp/spooder_run_fg/.    Open: https://app.foxglove.dev ->
#   ws://localhost:8765 -> File > Open Layout > spooder_walk.foxglove.
#
# Bind-params note: the canonical run_oakd_lite.sh sim spawns at z=0.5
# which lets the robot settle into the leg resting pose without the
# 3 m slam-down observed in older bringups.
set -E
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GZ_IP=127.0.0.1

LOG_DIR=/tmp/spooder_run_fg
mkdir -p "${LOG_DIR}"
rm -f "${LOG_DIR}"/*.log "${LOG_DIR}"/*.pid

# Kill leftovers from any prior runs.
pkill -9 -f "ros2|gz sim|controller_manager|gait|ekf|octomap|foothold|topic_tools|oak_bringup|foxglove_bridge" || true
sleep 2

echo "[$(date +%H:%M:%S)] (1/3) Canonical sim (Gazebo GUI + perception + Nav2 + RViz)"
scripts/run_oakd_lite.sh sim >> "${LOG_DIR}/01_sim.log" 2>&1 &
echo $! > "${LOG_DIR}/01_sim.pid"

echo "[$(date +%H:%M:%S)] (2/3) Foxglove bridge (ws://localhost:8765)"
/opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge --port 8765 \
    > "${LOG_DIR}/02_foxglove.log" 2>&1 &
echo $! > "${LOG_DIR}/02_foxglove.pid"

# Wait for the bringup to settle (Gazebo spawn t=8s + controllers t=16s +
# EKF t=18s + Nav2 t=22s) before pushing a goal. Pushing too early stalls in
# the controller_manager because no controller has loaded joint_states yet.
echo "[$(date +%H:%M:%S)] (3/3) Waiting 35s for bringup, then firing Nav2 /goal_pose"
sleep 35

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{"header":{"frame_id":"map"},"pose":{"position":{"x":2.0,"y":0.6,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}' \
  > "${LOG_DIR}/03_goal.log" 2>&1 &
echo $! > "${LOG_DIR}/03_goal.pid"

echo "[$(date +%H:%M:%S)] All launched."
echo
echo "Logs in ${LOG_DIR}/"
ls -la "${LOG_DIR}"/*.log
echo
echo "Foxglove: open https://app.foxglove.dev (or desktop) and connect to ws://localhost:8765"
echo "  Then File > Open Layout > /home/daksh/spooder_ws/spooder_walk.foxglove"
echo "RViz: sim.rviz launched automatically by run_oakd_lite.sh sim"
