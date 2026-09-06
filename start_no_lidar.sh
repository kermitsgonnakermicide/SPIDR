#!/usr/bin/env bash
# Lidar-free, no-SLAM headless simulation. Wrapper around the canonical
# scripts/run_oakd_lite.sh sim --headless entry. The original numbered
# stages (01_gz, 02_spawn, ...) live inside oakd_bringup.launch.py now;
# this shell preserves a single log dir at /tmp/spooder_run/.
#
# Usage:
#   ./start_no_lidar.sh        # Bring up sim in the foreground of this shell.
#   ./start_no_lidar.sh &      # Background; track via /tmp/spooder_run/sim.pid.
#
# Logs: /tmp/spooder_run/sim.log    PIDs: /tmp/spooder_run/sim.pid
set -E
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GZ_IP=127.0.0.1

LOG_DIR=/tmp/spooder_run
mkdir -p "${LOG_DIR}"

# Kill any leftover sim / nav / perception nodes from prior runs.
pkill -9 -f "ros2|gz sim|controller_manager|gait|ekf|octomap|foothold|topic_tools|oak_bringup" || true
sleep 2

echo "[$(date +%H:%M:%S)] Starting headless Spooder sim via scripts/run_oakd_lite.sh sim --headless"
echo "[$(date +%H:%M:%S)] Logs: ${LOG_DIR}/sim.log"

# Canonical entry handles Gazebo world + robot spawn + perception + Nav2 + RViz
# in one ros2 launch. Tracing is single-stream by design — use
# `ros2 node list` / `ros2 topic info` to introspect, or grep the log.
exec scripts/run_oakd_lite.sh sim --headless \
    >> "${LOG_DIR}/sim.log" 2>&1
