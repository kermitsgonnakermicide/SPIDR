#!/usr/bin/env bash
# start_sim.sh — One-shot helper for the canonical Spooder sim.
# Builds the workspace if install/ is missing, launches the sim via
# scripts/run_oakd_lite.sh sim, then waits for bringup and runs
# a sanity check on the required nodes.
#
# After this script reports "sim is up", the robot is ready to accept
# /goal_pose. Logs: /tmp/spooder_run/sim.log.
#
# Tear down:
#   pkill -9 -f "ros2|gz sim|controller_manager|gait|ekf|octomap|foothold"
#
# This script does NOT include:
#   - Foxglove bridge           (use start_foxglove.sh for that)
#   - Headless mode             (use start_no_lidar.sh, or pass --headless through run_oakd_lite.sh sim)
#   - Hardware-only mode        (use scripts/run_oakd_lite.sh hardware)
set -E
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

source /opt/ros/jazzy/setup.bash
[[ -f install/setup.bash ]] && source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GZ_IP=127.0.0.1

LOG_DIR=/tmp/spooder_run
mkdir -p "${LOG_DIR}"

# 1. Build if install/ is missing.
if [[ ! -d install/spooder_perception ]]; then
    echo "[$(date +%H:%M:%S)] Building spooder stack..."
    colcon build --symlink-install \
      --packages-select \
        spooder_description spooder_gazebo spooder_perception \
        spooder_navigation spooder_control hexapod_nav
    source install/setup.bash
fi

# 2. Kill anything left over from a prior run.
pkill -9 -f "ros2|gz sim|controller_manager|gait|ekf|octomap|foothold|topic_tools" 2>/dev/null || true
sleep 2

# 3. Boot the canonical sim.
echo "[$(date +%H:%M:%S)] Launching scripts/run_oakd_lite.sh sim  (logs: ${LOG_DIR}/sim.log)"
scripts/run_oakd_lite.sh sim >> "${LOG_DIR}/sim.log" 2>&1 &
echo $! > "${LOG_DIR}/sim.pid"

# 4. Wait for gait_controller_node — once it is alive, joints are
#    commandable and the gait cycle is engaged.
echo -n "[$(date +%H:%M:%S)] Waiting for boot "
for elapsed in $(seq 1 60); do
    sleep 1
    echo -n "."
    if ros2 node list 2>/dev/null | grep -q "gait_controller_node"; then
        echo " gait_controller_node up after ${elapsed}s"
        break
    fi
    if [[ $elapsed -ge 60 ]]; then
        echo " TIMEOUT after 60s — check ${LOG_DIR}/sim.log"
        exit 1
    fi
done

# 5. Sanity check.
echo ""
echo "=== Node check (required) ==="
NODES_OK=0
for n in robot_state_publisher gz sim controller_manager octomap_server \
         octomap_terrain_node terrain_cost_node foothold_planner_node \
         gait_controller_node robot_state_viz bt_navigator planner_server \
         controller_server behavior_server; do
    if ros2 node list 2>/dev/null | grep -q "/${n}\b\| ${n}\b"; then
        echo "  [OK]   $n"
        NODES_OK=$((NODES_OK+1))
    else
        echo "  [----] $n (not yet, may still be starting)"
    fi
done
echo "  ($NODES_OK required nodes up)"
echo ""

echo "=== Topic liveness (3s sample) ==="
for t in /clock /joint_states /oak_d/points /cmd_vel; do
    line=$(timeout 3 ros2 topic hz "$t" 2>&1 | grep -m1 "rate:")
    if [[ -n "$line" ]]; then
        echo "  $t -> $line"
    else
        echo "  $t -> NO PUBLISHERS"
    fi
done

# /cmd_vel_nav must have 0 subscribers — proves the dead relay is gone.
info=$(ros2 topic info /cmd_vel_nav 2>/dev/null)
sub_count=$(echo "$info" | grep "Subscription count:" | awk '{print $3}')
echo ""
echo "=== /cmd_vel_nav subscribers: ${sub_count:-?} (must be 0) ==="
if [[ "${sub_count:-0}" -gt 0 ]]; then
    echo "  WARNING: someone is subscribing to /cmd_vel_nav; that topic should be empty."
fi

echo ""
echo "[$(date +%H:%M:%S)] Sim is up."
echo ""
echo "Send a goal:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "    '{\"header\":{\"frame_id\":\"map\"},\"pose\":{\"position\":{\"x\":2.0,\"y\":0.6,\"z\":0.0},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}}'"
echo ""
echo "Visualize in RViz (already opened by run_oakd_lite.sh):"
echo "    (uses /home/daksh/spooder_ws/src/hexapod_nav/rviz/sim.rviz)"
echo ""
echo "Or visualize in Foxglove: ./start_foxglove.sh (separate boot)"
