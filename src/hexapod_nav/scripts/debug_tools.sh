#!/bin/bash
# debug_tools.sh - Launch RViz + rqt_plot for hexapod debugging
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/../rviz/sim.rviz"

cleanup() { kill $(jobs -p) 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

case "${1:-all}" in
    --rviz-only)
        rviz2 -d "$RVIZ_CONFIG" &
        wait
        ;;
    --plots-only)
        rqt_plot /cmd_vel_nav/linear/x /cmd_vel_nav/angular/z /cmd_vel/linear/x /cmd_vel/angular/z &
        rqt_plot /odom/twist/twist/linear/x /odom/twist/twist/angular/z &
        rqt_plot /imu/angular_velocity/x /imu/angular_velocity/y /imu/angular_velocity/z &
        rqt_plot /leg_phase/data /spooder_controller/commands/data &
        wait
        ;;
    --rerun)
        rviz2 -d "$RVIZ_CONFIG" &
        python3 "$SCRIPT_DIR/rerun_bridge.py" &
        wait
        ;;
    all|*)
        rviz2 -d "$RVIZ_CONFIG" &
        sleep 1
        rqt_plot /cmd_vel_nav/linear/x /cmd_vel_nav/angular/z /cmd_vel/linear/x /cmd_vel/angular/z &
        rqt_plot /odom/twist/twist/linear/x /odom/twist/twist/angular/z &
        rqt_plot /imu/angular_velocity/x /imu/angular_velocity/y /imu/angular_velocity/z &
        python3 "$SCRIPT_DIR/rerun_bridge.py" &
        wait
        ;;
esac
