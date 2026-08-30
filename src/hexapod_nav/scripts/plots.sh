#!/bin/bash
# plots.sh - Launch rqt_plot windows for all numeric topic data
# Opens 4 separate plot windows for different data streams
# Usage: ./plots.sh

echo "[plots] Launching velocity pipeline plot..."
rqt_plot /cmd_vel_nav/linear/x /cmd_vel_nav/angular/z /cmd_vel/linear/x /cmd_vel/angular/z &
P1=$!

echo "[plots] Launching odom twist plot..."
rqt_plot /odom/twist/twist/linear/x /odom/twist/twist/linear/y /odom/twist/twist/angular/z &
P2=$!

echo "[plots] Launching IMU plot..."
rqt_plot /imu/angular_velocity/x /imu/angular_velocity/y /imu/angular_velocity/z &
P3=$!

echo "[plots] Launching leg phase + joint commands plot..."
rqt_plot /leg_phase/data /spooder_controller/commands/data &
P4=$!

echo "[plots] All 4 plot windows open. Ctrl+C to close all."

cleanup() {
    kill $P1 $P2 $P3 $P4 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM
wait
