#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi



pkill -9 -f "ros|gz|gazebo|rviz|ruby|python3" || true
sleep 2
ros2 daemon stop && ros2 daemon start
sleep 1

colcon build --symlink-install --packages-select spooder_description spooder_gazebo spooder_navigation spooder_control
source install/setup.bash


export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch spooder_gazebo 01_sim_world.launch.py &
sleep 5

ros2 launch spooder_gazebo 02_robot_spawn.launch.py &
sleep 10 # Wait for spawner and EKF to stabilize

ros2 launch spooder_navigation slam.launch.py &
sleep 5

ros2 launch spooder_navigation navigation.launch.py &
sleep 10 # Wait for lifecycle manager to activate all 11 nodes

ros2 launch spooder_gazebo 04_viz.launch.py
