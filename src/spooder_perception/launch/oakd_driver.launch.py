"""
OAK-D Lite hardware driver launch file.

Launches the depthai_ros_driver for a real OAK-D Lite camera connected via USB.
This is for hardware experiments only — for simulation, use oakd_sim.launch.py
or the unified oakd_bringup.launch.py with mode:=simulation.

Usage:
  ros2 launch spooder_perception oakd_driver.launch.py
  ros2 launch spooder_perception oakd_driver.launch.py params_file:=<custom.yaml>

The driver publishes:
  /oak_d/points        - PointCloud2 (depth point cloud)
  /oak_d/stereo/image - Depth image
  /oak_d/rgb/image    - RGB image
  /oak_d/imu          - IMU data
  /oak_d/left/camera_info, /oak_d/right/camera_info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_spooder_perception = get_package_share_directory('spooder_perception')

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_spooder_perception, 'config', 'oakd_lite.yaml'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_spooder_perception, 'config', 'oakd_lite.yaml'),
            description='Full path to OAK-D Lite params YAML file',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='oak_d',
            description='Camera namespace/prefix',
        ),
        DeclareLaunchArgument(
            'parent_frame',
            default_value='spooder/base_link',
            description='TF parent frame for the OAK-D camera',
        ),

        # depthai_ros_driver composable node container
        Node(
            package='rclcpp_components',
            executable='component_container',
            name='oak_d_container',
            output='screen',
        ),

        # OAK-D driver
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak_d',
            namespace=LaunchConfiguration('camera_name'),
            output='screen',
            parameters=[params_file],
        ),

        # Robot State Publisher for OAK-D URDF (for TF tree + RViz)
        # This is needed because depthai_ros_driver's built-in RSP expects
        # depthai_descriptions URDF, but we want our custom one.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='oak_d_state_publisher',
            namespace=LaunchConfiguration('camera_name'),
            output='screen',
            parameters=[{
                'robot_description':
                    # Use depthai_descriptions for the OAK-D Lite URDF
                    f'<?xml version="1.0"?><robot name="oak_d_lite">'
                    f'<link name="oak_d_base_frame"/></robot>',
                'use_sim_time': False,
            }],
        ),
    ])
