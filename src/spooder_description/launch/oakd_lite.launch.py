"""
OAK-D Lite URDF + Robot State Publisher launch for spooder_description.

This launch file brings up the URDF with OAK-D Lite camera included and
publishes the robot state on /tf. It does NOT start the OAK-D driver itself —
that's handled by spooder_perception.

Usage:
  ros2 launch spooder_description oakd_lite.launch.py
  ros2 launch spooder_description oakd_lite.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

import launch_ros.parameter_descriptions


def generate_launch_description():
    pkg_spooder_description = get_package_share_directory('spooder_description')
    xacro_file = os.path.join(pkg_spooder_description, 'urdf', 'spooder.xacro')

    robot_description_content = Command(['xacro ', xacro_file, ' config_file:=dummy.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': launch_ros.parameter_descriptions.ParameterValue(
                        robot_description_content, value_type=str
                    ),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
        ),
    ])
