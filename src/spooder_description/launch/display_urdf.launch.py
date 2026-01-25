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
    
    # Simple Robot Description (no config_file arg needed for pure viz if we default it)
    robot_description_content = Command(['xacro ', xacro_file, ' config_file:=dummy.yaml'])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch_ros.parameter_descriptions.ParameterValue(robot_description_content, value_type=str)}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_spooder_description, 'rviz', 'config.rviz')]
        )
    ])
