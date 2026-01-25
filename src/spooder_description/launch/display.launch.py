import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('spooder_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'spooder.xacro')
    # Use the existing Nav2 RViz config or a simple one if available. 
    # For now, we point to the one in spooder_slam or create a default if needed.
    # The tutorial mentions 'config.rviz'. We'll assume user might want one. 
    # Let's trust spooder_slam's for now or just generic.
    
    # Actually, let's look for one in spooder_description if it exists, else spooder_slam's.
    # I'll default to standard view if none provided, but the tutorial creates one.
    # I'll generate a basic one if I can't find one, but for now let's just default to empty/standard logic.
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui', 
            default_value='True', 
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path, 
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path, 
            description='Absolute path to rviz config file'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        )
    ])
