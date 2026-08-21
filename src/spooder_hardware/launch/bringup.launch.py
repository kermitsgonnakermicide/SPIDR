import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('spooder_hardware')
    default_hw = os.path.join(pkg_share, 'config', 'hardware.yaml')
    default_ctrl = os.path.join(pkg_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=default_hw),
        DeclareLaunchArgument('controllers_config', default_value=default_ctrl),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                LaunchConfiguration('hardware_config'),
                LaunchConfiguration('controllers_config'),
            ],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager',
                       '/controller_manager'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['spooder_controller', '--controller-manager',
                       '/controller_manager'],
            output='screen',
        ),
    ])
