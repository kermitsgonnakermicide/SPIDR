import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_spooder_navigation = get_package_share_directory('spooder_navigation')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    # SLAM disabled: lidar removed. Static odom->map identity is published
    # elsewhere (Gazebo diff-drive plugin) so Nav2 can navigate in odom frame.

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)

    return ld
