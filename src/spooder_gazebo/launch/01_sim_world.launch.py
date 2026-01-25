import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spooder_gazebo = get_package_share_directory('spooder_gazebo')

    # Configs
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='test_world')
    world_file = PathJoinSubstitution([pkg_spooder_gazebo, 'worlds', [world_name, '.sdf']])

    # 1. Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items(),
    )

    # 2. Bridge
    # CRITICAL: We bridge the clock so ROS time syncs with Sim time
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_spooder_gazebo, 'config', 'bridge_config.yaml'),
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='test_world'),
        gazebo,
        bridge,
    ])
