import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_spooder_navigation = get_package_share_directory('spooder_navigation')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_spooder_navigation, 'config', 'nav2_params.yaml'))
    
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='True')
    
    # Launch Nav2 Bringup (which launches Controller, Planner, Recoveries, BT Navigator, Lifecycle Mgr)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_spooder_navigation, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        
        nav2_launch
    ])
