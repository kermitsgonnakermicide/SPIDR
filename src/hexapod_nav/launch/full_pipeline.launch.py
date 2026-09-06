from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('hexapod_nav')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        # --- OAK-D Camera (delegated to spooder_perception) ---
        # Note: The OAK-D driver is now owned by spooder_perception.
        # Use oakd_driver.launch.py for hardware or oakd_sim.launch.py for sim.
        # This launch file assumes the camera is already running.
        # See spooder_perception/launch/oakd_bringup.launch.py for unified bringup.

        # --- OctoMap Server ---
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[os.path.join(pkg, 'config', 'octomap_params.yaml')],
            remappings=[
                ('cloud_in', '/oak_d/points'),
            ]
        ),

        # --- Terrain Extraction ---
        Node(
            package='hexapod_nav',
            executable='octomap_terrain_node',
            name='octomap_terrain_node',
        ),

        # --- Cost Computation ---
        Node(
            package='hexapod_nav',
            executable='terrain_cost_node',
            name='terrain_cost_node',
            parameters=[os.path.join(pkg, 'config', 'foothold_params.yaml')],
        ),

        # --- Foothold Planner ---
        Node(
            package='hexapod_nav',
            executable='foothold_planner_node',
            name='foothold_planner_node',
            parameters=[os.path.join(pkg, 'config', 'foothold_params.yaml')],
        ),

        # --- Gait Controller ---
        Node(
            package='hexapod_nav',
            executable='gait_controller_node',
            name='gait_controller_node',
        ),

        # --- Nav2 (body path only) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])