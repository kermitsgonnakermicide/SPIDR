"""
Simulation launch for hexapod_nav.

Assumes the base Gazebo simulation (world, robot spawn, controllers, EKF, SLAM)
is already running from start_spooder.sh. This launches only the hexapod_nav
OctoMap + terrain pipeline on top.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hexapod_nav = get_package_share_directory('hexapod_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # --- OctoMap Server (subscribes to /camera/points from Gazebo depth camera) ---
        TimerAction(period=2.0, actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                parameters=[
                    os.path.join(pkg_hexapod_nav, 'config', 'octomap_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('cloud_in', '/camera/points'),
                ],
                output='screen'
            )
        ]),

        # --- Terrain Extraction (floor/ceiling from OctoMap voxels) ---
        TimerAction(period=4.0, actions=[
            Node(
                package='hexapod_nav',
                executable='octomap_terrain_node',
                name='octomap_terrain_node',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]),

        # --- Cost Computation (slope + roughness + clearance) ---
        TimerAction(period=5.0, actions=[
            Node(
                package='hexapod_nav',
                executable='terrain_cost_node',
                name='terrain_cost_node',
                parameters=[
                    os.path.join(pkg_hexapod_nav, 'config', 'foothold_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]),

        # --- Foothold Planner (IK reachability + cost query) ---
        TimerAction(period=6.0, actions=[
            Node(
                package='hexapod_nav',
                executable='foothold_planner_node',
                name='foothold_planner_node',
                parameters=[
                    os.path.join(pkg_hexapod_nav, 'config', 'foothold_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]),

        # --- Gait Controller (replaces spooder_control's gait_controller) ---
        TimerAction(period=7.0, actions=[
            Node(
                package='hexapod_nav',
                executable='gait_controller_node',
                name='gait_controller_node',
                parameters=[
                    os.path.join(pkg_hexapod_nav, 'config', 'foothold_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]),

        # --- Robot State Viz (heading arrow, tilt disc, elevation text) ---
        TimerAction(period=8.0, actions=[
            Node(
                package='hexapod_nav',
                executable='robot_state_viz',
                name='robot_state_viz',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]),
    ])
