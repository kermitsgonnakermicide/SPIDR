#!/usr/bin/env python3
"""
OAK-D Lite Unified Bringup Launch

This is the canonical entry point for running the hexapod with OAK-D Lite camera.
It handles both hardware and simulation modes, and optionally records ROS 2 bags.

Usage:
  # Simulation (default)
  ros2 launch spooder_perception oakd_bringup.launch.py mode:=simulation

  # Hardware
  ros2 launch spooder_perception oakd_bringup.launch.py mode:=hardware

  # Simulation with bag recording
  ros2 launch spooder_perception oakd_bringup.launch.py mode:=simulation record:=true

Arguments:
  mode        - 'simulation' | 'hardware' | 'simulation_only'
  record      - 'true' to auto-record bag (default: false)
  use_octomap - 'true' to run OctoMap terrain pipeline (default: true)
  world       - Gazebo world name (simulation only)
  spawn_x/y/z - Robot spawn position (simulation only)
"""

import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def get_bag_path():
    """Generate a timestamped bag directory path."""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = Path.home() / 'bags' / f'exp_{timestamp}'
    bag_dir.mkdir(parents=True, exist_ok=True)
    return str(bag_dir)


def generate_launch_description():
    pkg_spooder_perception = get_package_share_directory('spooder_perception')
    pkg_spooder_gazebo = get_package_share_directory('spooder_gazebo')
    pkg_spooder_navigation = get_package_share_directory('spooder_navigation')
    pkg_hexapod_nav = get_package_share_directory('hexapod_nav')

    # Arguments
    mode = LaunchConfiguration('mode')
    record = LaunchConfiguration('record')
    use_octomap = LaunchConfiguration('use_octomap')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    headless = LaunchConfiguration('headless')

    camera_params = os.path.join(pkg_spooder_perception, 'config', 'oakd_lite.yaml')
    pointcloud_params = os.path.join(pkg_spooder_perception, 'config', 'pointcloud_saver_params.yaml')

    bag_path = get_bag_path()

    # === Gazebo world ===
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_gazebo, 'launch', '01_sim_world.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'headless': headless,
        }.items(),
    )

    # === Robot spawn (includes rsp + spawn + controllers + EKF + gait) ===
    sim_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_gazebo, 'launch', '02_robot_spawn.launch.py')
        ),
        launch_arguments={
            'use_hexapod_nav': 'true',
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # === Static TF map -> odom (published immediately) ===
    # use_sim_time=True so the latched transform uses the simulated clock.
    # Without it, the stamp is wall-clock time, and tf2_buffer floods with
    # "Detected jump back in time" once the Gazebo /clock catches up.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'spooder/odom',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
    )

    # === Perception pipeline ===
    perception_nodes = [
        Node(
            package='spooder_perception',
            executable='pointcloud_optimizer',
            name='pointcloud_optimizer',
            output='screen',
            parameters=[pointcloud_params, {
                'input_topic': '/oak_d/points',
                'output_topic': '/oak_d/points/optimized',
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='spooder_perception',
            executable='terrain_analyzer',
            name='terrain_analyzer',
            output='screen',
            parameters=[pointcloud_params, {
                'input_topic': '/oak_d/points/optimized',
                'use_sim_time': use_sim_time,
            }],
        ),
    ]

    # === OctoMap pipeline ===
    octomap_nodes = [
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'map',
                'base_frame_id': 'spooder/base_footprint',
                'resolution': 0.05,
                'sensor_model.max_range': 5.0,
                # Filter the floor so OpenNav costmap does not treat the
                # immediately-below terrain as obstacle. Hexapod base height
                # ~15.4 cm above floor; an OAK-D-pointed-flat-surface ground
                # plane is detected and marked free (RAIB style).
                'filter_ground_plane': True,
                'ground_filter.distance': 0.04,  # 4 cm plane fit tolerance
                'ground_filter.angle': 0.15,      # ~9 deg from horizontal
                'ground_filter.plane_distance': 0.07,
                'ground_filter.frame_id': 'spooder/base_footprint',
                # Clip points below robot's belly (was -2.0; tightened so
                # noise below floor doesn't bleed in).
                'point_cloud_min_z': -0.5,
                'point_cloud_max_z': 5.0,
                # Only mark voxels at roughly chassis-clearance height as occupied.
                'occupancy_min_z': -0.05,
                'occupancy_max_z': 5.0,
                # Keep publish_free_space true so Nav2 planners can compute
                # traversable regions instead of seeing an obstacle cushion
                # around the robot.
                'publish_free_space': True,
                'latch': True,
            }],
            remappings=[('cloud_in', '/oak_d/points/optimized')],
        ),
        Node(
            package='spooder_perception',
            executable='octomap_terrain_adapter',
            name='octomap_terrain_adapter',
            output='screen',
            parameters=[pointcloud_params, {'use_sim_time': use_sim_time}],
        ),
    ]

    # === hexapod_nav terrain pipeline ===
    hexapod_nav_nodes = [
        Node(
            package='hexapod_nav',
            executable='octomap_terrain_node',
            name='octomap_terrain_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='hexapod_nav',
            executable='terrain_cost_node',
            name='terrain_cost_node',
            output='screen',
            parameters=[
                os.path.join(pkg_hexapod_nav, 'config', 'foothold_params.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='hexapod_nav',
            executable='foothold_planner_node',
            name='foothold_planner_node',
            output='screen',
            parameters=[
                os.path.join(pkg_hexapod_nav, 'config', 'foothold_params.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='hexapod_nav',
            executable='gait_controller_node',
            name='gait_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='hexapod_nav',
            executable='robot_state_viz',
            name='robot_state_viz',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    # === Nav2 ===
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_hexapod_nav, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    # === RViz ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_hexapod_nav, 'rviz', 'sim.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # === Hardware-only: OAK-D driver ===
    hardware_driver = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak_d',
        output='screen',
        parameters=[camera_params],
    )

    # === Bag recording ===
    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '-s', 'mcap',
            '/oak_d/points',
            '/tf',
            '/tf_static',
            '/odom',
            '/projected_map',
            '/cmd_vel',
            '/joint_states',
            '/terrain_cost_map',
            '/foothold_plan',
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('mode', default_value='simulation',
                             description='Launch mode: simulation | hardware | simulation_only'),
        DeclareLaunchArgument('record', default_value='false',
                             description='Enable ROS 2 bag recording'),
        DeclareLaunchArgument('use_octomap', default_value='true',
                             description='Run OctoMap terrain pipeline'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='plain_world'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.6'),
        # Spawn at z=0.5 instead of 3.0 so the robot lands gently and settles
        # into the resting leg pose without 3 m of free-fall crash dynamics.
        DeclareLaunchArgument('spawn_z', default_value='0.5'),
        DeclareLaunchArgument('headless', default_value='false',
                             description='Run Gazebo headless (no GUI). Wired to 01_sim_world.launch.py.'),

        # Log mode
        LogInfo(msg=['OAK-D Lite bringup: mode=', mode]),

        # === SIMULATION MODE ===
        GroupAction(
            actions=[
                # Phase 1: Gazebo world
                TimerAction(period=0.0, actions=[sim_gazebo]),

                # Phase 2: Robot spawn (spawns at t=8s, ekf at t=18s)
                TimerAction(period=2.0, actions=[sim_robot_spawn]),

                # Phase 3: Static TF (map->odom)
                TimerAction(period=4.0, actions=[static_tf]),

                # Phase 4: Perception pipeline (after robot spawn is up)
                TimerAction(period=12.0, actions=perception_nodes),

                # Phase 5: OctoMap (needs perception running)
                TimerAction(period=14.0, actions=octomap_nodes),

                # Phase 6: hexapod_nav terrain pipeline
                TimerAction(period=16.0, actions=hexapod_nav_nodes),

                # Phase 7: Nav2 (needs EKF TF up from spawn at t=18s)
                TimerAction(period=22.0, actions=[nav2_launch]),

                # Phase 8: RViz (last, foreground)
                TimerAction(period=24.0, actions=[rviz_node]),
            ],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'simulation'"])),
        ),

        # === HARDWARE MODE ===
        GroupAction(
            actions=[
                TimerAction(period=0.0, actions=[hardware_driver]),
                TimerAction(period=2.0, actions=perception_nodes),
            ],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'hardware'"])),
        ),

        # === BAG RECORDING ===
        TimerAction(period=30.0, actions=[record_process],
                   condition=IfCondition(PythonExpression(["'", record, "' == 'true'"]))),
    ])
