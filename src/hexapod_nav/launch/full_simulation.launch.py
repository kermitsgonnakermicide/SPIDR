"""
Full simulation launch — hexapod_nav + Gazebo.

Self-contained launch that starts everything:
  1. Gazebo Sim with world file
  2. ros_gz_bridge (dynamic clock config)
  3. Robot State Publisher + spawn into Gazebo
  4. Joint controllers + EKF localization
  5. SLAM Toolbox (map -> odom TF)
  6. OctoMap server (must start BEFORE Nav2 so static_layer has data)
  7. hexapod_nav terrain pipeline (OctoMap -> terrain -> cost -> planner -> gait)
  8. Nav2 with hexapod_nav's OctoMap-projected costmaps
  9. RViz with hexapod_nav visualization

Usage:
  ros2 launch hexapod_nav full_simulation.launch.py
  ros2 launch hexapod_nav full_simulation.launch.py world:=foothold_terrain spawn_x:=1.0
  ros2 launch hexapod_nav full_simulation.launch.py spawn_z:=3.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -- Package directories ------------------------------------------------
    pkg_gazebo   = get_package_share_directory('spooder_gazebo')
    pkg_nav      = get_package_share_directory('spooder_navigation')
    pkg_hex      = get_package_share_directory('hexapod_nav')

    # -- Launch arguments ---------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world        = LaunchConfiguration('world',        default='test_world')
    headless     = LaunchConfiguration('headless',     default='false')
    spawn_x      = LaunchConfiguration('spawn_x',      default='0.0')
    spawn_y      = LaunchConfiguration('spawn_y',      default='0.0')
    spawn_z      = LaunchConfiguration('spawn_z',      default='3.0')
    spawn_yaw    = LaunchConfiguration('spawn_yaw',    default='0.0')

    # -- 1. Gazebo + Bridge ------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', '01_sim_world.launch.py')
        ),
        launch_arguments={
            'world':        world,
            'use_sim_time': use_sim_time,
            'headless':     headless,
        }.items(),
    )

    # -- 2. Robot Spawn + Controllers + EKF --------------------------------
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', '02_robot_spawn.launch.py')
        ),
        launch_arguments={
            'use_hexapod_nav': 'true',
            'spawn_x':         spawn_x,
            'spawn_y':         spawn_y,
            'spawn_z':         spawn_z,
            'spawn_yaw':       spawn_yaw,
            'use_sim_time':    use_sim_time,
        }.items(),
    )

    # -- 3. SLAM Toolbox ---------------------------------------------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # -- 4. OctoMap Server (MUST start before Nav2) ------------------------
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[
            os.path.join(pkg_hex, 'config', 'octomap_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cloud_in', '/camera/points')],
        output='screen',
    )

    # -- 5. Terrain Extraction ----------------------------------------------
    terrain_extract = Node(
        package='hexapod_nav',
        executable='octomap_terrain_node',
        name='octomap_terrain_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # -- 6. Cost Computation ------------------------------------------------
    terrain_cost = Node(
        package='hexapod_nav',
        executable='terrain_cost_node',
        name='terrain_cost_node',
        parameters=[
            os.path.join(pkg_hex, 'config', 'foothold_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # -- 7. Foothold Planner ------------------------------------------------
    foothold_planner = Node(
        package='hexapod_nav',
        executable='foothold_planner_node',
        name='foothold_planner_node',
        parameters=[
            os.path.join(pkg_hex, 'config', 'foothold_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # -- 8. Gait Controller -------------------------------------------------
    gait_controller = Node(
        package='hexapod_nav',
        executable='gait_controller_node',
        name='gait_controller_node',
        parameters=[
            os.path.join(pkg_hex, 'config', 'foothold_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # -- 9. Nav2 (uses hexapod_nav's OctoMap-projected costmaps) -----------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  os.path.join(pkg_hex, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    # -- 10. RViz -----------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_hex, 'rviz', 'sim.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ======================================================================
    #  LAUNCH SEQUENCE
    # ======================================================================
    #
    #  (times from 02_robot_spawn.launch.py, included at t=0)
    #  t= 0s   Gazebo + bridge + RSP
    #  t= 3s   RViz
    #  t= 8s   Spawn robot (ros_gz_sim create, no use_sim_time)
    #  t=14s   joint_state_broadcaster
    #  t=16s   spooder_controller
    #  t=18s   EKF
    #  t=22s   SLAM Toolbox (map -> odom TF)
    #  t=24s   OctoMap server (must precede Nav2)
    #  t=26s   Terrain pipeline
    #  t=32s   Nav2 (static_layer on /projected_map)
    #
    return LaunchDescription([
        # -- Arguments -------------------------------------------------------
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world',        default_value='test_world'),
        DeclareLaunchArgument('headless',     default_value='false'),
        DeclareLaunchArgument('spawn_x',      default_value='0.0'),
        DeclareLaunchArgument('spawn_y',      default_value='0.0'),
        DeclareLaunchArgument('spawn_z',      default_value='3.0'),
        DeclareLaunchArgument('spawn_yaw',    default_value='0.0'),

        # -- Phase 1: Infrastructure (immediate) ----------------------------
        gazebo,
        robot_spawn,

        # -- Phase 2: Visualization (t=3s) ----------------------------------
        TimerAction(period=3.0, actions=[rviz]),

        # -- Phase 3: SLAM (after EKF at t=18s) -----------------------------
        TimerAction(period=22.0, actions=[slam]),

        # -- Phase 4: OctoMap (needs TF; must precede Nav2) -----------------
        TimerAction(period=24.0, actions=[octomap_server]),

        # -- Phase 5: Terrain pipeline --------------------------------------
        TimerAction(period=26.0, actions=[terrain_extract]),
        TimerAction(period=27.0, actions=[terrain_cost]),
        TimerAction(period=28.0, actions=[foothold_planner]),
        TimerAction(period=29.0, actions=[gait_controller]),

        # -- Phase 6: Nav2 --------------------------------------------------
        TimerAction(period=32.0, actions=[nav2]),
    ])
