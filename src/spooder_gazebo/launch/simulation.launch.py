"""
MINIMAL WORKING SIMULATION LAUNCH FILE
This is stripped down to the bare essentials to ensure it works.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.parameter_descriptions


def generate_launch_description():
    # Package directories
    pkg_spooder_gazebo = get_package_share_directory('spooder_gazebo')
    pkg_spooder_description = get_package_share_directory('spooder_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spooder_control = get_package_share_directory('spooder_control')
    pkg_spooder_navigation = get_package_share_directory('spooder_navigation')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='test_world')

    # World file path
    world_file = PathJoinSubstitution([pkg_spooder_gazebo, 'worlds', [world_name, '.sdf']])

    # Robot description from xacro
    xacro_file = os.path.join(pkg_spooder_description, 'urdf', 'spooder.xacro')
    config_file = os.path.join(pkg_spooder_control, 'config', 'ros2_control.yaml')
    robot_description = Command(['xacro ', xacro_file, ' config_file:=', config_file])

    # ========== NODES ==========

    # 1. GAZEBO SIMULATOR
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file]
        }.items(),
    )

    # 2. ROS-GAZEBO BRIDGE (Provide clock, sensors, commands)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_spooder_gazebo, 'config', 'bridge_config.yaml'),
        }],
        output='screen'
    )

    # 3. ROBOT STATE PUBLISHER (Provide robot TF tree)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.parameter_descriptions.ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # 4. SPAWN ROBOT INTO GAZEBO
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'spooder',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # 5. JOINT STATE BROADCASTER
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        output='screen',
    )

    # 6. ROBOT CONTROLLER
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['spooder_controller', '--controller-manager-timeout', '120'],
        output='screen',
    )

    # 7. EKF LOCALIZATION
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_spooder_control, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('odometry/filtered', 'odometry/filtered')]
    )

    # 8. SLAM TOOLBOX
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_navigation, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 9. NAV2 NAVIGATION
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 10. GAIT CONTROLLER
    gait_controller = Node(
        package='spooder_control',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 11. RVIZ2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_spooder_navigation, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== LAUNCH SEQUENCE ==========
    # Use TimerActions to ensure proper startup order
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='test_world'),
        
        # Phase 1: Robot Description & TF (Immediate)
        robot_state_publisher,
        
        # Phase 2: Bridge (Connects topics)
        bridge,

        # Phase 3: Gazebo (The heavy sim)
        TimerAction(period=2.0, actions=[gazebo]),
        
        # Phase 4: Spawn Robot (t=8s - wait for Gazebo)
        TimerAction(period=8.0, actions=[spawn_entity]),
        
        # Phase 5: Controllers (t=15s)
        TimerAction(period=15.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=18.0, actions=[robot_controller_spawner]),
        
        # Phase 6: Localization & SLAM (t=20s)
        TimerAction(period=20.0, actions=[ekf_node]),
        TimerAction(period=25.0, actions=[slam_toolbox]),
        
        # Phase 7: Navigation (t=30s)
        TimerAction(period=30.0, actions=[navigation]),
        
        # Phase 8: Application & Viz (t=5s - show RSP immediately)
        TimerAction(period=35.0, actions=[gait_controller]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
