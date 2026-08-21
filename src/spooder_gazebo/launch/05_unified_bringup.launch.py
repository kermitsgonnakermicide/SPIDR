import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def require_ros_package(package_name, apt_package):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Missing ROS package '{package_name}'. Install it with: "
            f"sudo apt-get install -y {apt_package}"
        ) from exc


def generate_launch_description():
    require_ros_package('gz_ros2_control', 'ros-jazzy-gz-ros2-control')

    pkg_spooder_description = get_package_share_directory('spooder_description')
    pkg_spooder_navigation = get_package_share_directory('spooder_navigation')
    pkg_spooder_control = get_package_share_directory('spooder_control')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file = os.path.join(pkg_spooder_description, 'urdf', 'spooder.xacro')

    controller_config_file = os.path.join(pkg_spooder_control, 'config', 'ros2_control.yaml')
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file, ' config_file:=', controller_config_file]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True,
            'frame_prefix': 'spooder/'
        }]
    )

    # No use_sim_time on create — avoids hung world-name discovery.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spooder', '-z', '3.0'],
        additional_env={'GZ_IP': '127.0.0.1'},
        output='screen'
    )

    load_spooder_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'spooder_controller', '-c', 'controller_manager'],
        output='screen'
    )

    gait_controller = Node(
        package='spooder_control',
        executable='gait_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_navigation, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    nav2_params = os.path.join(pkg_spooder_navigation, 'config', 'nav2_params.yaml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
        }]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        robot_state_publisher,

        # Spawn after Gazebo (world started separately / already up)
        TimerAction(period=8.0, actions=[spawn_entity]),
        TimerAction(period=14.0, actions=[load_spooder_controller]),
        TimerAction(period=16.0, actions=[gait_controller]),
        TimerAction(period=20.0, actions=[slam_toolbox]),
        TimerAction(period=25.0, actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager
        ]),
    ])
