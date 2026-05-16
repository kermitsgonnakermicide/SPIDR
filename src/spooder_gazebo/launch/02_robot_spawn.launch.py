import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions


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
    pkg_spooder_control = get_package_share_directory('spooder_control')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot Description
    xacro_file = os.path.join(pkg_spooder_description, 'urdf', 'spooder.xacro')
    config_file = os.path.join(pkg_spooder_control, 'config', 'ros2_control.yaml')
    robot_description_content = Command(['xacro ', xacro_file, ' config_file:=', config_file])

    # 1. Robot State Publisher (TF Tree Source)
    # Wraps description in ParameterValue to avoid parser errors
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.parameter_descriptions.ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time,
            'frame_prefix': 'spooder/'
        }]
    )


    # Spawn position parameters
    spawn_x = LaunchConfiguration('spawn_x', default='1.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.2')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')

    # 2. Spawn Entity (Injects model into Gazebo)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spooder', 
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


    # 3. Controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'spooder_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 4. Gait Controller
    gait_controller = Node(
        package='spooder_control',
        executable='gait_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. EKF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_spooder_control, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='1.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.2'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        
        robot_state_publisher,
        spawn_entity,
        
        # Sequence
        TimerAction(period=2.0, actions=[joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[controller]),
        TimerAction(period=6.0, actions=[ekf]),
        TimerAction(period=8.0, actions=[gait_controller]),
    ])
