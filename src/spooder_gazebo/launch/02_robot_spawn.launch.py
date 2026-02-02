import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions

def generate_launch_description():
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
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.1')
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
    # These depend on the Gazebo plugin loading the model first. 
    # We add a delay to be safe, but they will retry if not ready.
    joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['spooder_controller'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 4. Gait Controller (The Python script)
    # Added try-catch in script, safe to launch
    gait_controller = Node(
        package='spooder_control',
        executable='gait_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. EKF (Localization)
    # Needs Odom from bridge + IMU
    ekf = Node(
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Immediate TF
        robot_state_publisher,
        
        # Spawn
        spawn_entity,
        
        # Wait for spawn -> load controllers
        TimerAction(period=5.0, actions=[joint_broadcaster]),
        TimerAction(period=7.0, actions=[controller]),
        
        # Start Logic
        TimerAction(period=12.0, actions=[ekf]),
        TimerAction(period=15.0, actions=[gait_controller]),
    ])
