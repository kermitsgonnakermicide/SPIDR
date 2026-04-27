import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions

def generate_launch_description():
    diddler_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- 1. Gazebo World ---
    world_file = os.path.join(diddler_dir, 'rough_terrain.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # --- 2. Clock Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # --- 3. Robot Description ---
    xacro_file = os.path.join(diddler_dir, 'sim.xacro')
    robot_description_content = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.parameter_descriptions.ParameterValue(
                robot_description_content, value_type=str
            ),
            'use_sim_time': use_sim_time,
        }]
    )

    # --- 4. Spawn Robot on the terrain ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diddler',
            '-x', '0', '-y', '0', '-z', '0.2',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Launch Gazebo with rough terrain
        gazebo,
        bridge,

        # Robot
        robot_state_publisher,
        
        # Spawn after Gazebo is up
        TimerAction(period=5.0, actions=[spawn_entity]),
    ])
