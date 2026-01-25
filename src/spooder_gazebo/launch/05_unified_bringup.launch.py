import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Setup paths
    pkg_spooder_description = get_package_share_directory('spooder_description')
    pkg_spooder_gazebo = get_package_share_directory('spooder_gazebo')
    pkg_spooder_navigation = get_package_share_directory('spooder_navigation')
    pkg_spooder_control = get_package_share_directory('spooder_control')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file = os.path.join(pkg_spooder_description, 'urdf', 'spooder.xacro')

    # 2. Robot State Publisher
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

    # 3. Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spooder', '-z', '0.2'],
        output='screen'
    )

    # 4. Controllers (Delayed)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen'
    )

    load_spooder_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'spooder_controller', '-c', 'controller_manager'],
        output='screen'
    )

    gait_controller = Node(
        package='spooder_control',
        executable='gait_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. SLAM Toolbox (Layer 2)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_spooder_navigation, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 6. Nav2 Stack (Layer 3)
    # Re-using the logic from 03_custom_nav but inside the master launch
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
            'node_names': ['slam_toolbox', 'planner_server', 'controller_server', 'bt_navigator']
        }]
    )

    # 7. Orchestration using TimerActions
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Layer 0: Basic TF
        robot_state_publisher,
        spawn_entity,

        # Layer 1: Controllers (After Spawn)
        TimerAction(period=5.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=7.0, actions=[load_spooder_controller]),
        TimerAction(period=9.0, actions=[gait_controller]),

        # Layer 2: SLAM (After Controllers are stable)
        TimerAction(period=12.0, actions=[slam_toolbox]),

        # Layer 3: Nav2 (After SLAM has a map)
        TimerAction(period=15.0, actions=[
            controller_server,
            planner_server,
            bt_navigator,
            lifecycle_manager
        ]),
    ])
