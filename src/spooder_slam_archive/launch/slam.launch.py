from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    world_name = LaunchConfiguration('world')
    
    pkg_spooder_slam = get_package_share_directory('spooder_slam')
    pkg_spooder_gazebo = get_package_share_directory('spooder_gazebo')
    
    rtabmap_launch_path = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
    rviz_config_path = os.path.join(pkg_spooder_slam, 'config', 'nav2_default.rviz')
    
    # Launch Simulation
    simulation_launch_path = os.path.join(pkg_spooder_gazebo, 'launch', 'simulation.launch.py')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='maze_world', description='World name'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map GUI'),
        
        # 1. Include Simulation (Gazebo, Robot Spawning, Bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world_name
            }.items()
        ),

        # 2. Staged Startup: Wait for simulation to stabilize
        TimerAction(
            period=10.0,
            actions=[
                # Gait Controller Node
                Node(
                    package='spooder_control',
                    executable='gait_controller_node',
                    name='gait_controller',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
                
                # RTAB-Map 3D Visual SLAM - DUAL SENSOR CONFIG
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtabmap_launch_path),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'stereo': 'true',
                        'frame_id': 'base_footprint',
                        'left_image_topic': '/camera/left/image_raw',
                        'right_image_topic': '/camera/right/image_raw',
                        'left_camera_info_topic': '/camera/left/camera_info',
                        'right_camera_info_topic': '/camera/right/camera_info',
                        'rtabmap_viz': rtabmap_viz,
                        'args': '--delete_db_on_start',
                        'approx_sync': 'true',
                        'approx_sync_max_interval': '0.1',
                        'queue_size': '30',
                        'subscribe_depth': 'false',
                        'stereo_to_depth': 'true',
                        'subscribe_scan': 'true',
                        'scan_topic': '/scan',
                        'publish_tf_odom': 'false', # Simulation publishes odom -> base_footprint
                        'odom_topic': '/odom',
                        'visual_odometry': 'false',
                        'icp_odometry': 'false', # We use sim odom now
                        'wait_for_transform': '1.0',
                        'rtabmap_args': (
                            '--delete_db_on_start '
                            '--Vis/MinInliers 10 '
                            '--Vis/MaxFeatures 800 '
                            '--RGBD/CreateOccupancyGrid true '
                            '--Reg/Strategy 1 '         # 1=ICP for robust maze mapping
                            '--Reg/Force3DoF true '     # For mostly planar hexapod movement
                            '--RGBD/NeighborLinkRefining true ' 
                            '--Grid/Sensor 0 '          # 0=Laser
                            '--Grid/RangeMax 10.0'
                        )
                    }.items()
                ),

                # NAV2 Stack
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': os.path.join(pkg_spooder_slam, 'config', 'nav2_params.yaml'),
                        'autostart': 'true',
                        'map_subscribe_transient_local': 'true'
                    }.items()
                ),

                # RViz2 Node
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_path],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                ),
                
            ]
        )
    ])
