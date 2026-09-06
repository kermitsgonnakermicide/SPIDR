"""
OAK-D Lite simulation launch file.

Launches the spooder_perception nodes for processing the Gazebo simulated
OAK-D Lite camera output. The Gazebo plugin in spooder_description publishes
/oak_d/points (PointCloud2) and the perception nodes optimize and forward.

This is for simulation only — for hardware, use oakd_driver.launch.py.

Usage:
  ros2 launch spooder_perception oakd_sim.launch.py
  ros2 launch spooder_perception oakd_sim.launch.py use_sim_time:=true

The Gazebo depth camera (in spooder_description) publishes:
  /oak_d/points        - PointCloud2
  /oak_d/rgb/image    - RGB image
  /oak_d/stereo/image - Depth image
  /oak_d/camera_info  - Camera info

This launch file runs the perception pipeline (pointcloud_optimizer, octomap_server,
octomap_terrain_adapter) on that input.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_spooder_perception = get_package_share_directory('spooder_perception')

    # Default config from spooder_perception
    pointcloud_params = os.path.join(
        pkg_spooder_perception, 'config', 'pointcloud_saver_params.yaml'
    )
    sim_params = os.path.join(
        pkg_spooder_perception, 'config', 'oakd_sim.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/oak_d/points')
    optimized_cloud_topic = LaunchConfiguration(
        'optimized_cloud_topic', default='/oak_d/points/optimized'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/oak_d/points'),
        DeclareLaunchArgument(
            'optimized_cloud_topic',
            default_value='/oak_d/points/optimized',
        ),

        # Point cloud optimizer
        Node(
            package='spooder_perception',
            executable='pointcloud_optimizer',
            name='pointcloud_optimizer',
            output='screen',
            parameters=[pointcloud_params, sim_params, {
                'use_sim_time': use_sim_time,
                'input_topic': cloud_topic,
                'output_topic': optimized_cloud_topic,
            }],
        ),

        # OctoMap server
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
                'sensor_model.max_range': 4.0,
                'filter_ground_plane': True,
                'point_cloud_min_z': -0.15,
                'point_cloud_max_z': 1.2,
                'occupancy_min_z': 0.04,
                'occupancy_max_z': 0.7,
                'publish_free_space': False,
                'latch': True,
            }],
            remappings=[('cloud_in', optimized_cloud_topic)],
        ),

        # Terrain adapter
        Node(
            package='spooder_perception',
            executable='octomap_terrain_adapter',
            name='octomap_terrain_adapter',
            output='screen',
            parameters=[pointcloud_params, {'use_sim_time': use_sim_time}],
        ),
    ])
