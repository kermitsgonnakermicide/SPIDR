#!/usr/bin/env python3
"""
Perception launch file for SPIDR.

Launches the depth-cloud optimizer, OctoMap, and terrain adapter.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    perception_dir = get_package_share_directory('spooder_perception')

    # Config files
    pointcloud_params = os.path.join(
        perception_dir,
        'config',
        'pointcloud_saver_params.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/camera/points')
    optimized_cloud_topic = LaunchConfiguration(
        'optimized_cloud_topic',
        default='/camera/points/optimized',
    )

    pointcloud_optimizer_node = Node(
        package='spooder_perception',
        executable='pointcloud_optimizer',
        name='pointcloud_optimizer',
        output='screen',
        parameters=[pointcloud_params, {
            'use_sim_time': use_sim_time,
            'input_topic': cloud_topic,
            'output_topic': optimized_cloud_topic,
        }],
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        namespace='octomap_server',
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
    )

    terrain_adapter_node = Node(
        package='spooder_perception',
        executable='octomap_terrain_adapter',
        name='octomap_terrain_adapter',
        output='screen',
        parameters=[pointcloud_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/points'),
        DeclareLaunchArgument(
            'optimized_cloud_topic',
            default_value='/camera/points/optimized',
        ),
        pointcloud_optimizer_node,
        octomap_server_node,
        terrain_adapter_node,
    ])
