#!/usr/bin/env python3
"""
Perception launch file for SPIDR
Launches point cloud saver and other perception nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    perception_dir = get_package_share_directory('spooder_perception')
    
    # Config files
    pointcloud_params = os.path.join(
        perception_dir,
        'config',
        'pointcloud_saver_params.yaml'
    )

    # Change-aware live stream for Nav2/terrain analysis. The saver below still
    # consumes raw camera points so the persistent map keeps accumulating.
    pointcloud_optimizer_node = Node(
        package='spooder_perception',
        executable='pointcloud_optimizer',
        name='pointcloud_optimizer',
        output='screen',
        parameters=[pointcloud_params],
    )
    
    # Point Cloud Saver Node
    pointcloud_saver_node = Node(
        package='spooder_perception',
        executable='pointcloud_saver',
        name='pointcloud_saver',
        output='screen',
        parameters=[pointcloud_params],
    )
    
    # Terrain Analyzer Node
    terrain_analyzer_node = Node(
        package='spooder_perception',
        executable='terrain_analyzer',
        name='terrain_analyzer',
        output='screen',
        parameters=[pointcloud_params],
    )
    
    return LaunchDescription([
        pointcloud_optimizer_node,
        pointcloud_saver_node,
        terrain_analyzer_node,
    ])
