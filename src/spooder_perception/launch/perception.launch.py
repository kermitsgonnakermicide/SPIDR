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
    pointcloud_saver_params = os.path.join(
        perception_dir,
        'config',
        'pointcloud_saver_params.yaml'
    )
    
    # Point Cloud Saver Node
    pointcloud_saver_node = Node(
        package='spooder_perception',
        executable='pointcloud_saver',
        name='pointcloud_saver',
        output='screen',
        parameters=[pointcloud_saver_params],
    )
    
    # Terrain Analyzer Node
    terrain_analyzer_node = Node(
        package='spooder_perception',
        executable='terrain_analyzer',
        name='terrain_analyzer',
        output='screen',
    )
    
    return LaunchDescription([
        pointcloud_saver_node,
        terrain_analyzer_node,
    ])
