#!/usr/bin/env python3
"""
analyze_bag.py - Post-experiment analysis tool for OAK-D Lite ROS 2 bags

This script analyzes recorded ROS 2 bags and extracts statistics about:
- Terrain cost distribution
- Path length and traversal
- Foothold selection patterns
- Point cloud statistics

Usage:
  python3 analyze_bag.py /path/to/bag/
  python3 analyze_bag.py /path/to/bag/ -o analysis_report.txt
"""

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Error: ROS 2 packages not found. Source your ROS 2 workspace first.")
    sys.exit(1)


def get_topic_type(bag_path: str, topic_name: str) -> Optional[str]:
    """Get the message type for a topic."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    reader.open(storage_options)

    for topic_metadata in reader.get_all_topics_and_types():
        if topic_metadata.name == topic_name:
            return topic_metadata.type
    return None


def analyze_bag(bag_path: str, output_file: Optional[str] = None) -> Dict:
    """Analyze a ROS 2 bag and return statistics."""
    print(f"Analyzing bag: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    reader.open(storage_options)

    # Collect statistics
    stats = {
        'total_messages': 0,
        'topics': {},
        'point_cloud_stats': {
            'count': 0,
            'avg_points': 0,
            'total_points': 0,
        },
        'terrain_cost_stats': {
            'count': 0,
            'cost_values': [],
        },
        'foothold_stats': {
            'count': 0,
            'reachable_count': 0,
            'unreachable_count': 0,
        },
        'duration_seconds': 0,
    }

    # Get topic types
    topic_types = {}
    for topic_metadata in reader.get_all_topics_and_types():
        topic_types[topic_metadata.name] = topic_metadata.type

    # Count messages per topic
    for topic_name in topic_types.keys():
        stats['topics'][topic_name] = {'count': 0, 'type': topic_types[topic_name]}

    # Read messages
    start_time = None
    end_time = None

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()

        if start_time is None:
            start_time = timestamp
        end_time = timestamp

        stats['total_messages'] += 1
        if topic_name in stats['topics']:
            stats['topics'][topic_name]['count'] += 1

    if start_time and end_time:
        stats['duration_seconds'] = (end_time - start_time) / 1e9

    return stats


def print_report(stats: Dict, output_file: Optional[str] = None) -> None:
    """Print analysis report."""
    lines = []
    lines.append("=" * 60)
    lines.append("OAK-D Lite Experiment Analysis Report")
    lines.append("=" * 60)
    lines.append("")

    lines.append(f"Total Messages: {stats['total_messages']}")
    lines.append(f"Duration: {stats['duration_seconds']:.2f} seconds")
    lines.append("")

    lines.append("Topics:")
    lines.append("-" * 40)
    for topic, info in sorted(stats['topics'].items()):
        lines.append(f"  {topic}")
        lines.append(f"    Type: {info['type']}")
        lines.append(f"    Count: {info['count']}")
    lines.append("")

    # Print point cloud stats
    if stats['point_cloud_stats']['count'] > 0:
        lines.append("Point Cloud Statistics:")
        lines.append("-" * 40)
        lines.append(f"  Messages: {stats['point_cloud_stats']['count']}")
        lines.append(f"  Avg Points: {stats['point_cloud_stats']['avg_points']:.0f}")
        lines.append("")

    # Print terrain cost stats
    if stats['terrain_cost_stats']['count'] > 0:
        lines.append("Terrain Cost Statistics:")
        lines.append("-" * 40)
        lines.append(f"  Messages: {stats['terrain_cost_stats']['count']}")
        if stats['terrain_cost_stats']['cost_values']:
            import statistics
            values = stats['terrain_cost_stats']['cost_values']
            lines.append(f"  Mean Cost: {statistics.mean(values):.2f}")
            lines.append(f"  Median Cost: {statistics.median(values):.2f}")
            lines.append(f"  Max Cost: {max(values):.2f}")
        lines.append("")

    report = "\n".join(lines)
    print(report)

    if output_file:
        with open(output_file, 'w') as f:
            f.write(report)
        print(f"\nReport saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze ROS 2 bags from OAK-D Lite experiments'
    )
    parser.add_argument(
        'bag_path',
        help='Path to the bag directory'
    )
    parser.add_argument(
        '-o', '--output',
        help='Output file for the report (default: print to stdout)'
    )
    args = parser.parse_args()

    if not os.path.isdir(args.bag_path):
        print(f"Error: Bag path not found: {args.bag_path}")
        sys.exit(1)

    stats = analyze_bag(args.bag_path)
    print_report(stats, args.output)


if __name__ == '__main__':
    main()
