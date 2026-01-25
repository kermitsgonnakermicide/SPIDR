from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spooder_control',
            executable='gait_controller',
            name='gait_controller',
            output='screen'
        )
    ])
