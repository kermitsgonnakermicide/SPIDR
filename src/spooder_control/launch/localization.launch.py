import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('spooder_control')
    
    # Path to ekf.yaml
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true', 
            description='Use simulation time'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path, 
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[('odometry/filtered', 'odometry/filtered')] 
            # Note: The tutorial text says it publishes "odometry/filtered".
            # We might want to remap this to /odom if we wanted to replace the sim odom entirely,
            # but usually /odom is the source and /odometry/filtered is the output.
            # Nav2 usually expects /odom. 
            # However, if we follow the tutorial, we publish /odometry/filtered and /tf.
            # We should probably remap odometry/filtered to /odom if we want to be drop-in compliant 
            # OR configure nav2 to listen to /odometry/filtered.
            # For now, let's keep it standard: /odometry/filtered works if TF is good.
        )
    ])
