from setuptools import setup

package_name = 'hexapod_nav'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/octomap_params.yaml',
                                               'config/nav2_params.yaml',
                                               'config/foothold_params.yaml',
                                               'config/robot_description.yaml']),
        ('share/' + package_name + '/launch', ['launch/full_pipeline.launch.py',
                                               'launch/simulation.launch.py',
                                               'launch/full_simulation.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/sim.rviz']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'octomap_terrain_node = hexapod_nav.octomap_terrain_node:main',
            'terrain_cost_node    = hexapod_nav.terrain_cost_node:main',
            'foothold_planner_node = hexapod_nav.foothold_planner_node:main',
            'gait_controller_node  = hexapod_nav.gait_controller_node:main',
            'rerun_bridge          = hexapod_nav.rerun_bridge:main',
            'robot_state_viz       = hexapod_nav.robot_state_viz:main',
        ],
    },
)
