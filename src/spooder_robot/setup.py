from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'spooder_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daksh',
    maintainer_email='daksh.vohra1@gmail.com',
    description='Onboard robot node: ST3215 + OAK-D Lite coordinator, IMU republisher, gait controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = spooder_robot.robot_node:main',
            'imu_node = spooder_robot.imu_node:main',
            'gait_node = spooder_robot.gait_node:main',
        ],
    },
)
