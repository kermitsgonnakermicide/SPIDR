from setuptools import setup
import os
from glob import glob

package_name = 'spooder_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    scripts=[
        'scripts/gait_controller_node',
        'scripts/simple_explorer_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daksh',
    maintainer_email='daksh.vohra1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_controller = spooder_control.gait_controller:main',
            'teleop = spooder_control.teleop:main',
            'simple_explorer = spooder_control.simple_explorer:main',
        ],
    },
)
