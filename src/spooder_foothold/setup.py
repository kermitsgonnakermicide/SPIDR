from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spooder_foothold'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Per-leg foothold optimization on 3D volumetric OctoMap',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'foothold_optimizer = spooder_foothold.foothold_optimizer:main',
        ],
    },
)
