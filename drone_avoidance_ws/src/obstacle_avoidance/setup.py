# Author: Mokshagna Anurag
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mokshagna Anurag',
    maintainer_email='barkath@todo.todo',
    description='Obstacle avoidance package for PX4 drone using depth camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_avoidance = obstacle_avoidance.depth_avoidance:main',
            'path_planner = obstacle_avoidance.path_planner:main',
            'mapping_node = obstacle_avoidance.mapping_node:main',
            'offboard = obstacle_avoidance.offboard:main'
        ],
    },
)
