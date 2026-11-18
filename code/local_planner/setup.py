"""Setuptools configuration for the local_planner ROS 2 package."""

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedarklord',
    maintainer_email='miracleyt404@gmail.com',
    description='Scan-aware local planner (gate + Frenet / RRT*) on the global raceline',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'local_planner_node = local_planner.local_planner_node:main',
        ],
    },
)
