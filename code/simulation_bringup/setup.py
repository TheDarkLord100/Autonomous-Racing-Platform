"""Setuptools configuration for the simulation_bringup ROS 2 package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'simulation_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'params'), glob('config/params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedarklord',
    maintainer_email='miracleyt404@gmail.com',
    description='One-command simulation bringup for the F1TENTH stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_monitor = simulation_bringup.run_monitor:main',
            'odom_tf_broadcaster = simulation_bringup.odom_tf_broadcaster:main',
            'localization_relay = simulation_bringup.localization_relay:main',
        ],
    },
)
