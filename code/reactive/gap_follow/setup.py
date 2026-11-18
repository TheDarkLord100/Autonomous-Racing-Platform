"""Setuptools configuration for the gap_follow ROS 2 package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedarklord',
    maintainer_email='miracleyt404@gmail.com',
    description='Reactive Follow-the-Gap controller (Python port of gap_follow_pkg)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gap_follow_node = gap_follow.gap_follow_node:main',
        ],
    },
)
