"""Run pid_wall on the car, gated by the web bridge.

pid_wall's /drive is remapped to /drive_auto; the web bridge only forwards it
to /drive while the driver holds AUTO. Start the car with
`bringup.launch.py teleop:=web` first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('car_bringup'), 'config', 'pid_wall.yaml')
    return LaunchDescription([
        Node(
            package='pid_wall',
            executable='pid_node',
            name='pid_node',
            output='screen',
            parameters=[config],
            remappings=[('/drive', '/drive_auto')],
        ),
    ])
