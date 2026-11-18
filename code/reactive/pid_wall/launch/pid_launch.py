"""Launch the PID wall-following node with config/pid_node_params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the PID wall-following node with its parameter file."""
    config = os.path.join(
        get_package_share_directory('pid_wall'),
        'config',
        'pid_node_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='pid_wall',
            executable='pid_node',
            name='pid_node',
            output='screen',
            parameters=[config],
        ),
    ])