"""Launch the gap-follow node with config/params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare ``publish_markers`` and start the gap-follow node."""
    config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'params.yaml'
    )

    publish_markers = DeclareLaunchArgument(
        'publish_markers', default_value='false',
        description='Publish RViz debug markers (costs several ms per scan)')

    return LaunchDescription([
        publish_markers,
        Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[
                config,
                # Listed after the YAML so the launch argument wins.
                {'publish_markers': ParameterValue(
                    LaunchConfiguration('publish_markers'), value_type=bool)},
            ]
        )
    ])