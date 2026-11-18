"""Launch the raceline publisher, pure pursuit and the lap logger."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare the launch arguments and start the three nodes."""
    cfg = os.path.join(get_package_share_directory('pure_pursuit'), 'config', 'pp_sim.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('csv_path', description='raceline CSV to publish'),
        DeclareLaunchArgument('log', default_value='true', description='run the lap logger'),
        DeclareLaunchArgument('tag', default_value='', description='label for the log file'),
        Node(package='global_planner', executable='raceline_publisher',
             name='raceline_publisher', output='screen',
             parameters=[{'csv_path': LaunchConfiguration('csv_path')}]),
        Node(package='pure_pursuit', executable='pure_pursuit_node',
             name='pure_pursuit', output='screen', parameters=[cfg]),
        Node(package='race_eval', executable='lap_logger', name='lap_logger',
             output='screen', condition=IfCondition(LaunchConfiguration('log')),
             parameters=[{'tag': LaunchConfiguration('tag')}]),
    ])