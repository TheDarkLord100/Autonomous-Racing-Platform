"""Launch the scan-matching node, optionally with the odom_noise node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare the launch arguments and start scan matching (and odom_noise if set)."""
    pkg = get_package_share_directory('scan_matching')
    default_params = os.path.join(pkg, 'config', 'scan_matching_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_noise = LaunchConfiguration('odom_noise')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'odom_noise', default_value='true',
            description='Also start odom_noise (/ego_racecar/odom_noisy)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('odom_noise'), 'launch',
                'odom_noise_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(odom_noise)),

        Node(
            package='scan_matching',
            executable='scan_matching_node',
            name='scan_matching',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),
    ])