import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_bringup')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar.launch.py')
        )
    )

    return LaunchDescription([
        Node(package='vesc_driver', executable='vesc_driver_node',
             name='vesc_driver_node', parameters=[vesc_config]),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
             name='ackermann_to_vesc_node', parameters=[vesc_config],
             remappings=[('ackermann_cmd', '/drive')]),
        Node(package='vesc_ackermann', executable='vesc_to_odom_node',
             name='vesc_to_odom_node', parameters=[vesc_config]),
        Node(package='car_bringup', executable='twist_to_ackermann',
             name='twist_to_ackermann_node', parameters=[vesc_config]),
        lidar_launch,
    ])