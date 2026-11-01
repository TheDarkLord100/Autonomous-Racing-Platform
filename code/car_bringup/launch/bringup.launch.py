import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('car_bringup'), 'config', 'vesc.yaml')

    return LaunchDescription([
        Node(package='vesc_driver', executable='vesc_driver_node',
             name='vesc_driver_node', parameters=[vesc_config]),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
             name='ackermann_to_vesc_node', parameters=[vesc_config],
             remappings=[('ackermann_cmd', '/drive')]),
        Node(package='vesc_ackermann', executable='vesc_to_odom_node',
             name='vesc_to_odom_node', parameters=[vesc_config]),
    ])