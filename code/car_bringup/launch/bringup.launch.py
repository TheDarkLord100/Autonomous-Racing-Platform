import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_bringup')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    web_bridge_config = os.path.join(pkg_share, 'config', 'web_bridge.yaml')

    # teleop:=keyboard -> twist_to_ackermann (run teleop_twist_keyboard yourself over SSH)
    # teleop:=web      -> web_bridge on http://<jetson>:8080
    # Only one of them publishes /drive at a time.
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='keyboard', choices=['keyboard', 'web'],
        description='Which teleop source drives /drive')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar.launch.py')
        )
    )

    return LaunchDescription([
        teleop_arg,
        Node(package='vesc_driver', executable='vesc_driver_node',
             name='vesc_driver_node', parameters=[vesc_config]),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
             name='ackermann_to_vesc_node', parameters=[vesc_config],
             remappings=[('ackermann_cmd', '/drive')]),
        Node(package='vesc_ackermann', executable='vesc_to_odom_node',
             name='vesc_to_odom_node', parameters=[vesc_config]),
        Node(package='car_bringup', executable='twist_to_ackermann',
             name='twist_to_ackermann_node', parameters=[vesc_config],
             condition=LaunchConfigurationEquals('teleop', 'keyboard')),
        Node(package='car_bringup', executable='web_bridge',
             name='web_bridge_node', parameters=[web_bridge_config],
             output='screen',
             condition=LaunchConfigurationEquals('teleop', 'web')),
        lidar_launch,
    ])