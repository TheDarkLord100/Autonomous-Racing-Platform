import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('car_bringup'), 'config', 'lidar.yaml')

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[cfg],
        output='screen',
        remappings=[('scan', 'scan_raw')],
    )

    # Rotates scan_raw (laser frame) into base_link orientation and publishes /scan
    scan_filter = Node(
        package='car_bringup',
        executable='scan_filter',
        name='scan_filter_node',
        parameters=[{'yaw_offset': 3.14159, 'frame_id': 'base_link'}],
    )

    # RPLiDAR's laser frame +x points opposite the housing arrow -> yaw = pi
    # TODO: fill x/z with measured offset from base_link (rear axle centre)
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '3.14159',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    return LaunchDescription([lidar, scan_filter, base_to_laser])