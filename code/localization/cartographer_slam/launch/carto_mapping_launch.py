"""Cartographer mapping: cartographer_node + occupancy grid + carto_node (+ odom_noise).

ros2 launch cartographer_slam carto_mapping_launch.py map_name:=Spielberg \
    origin_x:=<sx> origin_y:=<sy> origin_yaw:=<stheta>
Save: ros2 service call /carto/save_map std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP_DIR = os.path.expanduser('~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/data')


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('cartographer_slam')
    config_dir = os.path.join(share, 'config')
    use_sim_time = arg('use_sim_time') == 'true'
    odom_noise = arg('odom_noise') == 'true'
    map_name = arg('map_name')
    if not map_name:
        raise RuntimeError('map_name:=<name> is required (e.g. the track name)')
    origin = [float(arg('origin_x')), float(arg('origin_y')), float(arg('origin_yaw'))]

    actions = []
    if odom_noise:
        odom_topic = '/ego_racecar/odom_noisy'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('odom_noise'), 'launch', 'odom_noise_launch.py')),
            launch_arguments={'use_sim_time': arg('use_sim_time')}.items(),
        ))
    else:
        odom_topic = '/ego_racecar/odom'

    actions += [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', 'carto_mapping.lua'],
            remappings=[('scan', '/scan'), ('odom', odom_topic)],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            remappings=[('map', '/carto/map')],   # the sim's map_server owns /map
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='cartographer_slam',
            executable='carto_node',
            name='carto_node',
            output='screen',
            parameters=[arg('params_file'), {
                'use_sim_time': use_sim_time,
                'mode': 'mapping',
                'origin': origin,
                'map_dir': os.path.expanduser(arg('map_dir')),
                'map_name': map_name,
                'configuration_directory': config_dir,
            }],
        ),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory('cartographer_slam')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'carto_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_noise', default_value='false',
                              description='true: start odom_noise and map with its odometry'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='saved as <map_dir>/<map_name>.pbstream + .yaml'),
        DeclareLaunchArgument('map_dir', default_value=DEFAULT_MAP_DIR),
        DeclareLaunchArgument('origin_x', default_value='0.0',
                              description='car pose in map at mapping start (sx in sim.yaml)'),
        DeclareLaunchArgument('origin_y', default_value='0.0', description='sy in sim.yaml'),
        DeclareLaunchArgument('origin_yaw', default_value='0.0', description='stheta in sim.yaml'),
        OpaqueFunction(function=_nodes),
    ])