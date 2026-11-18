"""Cartographer pure localization on a saved map.

ros2 launch cartographer_slam carto_localization_launch.py map_name:=Spielberg
Then set the pose with RViz "2D Pose Estimate" (or use_initial_pose_param in the params file).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP_DIR = os.path.expanduser('~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/data')


def _nodes(context):
    """Build the Cartographer localization actions from the resolved launch arguments.

    Reads ``<map_dir>/<map_name>.yaml`` for the pbstream and origin. Raises RuntimeError if
    ``map_name`` is empty.
    """
    def arg(name):
        """Return the value of a launch argument."""
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('cartographer_slam')
    config_dir = os.path.join(share, 'config')
    use_sim_time = arg('use_sim_time') == 'true'
    odom_noise = arg('odom_noise') == 'true'

    map_name = arg('map_name')
    if not map_name:
        raise RuntimeError('map_name:=<name> is required')
    map_dir = os.path.expanduser(arg('map_dir'))
    sidecar = os.path.join(map_dir, map_name + '.yaml')
    if not os.path.isfile(sidecar):
        raise RuntimeError(f'{sidecar} not found (written by /carto/save_map when mapping)')
    with open(sidecar) as f:
        meta = yaml.safe_load(f)
    pbstream = os.path.join(map_dir, meta['pbstream'])
    if not os.path.isfile(pbstream):
        raise RuntimeError(f'{pbstream} not found')
    origin = [float(v) for v in meta['origin']]

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
                       '-configuration_basename', 'carto_localization.lua',
                       '-load_state_filename', pbstream,
                       '-load_frozen_state=true',
                       '-start_trajectory_with_default_topics=false'],  # carto_node starts it
            remappings=[('scan', '/scan'), ('odom', odom_topic)],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            remappings=[('map', '/carto/map')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='cartographer_slam',
            executable='carto_node',
            name='carto_node',
            output='screen',
            parameters=[arg('params_file'), {
                'use_sim_time': use_sim_time,
                'mode': 'localization',
                'origin': origin,
                'configuration_directory': config_dir,
                'configuration_basename': 'carto_localization.lua',
            }],
        ),
    ]
    return actions


def generate_launch_description():
    """Declare the launch arguments and defer node creation to ``_nodes``."""
    share = get_package_share_directory('cartographer_slam')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'carto_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_noise', default_value='true',
                              description='true: start odom_noise and localize with its odometry'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='loads <map_dir>/<map_name>.yaml and its .pbstream'),
        DeclareLaunchArgument('map_dir', default_value=DEFAULT_MAP_DIR),
        OpaqueFunction(function=_nodes),
    ])