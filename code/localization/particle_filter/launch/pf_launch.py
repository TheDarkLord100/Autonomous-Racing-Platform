import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    odom_noise = LaunchConfiguration('odom_noise').perform(context) == 'true'

    pf_overrides = {'use_sim_time': use_sim_time == 'true'}
    actions = []
    if odom_noise:
        # start the shared odom_noise node; the PF reads its drifting odometry
        pf_overrides['odom_topic'] = '/ego_racecar/odom_noisy'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('odom_noise'), 'launch', 'odom_noise_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ))
    actions.append(Node(
        package='particle_filter',
        executable='pf_node',
        name='particle_filter',
        output='screen',
        parameters=[params, pf_overrides],
    ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('particle_filter')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'pf_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_noise', default_value='false',
                              description='true: start the odom_noise node and feed its '
                                          'drifting odometry to the PF'),
        OpaqueFunction(function=_nodes),
    ])