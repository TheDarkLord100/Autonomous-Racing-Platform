import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params = LaunchConfiguration('params_file').perform(context)
    noise_params = LaunchConfiguration('odom_noise_params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    odom_noise = LaunchConfiguration('odom_noise').perform(context) == 'true'

    pf_overrides = {'use_sim_time': use_sim_time}
    nodes = []
    if odom_noise:
        # PF reads the drifting odometry instead of the sim's ground truth
        pf_overrides['odom_topic'] = '/ego_racecar/odom_noisy'
        nodes.append(Node(
            package='particle_filter',
            executable='odom_noise_node',
            name='odom_noise',
            output='screen',
            parameters=[noise_params, {'use_sim_time': use_sim_time}],
        ))
    nodes.append(Node(
        package='particle_filter',
        executable='pf_node',
        name='particle_filter',
        output='screen',
        parameters=[params, pf_overrides],
    ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('particle_filter')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'pf_params.yaml')),
        DeclareLaunchArgument('odom_noise_params_file',
                              default_value=os.path.join(share, 'config', 'odom_noise_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_noise', default_value='false',
                              description='true: corrupt sim odometry and feed it to the PF'),
        OpaqueFunction(function=_nodes),
    ])