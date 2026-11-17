"""raceline_publisher -> local_planner -> pure_pursuit (+ lap_logger), in sim.

ros2 launch local_planner local_planner_sim_launch.py \
    csv_path:=<raceline.csv> planner:=frenet|rrt|passthrough tag:=<label>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lp_cfg = os.path.join(get_package_share_directory('local_planner'), 'config',
                          'local_planner_sim.yaml')
    pp_cfg = os.path.join(get_package_share_directory('pure_pursuit'), 'config', 'pp_sim.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('csv_path', description='raceline CSV to publish'),
        DeclareLaunchArgument('planner', default_value='frenet',
                              description='passthrough | frenet | rrt'),
        DeclareLaunchArgument('log', default_value='true', description='run the lap logger'),
        DeclareLaunchArgument('tag', default_value='', description='label for the log file'),
        Node(package='global_planner', executable='raceline_publisher',
             name='raceline_publisher', output='screen',
             parameters=[{'csv_path': LaunchConfiguration('csv_path')}]),
        Node(package='local_planner', executable='local_planner_node',
             name='local_planner', output='screen',
             parameters=[lp_cfg, {'planner': LaunchConfiguration('planner')}]),
        # pure pursuit unchanged, it just tracks the local path instead of the global one
        Node(package='pure_pursuit', executable='pure_pursuit_node',
             name='pure_pursuit', output='screen',
             parameters=[pp_cfg, {'waypoints_topic': '/local_waypoints'}]),
        # lap logger still measures against the GLOBAL raceline
        Node(package='race_eval', executable='lap_logger', name='lap_logger',
             output='screen', condition=IfCondition(LaunchConfiguration('log')),
             parameters=[{'tag': LaunchConfiguration('tag')}]),
    ])
