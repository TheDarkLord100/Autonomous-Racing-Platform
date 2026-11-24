"""Launch building blocks shared by sim_bringup_launch.py and mapping_bringup_launch.py."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from simulation_bringup.config_resolver import make_run_dir

PKG = 'simulation_bringup'


def say(msg, color='36'):
    """Return a LogInfo action that prints a coloured ``[bringup]`` message."""
    return LogInfo(msg=f'\033[{color}m[bringup] {msg}\033[0m')


def node(pkg, exe, name, params=(), screen=False, **kw):
    """Every node logs to the run folder; only screen=True ones print here."""
    return Node(package=pkg, executable=exe, name=name, parameters=list(params),
                output='screen' if screen else 'log', **kw)


def after_log(target, target_name, pattern, actions, label, timeout):
    """Start `actions` the first time `target` prints `pattern` (stdout or stderr);
    warn on the terminal if that hasn't happened after `timeout` seconds."""
    state = {'done': False}

    def on_output(event):
        """Return the stage actions the first time the pattern appears in the output."""
        if state['done'] or pattern not in event.text.decode(errors='ignore'):
            return None
        state['done'] = True
        return [say(label)] + actions

    def check(_context):
        """Return a warning message if the pattern has not appeared by the timeout."""
        if not state['done']:
            return [say(f'still waiting for "{pattern}" from {target_name} after '
                        f'{timeout:.0f} s; check its log in the run folder', '33')]
        return []

    return [RegisterEventHandler(OnProcessIO(target_action=target, on_stdout=on_output,
                                             on_stderr=on_output)),
            TimerAction(period=timeout, actions=[OpaqueFunction(function=check)])]


def start_run(cfg, describe_lines):
    """Run folder, resolved config copy, logging env, summary lines."""
    run_dir = make_run_dir(cfg)
    with open(run_dir / 'bringup_resolved.yaml', 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    try:  # raw stdout/stderr of every process lives in the launch log folder
        import launch.logging
        (run_dir / 'launch_log').symlink_to(Path(launch.logging.launch_config.log_dir),
                                            target_is_directory=True)
    except Exception:
        pass
    actions = [
        SetEnvironmentVariable('ROS_LOG_DIR', str(run_dir / 'ros_logs')),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        say(f'run folder: {run_dir}'),
    ] + [say(line) for line in describe_lines]
    return run_dir, actions


def gym(cfg, run_dir, rviz_file):
    """The gym bridge launch with this run's sim params (written to the run folder)."""
    share = Path(get_package_share_directory(PKG))
    with open(cfg['params']['sim']) as f:
        sim = yaml.safe_load(f)
    sim['bridge']['ros__parameters'].update(cfg['derived']['sim_overrides'])
    sim_path = run_dir / 'sim_resolved.yaml'
    with open(sim_path, 'w') as f:
        yaml.safe_dump(sim, f, sort_keys=False)
    viz = cfg['run']['viz']
    args = {
        'config': str(sim_path),
        'map_path': cfg['world']['map'],
        'use_rviz': 'true' if viz == 'rviz' else 'false',
        'rviz_config': str(share / 'rviz' / rviz_file),
        'open_foxglove': 'true' if viz == 'foxglove' else 'false',
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge_launch.py')),
        launch_arguments=args.items())


def static_tf(x, y, yaw, parent, child):
    """Return a static_transform_publisher node for ``parent -> child``."""
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'tf_{parent}_to_{child}'.replace('/', '_'), output='log',
                arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                           '--frame-id', parent, '--child-frame-id', child])


def odom_noise(cfg):
    """Return the odom_noise node with its resolved parameter file."""
    return node('odom_noise', 'odom_noise_node', 'odom_noise', [cfg['params']['odom_noise']])


def monitor(cfg, run_dir):
    """Return the run_monitor node writing ``summary.csv`` into the run folder."""
    return node(PKG, 'run_monitor', 'run_monitor',
                [{'summary_path': str(run_dir / 'summary.csv'), 'tag': cfg['run']['tag']}],
                screen=True)


def bag(cfg, run_dir):
    """Return a ``ros2 bag record`` process for ``run.bag_topics`` into the run folder."""
    from launch.actions import ExecuteProcess
    return ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(run_dir / 'bag')] +
                          cfg['run']['bag_topics'], name='bag_record', output='log')


def cartographer_node(lua, odom_topic, extra_args=(), params=()):
    """Return the cartographer_node process with the given lua file and odometry topic."""
    config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    return node('cartographer_ros', 'cartographer_node', 'cartographer_node', params,
                arguments=['-configuration_directory', config_dir,
                           '-configuration_basename', lua] + list(extra_args),
                remappings=[('scan', '/scan'), ('odom', odom_topic)])

def carto_node(cfg, overrides, lua):
    """Return the carto_node wrapper with its parameter file, overrides and lua file."""
    config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    return node('cartographer_slam', 'carto_node', 'carto_node',
                [cfg['params']['carto'], dict(overrides, configuration_directory=config_dir,
                                              configuration_basename=lua)])
