"""One-command simulation bringup.

  ros2 launch simulation_bringup sim_bringup_launch.py                     # config/bringup.yaml
  ros2 launch simulation_bringup sim_bringup_launch.py config:=gt_gt.yaml planner:=rrt tag:=x

Order (event-based: each stage starts when the previous one logs that it is ready):
  1. run folder + logging, static TF map -> stack frame, gym (bridge, world map server,
     robot model, RViz/Foxglove), stack map server + its lifecycle manager
  2. localization (PF [+ odom_noise])         after the stack map server is active
  3. raceline publisher, local planner, pure pursuit, lap logger, run monitor, bag
                                              after the PF logs "initialized at"
                                              (method gt: straight after stage 1)

Every node logs to <log_root>/<YYYYmmdd_HHMMSS>_<tag>/; only the run monitor and
the bringup's own stage lines print to this terminal.
"""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simulation_bringup.config_resolver import describe, make_run_dir, resolve

PKG = 'simulation_bringup'


def _say(msg):
    return LogInfo(msg=f'\033[36m[bringup] {msg}\033[0m')


def _after_log(target, target_name, pattern, actions, label, timeout):
    """Start `actions` the first time `target` prints `pattern` (stdout or stderr).
    Warn on the terminal if that hasn't happened after `timeout` seconds."""
    state = {'done': False}

    def on_output(event):
        if state['done'] or pattern not in event.text.decode(errors='ignore'):
            return None
        state['done'] = True
        return [_say(label)] + actions

    def check(_context):
        if not state['done']:
            return [LogInfo(msg=f'\033[33m[bringup] still waiting for "{pattern}" from '
                                f'{target_name} after {timeout:.0f} s; check its log in '
                                f'the run folder\033[0m')]
        return []

    return [RegisterEventHandler(OnProcessIO(target_action=target, on_stdout=on_output,
                                             on_stderr=on_output)),
            TimerAction(period=timeout, actions=[OpaqueFunction(function=check)])]


def _setup(context):
    share = Path(get_package_share_directory(PKG))
    arg = LaunchConfiguration('config').perform(context)
    config_path = Path(arg) if os.path.isabs(arg) else share / 'config' / arg
    overrides = {k: LaunchConfiguration(k).perform(context)
                 for k in ('planner', 'localization', 'tag')}
    cfg = resolve(str(config_path), str(share / 'config' / 'params'), overrides)
    d, run, stack, P = cfg['derived'], cfg['run'], cfg['stack'], cfg['params']

    # ---------------- run folder ----------------
    run_dir = make_run_dir(cfg)
    with open(run_dir / 'bringup_resolved.yaml', 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    try:  # raw stdout/stderr of every process lives in the launch log folder
        import launch.logging
        launch_log = Path(launch.logging.launch_config.log_dir)
        (run_dir / 'launch_log').symlink_to(launch_log, target_is_directory=True)
    except Exception:
        pass

    out = [
        SetEnvironmentVariable('ROS_LOG_DIR', str(run_dir / 'ros_logs')),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        _say(f'run folder: {run_dir}'),
    ] + [_say(line) for line in describe(cfg)]

    def node(pkg, exe, name, params=(), screen=False, **kw):
        return Node(package=pkg, executable=exe, name=name, parameters=list(params),
                    output='screen' if screen else 'log', **kw)

    # ---------------- stage 1: world + maps + TF ----------------
    gym_args = {
        'config': P['sim'],
        'map_path': cfg['world']['map'],
        'use_rviz': 'true' if run['viz'] == 'rviz' else 'false',
        'rviz_config': str(share / 'rviz' / 'sim.rviz'),
        'open_foxglove': 'true' if run['viz'] == 'foxglove' else 'false',
    }
    out.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge_launch.py')),
        launch_arguments=gym_args.items()))

    if d['static_tf']:
        x, y, yaw = stack['tf']
        out.append(Node(package='tf2_ros', executable='static_transform_publisher',
                        name=f'tf_map_to_{stack["frame"]}', output='log',
                        arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                                   '--frame-id', 'map', '--child-frame-id', stack['frame']]))

    stack_lifecycle = None
    if d['stack_map_server']:
        out.append(node('nav2_map_server', 'map_server', 'stack_map_server', [{
            'yaml_filename': d['stack_map_yaml'],
            'topic_name': d['stack_map_topic'].lstrip('/'),
            'frame_id': stack['frame']}]))
        stack_lifecycle = node('nav2_lifecycle_manager', 'lifecycle_manager',
                               'stack_map_lifecycle',
                               [{'node_names': ['stack_map_server'], 'autostart': True}])
        out.append(stack_lifecycle)

    # ---------------- stage 3 nodes: planning, control, evaluation ----------------
    planner = cfg['planning']['local_planner']
    stack_nodes = [
        node('global_planner', 'raceline_publisher', 'raceline_publisher',
             [{'csv_path': stack['raceline'], 'frame_id': stack['frame']}]),
        node('local_planner', 'local_planner_node', 'local_planner',
             [P['local_planner'], {'planner': planner, 'pose_topic': d['pose_topic']}]),
        node('pure_pursuit', 'pure_pursuit_node', 'pure_pursuit',
             [P['pure_pursuit'], {'waypoints_topic': '/local_waypoints',
                                  'pose_topic': d['pose_topic']}]),
        # truth: GT pose against the global raceline
        node('race_eval', 'lap_logger', 'lap_logger',
             [{'log_dir': str(run_dir), 'tag': run['tag'], 'pose_topic': '/ego_racecar/odom'}]),
    ]
    if run['monitor']:
        stack_nodes.append(node(PKG, 'run_monitor', 'run_monitor',
                                [{'summary_path': str(run_dir / 'summary.csv'),
                                  'tag': run['tag']}], screen=True))
    if run['record_bag']:
        stack_nodes.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', str(run_dir / 'bag')] + run['bag_topics'],
            name='bag_record', output='log'))

    # ---------------- stage 2: localization, then stage 3 ----------------
    timeout = float(run['stage_timeout'])
    if cfg['localization']['method'] == 'pf':
        loc_nodes = []
        if cfg['localization']['odom_noise']:
            loc_nodes.append(node('odom_noise', 'odom_noise_node', 'odom_noise',
                                  [P['odom_noise']]))
        pf = node('particle_filter', 'pf_node', 'particle_filter',
                  [P['particle_filter'], d['pf_overrides']])
        loc_nodes.append(pf)
        loc_nodes += _after_log(pf, 'particle_filter', 'initialized at', stack_nodes,
                                'stage 3: particle filter initialized -> starting the stack',
                                timeout)
        if stack_lifecycle is not None:
            out += _after_log(stack_lifecycle, 'stack_map_lifecycle',
                              'Managed nodes are active', loc_nodes,
                              'stage 2: stack map active -> starting localization', timeout)
        else:
            out += [_say('stage 2: localization on the world map')] + loc_nodes
    else:
        # GT pose: nothing to wait for; a short delay lets the gym come up first
        out.append(TimerAction(period=2.0, actions=[
            _say('stage 3: GT odometry -> starting the stack')] + stack_nodes))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='bringup.yaml',
                              description='scenario file in config/, or an absolute path'),
        DeclareLaunchArgument('planner', default_value='',
                              description='override planning.local_planner'),
        DeclareLaunchArgument('localization', default_value='',
                              description='override localization.method (gt | pf)'),
        DeclareLaunchArgument('tag', default_value='', description='override run.tag'),
        OpaqueFunction(function=_setup),
    ])
