"""One-command simulation bringup.

  ros2 launch simulation_bringup sim_bringup_launch.py                          # GT pose
  ros2 launch simulation_bringup sim_bringup_launch.py localization:=pf
  ros2 launch simulation_bringup sim_bringup_launch.py localization:=carto odom_noise:=true
  ros2 launch simulation_bringup sim_bringup_launch.py localization:=scan_matching planner:=rrt
  ros2 launch simulation_bringup sim_bringup_launch.py config:=gt_gt.yaml tf_source:=localization

Order (event-based: each stage starts when the previous one logs that it is ready):
  1. run folder + logging, static TF map -> stack frame, gym (bridge, world map server,
     robot model, RViz/Foxglove), stack map server + its lifecycle manager
  2. localization (pf | carto | scan_matching, + odom_noise, + relay / TF broadcaster)
                                     after the stack map server is active
  3. raceline publisher, local planner, pure pursuit, lap logger, run monitor, bag
                                     after the localizer is ready (gt: straight after 1)
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from simulation_bringup.config_resolver import (BASE_FRAME, ODOM_FRAME, SCENARIO_ARGS,
                                                describe, resolve)
from simulation_bringup.launch_helpers import (PKG, after_log, bag, carto_node,
                                               cartographer_node, gym, monitor, node,
                                               odom_noise, say, start_run, static_tf)


def _setup(context):
    share = Path(get_package_share_directory(PKG))
    arg = LaunchConfiguration('config').perform(context)
    config_path = Path(arg) if os.path.isabs(arg) else share / 'config' / arg
    overrides = {k: LaunchConfiguration(k).perform(context) for k in SCENARIO_ARGS}
    cfg = resolve(str(config_path), str(share / 'config' / 'params'), overrides)
    d, run, stack, P = cfg['derived'], cfg['run'], cfg['stack'], cfg['params']
    method = cfg['localization']['method']
    timeout = float(run['stage_timeout'])

    run_dir, out = start_run(cfg, describe(cfg))

    # ---------------- stage 1: world + maps + TF ----------------
    out.append(gym(cfg, run_dir, d['rviz_file']))
    if d['static_tf']:
        out.append(static_tf(*stack['tf'], 'map', stack['frame']))
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

    # ---------------- stage 3: planning, control, evaluation ----------------
    stack_nodes = [
        node('global_planner', 'raceline_publisher', 'raceline_publisher',
             [{'csv_path': stack['raceline'], 'frame_id': stack['frame']}]),
        node('local_planner', 'local_planner_node', 'local_planner',
             [P['local_planner'], {'planner': cfg['planning']['local_planner'],
                                   'pose_topic': d['pose_topic']}]),
        node('pure_pursuit', 'pure_pursuit_node', 'pure_pursuit',
             [P['pure_pursuit'], {'waypoints_topic': '/local_waypoints',
                                  'pose_topic': d['pose_topic']}]),
        # truth: GT pose against the global raceline
        node('race_eval', 'lap_logger', 'lap_logger',
             [{'log_dir': str(run_dir), 'tag': run['tag'], 'pose_topic': '/ego_racecar/odom'}]),
    ]
    if run['monitor']:
        stack_nodes.append(monitor(cfg, run_dir))
    if run['record_bag']:
        stack_nodes.append(bag(cfg, run_dir))

    if method == 'gt':
        # nothing to wait for; a short delay lets the gym come up first
        out.append(TimerAction(period=2.0, actions=[
            say('stage 3: GT odometry -> starting the stack')] + stack_nodes))
        return out

    # ---------------- stage 2: localization ----------------
    loc = []
    if cfg['localization']['odom_noise']:
        loc.append(odom_noise(cfg))
    if d['tf_from_localization']:     # odometry half of map -> odom -> base_link
        loc.append(node(PKG, 'odom_tf_broadcaster', 'odom_tf_broadcaster',
                        [{'odom_topic': d['odom_in'], 'odom_frame': ODOM_FRAME,
                          'base_frame': BASE_FRAME}]))
    if method == 'pf':
        ready = node('particle_filter', 'pf_node', 'particle_filter',
                     [P['particle_filter'], d['pf_overrides']])
        loc.append(ready)
    else:
        if method == 'scan_matching':
            loc.append(node('scan_matching', 'scan_matching_node', 'scan_matching',
                            [P['scan_matching'], d['sm_overrides']]))
        else:  # carto
            loc.append(cartographer_node('carto_localization.lua', d['odom_in'], [
                '-load_state_filename', d['carto']['pbstream'], '-load_frozen_state=true',
                '-start_trajectory_with_default_topics=false']))
            loc.append(carto_node(cfg, d['carto_overrides'], 'carto_localization.lua'))
        ready = node(PKG, 'localization_relay', 'localization_relay', [d['relay']])
        loc.append(ready)
    name, pattern = d['ready']
    loc += after_log(ready, name, pattern, stack_nodes,
                     f'stage 3: {method} ready -> starting the stack', timeout)

    if stack_lifecycle is not None:
        out += after_log(stack_lifecycle, 'stack_map_lifecycle', 'Managed nodes are active',
                         loc, f'stage 2: stack map active -> starting {method}', timeout)
    else:
        out += [say(f'stage 2: starting {method} on the world map')] + loc
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='bringup.yaml',
                              description='scenario file in config/, or an absolute path'),
        DeclareLaunchArgument('localization', default_value='',
                              description='override: gt | pf | carto | scan_matching'),
        DeclareLaunchArgument('odom_noise', default_value='',
                              description='override: true | false (localizers only)'),
        DeclareLaunchArgument('planner', default_value='',
                              description='override: passthrough | frenet | rrt'),
        DeclareLaunchArgument('tf_source', default_value='',
                              description='override: gym (truth view) | localization (belief)'),
        DeclareLaunchArgument('tag', default_value='', description='override run.tag'),
        OpaqueFunction(function=_setup),
    ])
