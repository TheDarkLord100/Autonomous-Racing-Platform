"""Replay the recorded bag into one localizer for the localization comparison.

  B=~/f1tenth_simulation_logs/20261123_235619_locbag_Spielberg_ftg_3mps/bag
  ros2 launch simulation_bringup loc_replay_launch.py bag:=$B localization:=pf
  ros2 launch simulation_bringup loc_replay_launch.py bag:=$B localization:=carto
  ros2 launch simulation_bringup loc_replay_launch.py bag:=$B localization:=scan_matching

No gym and no odom_noise: /scan, GT odometry, the noisy odometry and TF all come from the
bag, so every localizer gets exactly the same input. The localizer runs on sim time (bag
played with --clock). Order: stack map server (+ map -> stack frame TF) -> localizer ->
after play_delay s, recorder + bag play. Shuts down when the bag ends.
Output: <log_root>/<time>_locreplay_<method>/bag_out (GT odom, estimate, compute time).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from simulation_bringup.config_resolver import GT_ODOM, resolve
from simulation_bringup.launch_helpers import (PKG, after_log, carto_node, cartographer_node,
                                               node, say, start_run, static_tf)

PLAY_TOPICS = ['/scan', '/ego_racecar/odom', '/ego_racecar/odom_noisy', '/tf', '/tf_static']
OUT_TOPICS = {'pf': ['/pf/pose', '/pf/compute_ms'],
              'carto': ['/carto/pose'],
              'scan_matching': ['/scan_match/odom', '/scan_match/compute_ms']}


def _setup(context):
    """Resolve the scenario for one localizer and return the replay actions."""
    share = Path(get_package_share_directory(PKG))

    def arg(k):
        return LaunchConfiguration(k).perform(context)

    method = arg('localization')
    if method not in OUT_TOPICS:
        raise ValueError(f'localization must be one of {tuple(OUT_TOPICS)}, got {method!r}')
    in_bag = Path(os.path.expanduser(arg('bag')))
    if not in_bag.exists():
        raise FileNotFoundError(f'bag: {in_bag} does not exist')
    cfg_arg = arg('config')
    config_path = Path(cfg_arg) if os.path.isabs(cfg_arg) else share / 'config' / cfg_arg
    cfg = resolve(str(config_path), str(share / 'config' / 'params'),
                  {'localization': method, 'odom_noise': 'true', 'tf_source': 'gym',
                   'tag': f'locreplay_{method}'})
    d, stack, P = cfg['derived'], cfg['stack'], cfg['params']
    timeout = float(cfg['run']['stage_timeout'])

    run_dir, out = start_run(cfg, [
        f'input bag : {in_bag}',
        f'localizer : {method} on {d["stack_map_yaml"]} (frame {stack["frame"]})',
        f'odometry  : {d["odom_in"]} from the bag',
    ])

    # ---- stage 1: stack map + TF (wall time; no gym here, so we always serve the map)
    if d['static_tf']:
        out.append(static_tf(*stack['tf'], 'map', stack['frame']))
    out.append(node('nav2_map_server', 'map_server', 'stack_map_server', [{
        'yaml_filename': d['stack_map_yaml'],
        'topic_name': d['stack_map_topic'].lstrip('/'),
        'frame_id': stack['frame']}]))
    lifecycle = node('nav2_lifecycle_manager', 'lifecycle_manager', 'stack_map_lifecycle',
                     [{'node_names': ['stack_map_server'], 'autostart': True}])
    out.append(lifecycle)

    # ---- stage 2: localizer on sim time (use_sim_time in each node's own params)
    sim = {'use_sim_time': True}
    if method == 'pf':
        loc = [node('particle_filter', 'pf_node', 'particle_filter',
                    [P['particle_filter'], d['pf_overrides'], sim])]
    elif method == 'scan_matching':
        loc = [node('scan_matching', 'scan_matching_node', 'scan_matching',
                    [P['scan_matching'], d['sm_overrides'], sim])]
    else:  # carto
        loc = [cartographer_node('carto_localization.lua', d['odom_in'], [
                   '-load_state_filename', d['carto']['pbstream'], '-load_frozen_state=true',
                   '-start_trajectory_with_default_topics=false'], [sim]),
               carto_node(cfg, dict(d['carto_overrides'], use_sim_time=True),
                          'carto_localization.lua')]

    # ---- stage 3: record (wall-clock receive times, for latency) + play
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', str(run_dir / 'bag_out'), GT_ODOM] +
        OUT_TOPICS[method], name='bag_record', output='log')
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', str(in_bag), '--clock', '100', '--rate', arg('rate'),
             '--topics'] + PLAY_TOPICS, name='bag_play', output='screen')
    finish = RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[
        say('bag finished -> shutting down', '32'),
        TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='bag finished'))])]))
    start = [say(f'recording to {run_dir / "bag_out"}; playing the bag', '32'),
             record, finish, TimerAction(period=1.0, actions=[play])]

    stage2 = loc + [TimerAction(period=float(arg('play_delay')), actions=start)]
    out += after_log(lifecycle, 'stack_map_lifecycle', 'Managed nodes are active', stage2,
                     f'stack map active -> starting {method}', timeout)
    return out


def generate_launch_description():
    """Declare the replay arguments and defer the actions to ``_setup``."""
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='input bag folder from loc_record_launch'),
        DeclareLaunchArgument('localization', default_value='pf',
                              description='pf | carto | scan_matching'),
        DeclareLaunchArgument('config', default_value='bringup.yaml',
                              description='scenario for the stack map / frame / tf'),
        DeclareLaunchArgument('rate', default_value='1.0', description='bag play rate'),
        DeclareLaunchArgument('play_delay', default_value='5.0',
                              description='s between starting the localizer and playing'),
        OpaqueFunction(function=_setup),
    ])