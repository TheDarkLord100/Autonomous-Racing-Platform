"""Record the input bag for the localization comparison.

  ros2 launch simulation_bringup loc_record_launch.py                  # FTG, 3 m/s
  ros2 launch simulation_bringup loc_record_launch.py max_speed:=2.5

Starts the gym, odom_noise and the bag recorder; the reactive driver starts 3 s later,
so the bag begins with the car at rest on the spawn pose (every localizer's initial
pose). Stop with Ctrl-C after the monitor prints lap 5.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from simulation_bringup.config_resolver import resolve_mapping
from simulation_bringup.launch_helpers import (PKG, bag, gym, monitor, node, odom_noise,
                                               say, start_run)

ARGS = ('controller', 'max_speed', 'tag')


def _setup(context):
    """Resolve the recording config and return the launch actions."""
    share = Path(get_package_share_directory(PKG))
    arg = LaunchConfiguration('config').perform(context)
    config_path = Path(arg) if os.path.isabs(arg) else share / 'config' / arg
    overrides = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    cfg = resolve_mapping(str(config_path), str(share / 'config' / 'params'), overrides)
    d, run, c, P = cfg['derived'], cfg['run'], cfg['controller'], cfg['params']
    if not cfg['mapping']['odom_noise']:
        raise ValueError('loc_record needs mapping.odom_noise: true')
    if not run['bag_topics']:
        raise ValueError('loc_record needs run.bag_topics')

    run_dir, out = start_run(cfg, [
        f'world map : {cfg["world"]["map"]}',
        f'driver    : {c["type"]} at max {c["max_speed"]:g} m/s',
        f'odometry  : {d["odom_in"]} recorded alongside ground truth',
        f'bag       : {" ".join(run["bag_topics"])}',
    ])
    out += [gym(cfg, run_dir, 'mapping.rviz'), odom_noise(cfg), bag(cfg, run_dir)]

    pkg, exe, name, key = d['ctl_node']
    driver = [say(f'starting {c["type"]}; stop with Ctrl-C after lap 5', '32'),
              node(pkg, exe, name, [P[key], d['ctl_overrides']])]
    if run['monitor']:
        driver.append(monitor(cfg, run_dir))
    out.append(TimerAction(period=3.0, actions=driver))
    return out


def generate_launch_description():
    """Declare the overrides and defer the actions to ``_setup``."""
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='loc_record.yaml',
                              description='recording file in config/, or an absolute path'),
        DeclareLaunchArgument('controller', default_value='', description='ftg | pid'),
        DeclareLaunchArgument('max_speed', default_value='', description='m/s'),
        DeclareLaunchArgument('tag', default_value='', description='override run.tag'),
        OpaqueFunction(function=_setup),
    ])