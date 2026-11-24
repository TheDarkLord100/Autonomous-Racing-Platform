"""Mapping bringup: gym + Cartographer (mapping mode) + a reactive driver.

  ros2 launch simulation_bringup mapping_bringup_launch.py                       # FTG, 3 m/s
  ros2 launch simulation_bringup mapping_bringup_launch.py controller:=pid max_speed:=2.5
  ros2 launch simulation_bringup mapping_bringup_launch.py odom_noise:=true \
      out_dir:=Spielberg/carto_noisy

Order: gym + static TF map -> carto_map -> Cartographer -> driver (once carto_node is up).
Save when the map looks complete (the terminal prints both commands with your paths):
  ros2 service call /carto/save_map std_srvs/srv/Trigger           # pbstream + sidecar
  ros2 run nav2_map_server map_saver_cli -t /carto/map -f <out_dir>/<map_name>_map  # image
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from simulation_bringup.config_resolver import MAPPING_ARGS, describe_mapping, resolve_mapping
from simulation_bringup.launch_helpers import (PKG, after_log, bag, carto_node,
                                               cartographer_node, gym, monitor, node,
                                               odom_noise, say, start_run, static_tf)


def _setup(context):
    """Resolve the mapping config and return the launch actions.

    Starts the simulator, the ``map -> carto_map`` static TF, Cartographer in mapping mode
    (with odom_noise if set) and, once ``carto_node`` is up, the reactive driver.
    """
    share = Path(get_package_share_directory(PKG))
    arg = LaunchConfiguration('config').perform(context)
    config_path = Path(arg) if os.path.isabs(arg) else share / 'config' / arg
    overrides = {k: LaunchConfiguration(k).perform(context) for k in MAPPING_ARGS}
    cfg = resolve_mapping(str(config_path), str(share / 'config' / 'params'), overrides)
    d, run, m, P = cfg['derived'], cfg['run'], cfg['mapping'], cfg['params']
    lua = LaunchConfiguration('lua').perform(context)
    Path(m['out_dir']).mkdir(parents=True, exist_ok=True)

    run_dir, out = start_run(cfg, describe_mapping(cfg))
    out.append(gym(cfg, run_dir, 'mapping.rviz'))
    # the occupancy grid comes out in carto_map, whose origin is the spawn pose
    out.append(static_tf(*d['spawn'], 'map', 'carto_map'))
    if m['odom_noise']:
        out.append(odom_noise(cfg))

    pkg, exe, name, key = d['ctl_node']
    driver = [node(pkg, exe, name, [P[key], d['ctl_overrides']])]
    if run['monitor']:
        driver.append(monitor(cfg, run_dir))
    if run['record_bag']:
        driver.append(bag(cfg, run_dir))
    driver += [
        say('when the map looks complete, save it with:', '32'),
        say('  ros2 service call /carto/save_map std_srvs/srv/Trigger', '32'),
        say(f'  ros2 run nav2_map_server map_saver_cli -t /carto/map -f {d["image_out"]}', '32'),
    ]

    cnode = carto_node(cfg, d['carto_overrides'], lua)
    out += [
        cartographer_node(lua, d['odom_in']),
        node('cartographer_ros', 'cartographer_occupancy_grid_node',
             'cartographer_occupancy_grid_node',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             remappings=[('map', '/carto/map')]),   # the gym's map_server owns /map
        cnode,
    ]
    out += after_log(cnode, 'carto_node', 'carto_node (mapping)', driver,
                     f'Cartographer mapping up -> starting {cfg["controller"]["type"]} at '
                     f'{cfg["controller"]["max_speed"]:g} m/s', float(run['stage_timeout']))
    return out


def generate_launch_description():
    """Declare the mapping overrides and defer the actions to ``_setup``."""
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='mapping.yaml',
                              description='mapping file in config/, or an absolute path'),
        DeclareLaunchArgument('controller', default_value='', description='ftg | pid'),
        DeclareLaunchArgument('max_speed', default_value='', description='m/s'),
        DeclareLaunchArgument('odom_noise', default_value='',
                              description='true: map with drifting odometry'),
        DeclareLaunchArgument('out_dir', default_value='',
                              description='where to save, relative to data_root'),
        DeclareLaunchArgument('map_name', default_value='', description='saved file name'),
        DeclareLaunchArgument('tag', default_value='', description='override run.tag'),
        DeclareLaunchArgument('lua', default_value='carto_mapping.lua',
                              description='Cartographer config in cartographer_slam/config'),
        OpaqueFunction(function=_setup),
    ])
