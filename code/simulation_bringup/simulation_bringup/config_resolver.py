"""Turns a bringup.yaml scenario into everything the launch file needs.

No ROS imports: this is plain Python so it can be tested offline. The launch file
only turns the resolved dict into launch actions.

Derived values (so they can never disagree with each other):
  - pose topic for planner + controller        <- localization.method
  - PF odom input                              <- localization.odom_noise
  - PF map topic / frame / initial pose        <- stack.map, stack.frame, stack.tf, sim spawn
  - whether a second map server is needed      <- stack.map vs world.map
  - static TF map -> stack.frame               <- stack.tf
"""
import copy
import math
import os
import time
from pathlib import Path

import yaml

DEFAULTS = {
    'run': {
        'tag': 'run',
        'log_root': '~/f1tenth_simulation_logs',
        'viz': 'rviz',                 # rviz | foxglove
        'record_bag': True,
        'bag_topics': [],
        'monitor': True,
        'stage_timeout': 30.0,         # s; warn if a stage is still waiting after this
    },
    'data_root': '~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/data',
    'world': {'map': ''},
    'stack': {'map': '', 'frame': 'map', 'tf': [0.0, 0.0, 0.0], 'raceline': ''},
    'localization': {'method': 'gt', 'odom_noise': False},
    'planning': {'local_planner': 'frenet'},
    'params': {
        'sim': 'sim.yaml',
        'particle_filter': 'pf_params.yaml',
        'odom_noise': 'odom_noise_params.yaml',
        'local_planner': 'local_planner.yaml',
        'pure_pursuit': 'pure_pursuit.yaml',
    },
}

GT_ODOM = '/ego_racecar/odom'
NOISY_ODOM = '/ego_racecar/odom_noisy'
PF_ODOM = '/pf/odom'
METHODS = ('gt', 'pf')
PLANNERS = ('passthrough', 'frenet', 'rrt')
VIZ = ('rviz', 'foxglove')


def deep_merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def expand(p):
    return str(Path(os.path.expandvars(str(p))).expanduser())


def under(root, p):
    """Absolute path: p itself if absolute (or ~), else root/p."""
    p = expand(p)
    return p if os.path.isabs(p) else str(Path(root) / p)


def map_yaml(path):
    """A map given with or without extension -> its .yaml (gym style: X, X.yaml, X_map.yaml)."""
    p = Path(path)
    if p.suffix in ('.yaml', '.yml'):
        return p
    for c in (p.with_suffix('.yaml'), p.parent / f'{p.name}_map.yaml'):
        if c.exists():
            return c
    return p.with_suffix('.yaml')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_in_frame(spawn, frame_tf):
    """Gym-frame pose -> pose in a frame whose pose in the gym frame is frame_tf."""
    sx, sy, st = spawn
    tx, ty, ta = frame_tf
    c, s = math.cos(ta), math.sin(ta)
    dx, dy = sx - tx, sy - ty
    return [round(c * dx + s * dy, 6) + 0.0, round(-s * dx + c * dy, 6) + 0.0,
            round(wrap(st - ta), 6) + 0.0]


def apply_overrides(cfg, overrides):
    """overrides: {'planner': 'rrt', 'tag': 'x', 'localization': 'gt'} from launch args."""
    if overrides.get('planner'):
        cfg['planning']['local_planner'] = overrides['planner']
    if overrides.get('localization'):
        cfg['localization']['method'] = overrides['localization']
    if overrides.get('tag'):
        cfg['run']['tag'] = overrides['tag']
    return cfg


def resolve(config_path, params_dir, overrides=None):
    with open(config_path) as f:
        user = yaml.safe_load(f) or {}
    cfg = apply_overrides(deep_merge(DEFAULTS, user), overrides or {})

    # ---- validate the choices
    run, stack, loc = cfg['run'], cfg['stack'], cfg['localization']
    if loc['method'] not in METHODS:
        raise ValueError(f'localization.method must be one of {METHODS}, got {loc["method"]!r}')
    if cfg['planning']['local_planner'] not in PLANNERS:
        raise ValueError(f'planning.local_planner must be one of {PLANNERS}')
    if run['viz'] not in VIZ:
        raise ValueError(f'run.viz must be one of {VIZ}')
    for key in ('map', 'raceline'):
        if not stack[key]:
            raise ValueError(f'stack.{key} is required')
    if not cfg['world']['map']:
        raise ValueError('world.map is required (what the simulator raycasts against)')

    # ---- paths
    root = expand(cfg['data_root'])
    cfg['data_root'] = root
    cfg['world']['map'] = under(root, cfg['world']['map'])
    stack['map'] = under(root, stack['map'])
    stack['raceline'] = under(root, stack['raceline'])
    cfg['params'] = {k: under(params_dir, v) for k, v in cfg['params'].items()}
    run['log_root'] = expand(run['log_root'])
    for name, p in [('world.map', map_yaml(cfg['world']['map'])),
                    ('stack.map', map_yaml(stack['map'])),
                    ('stack.raceline', Path(stack['raceline']))] + \
            [(f'params.{k}', Path(v)) for k, v in cfg['params'].items()]:
        if not p.exists():
            raise FileNotFoundError(f'{name}: {p} does not exist')

    # ---- derived values
    frame = stack['frame']
    world_yaml = map_yaml(cfg['world']['map']).resolve()
    stack_yaml = map_yaml(stack['map']).resolve()
    d = {}
    d['stack_map_yaml'] = str(stack_yaml)
    d['static_tf'] = frame != 'map'
    # the gym already serves the world map on /map in frame map; only add a server
    # when the stack localizes on a different map
    d['stack_map_server'] = stack_yaml != world_yaml
    d['stack_map_topic'] = ('/map' if not d['stack_map_server']
                            else '/stack_map' if frame == 'map' else f'/{frame}')
    if not d['stack_map_server'] and frame != 'map':
        raise ValueError('stack.frame differs from map but stack.map is the world map')

    with open(cfg['params']['sim']) as f:
        bridge = yaml.safe_load(f)['bridge']['ros__parameters']
    spawn = [float(bridge['sx']), float(bridge['sy']), float(bridge['stheta'])]
    d['spawn'] = spawn
    d['initial_pose'] = pose_in_frame(spawn, stack['tf']) if frame != 'map' else spawn
    d['odom_in'] = NOISY_ODOM if loc['odom_noise'] else GT_ODOM
    d['pose_topic'] = PF_ODOM if loc['method'] == 'pf' else GT_ODOM
    if loc['method'] == 'gt' and loc['odom_noise']:
        raise ValueError('odom_noise only makes sense with localization.method: pf')

    d['pf_overrides'] = {
        'map_topic': d['stack_map_topic'],
        'map_frame': frame,
        'odom_topic': d['odom_in'],
        'use_initial_pose_param': True,
        'initial_pose': d['initial_pose'],
    }
    if not run['bag_topics']:
        run['bag_topics'] = [
            GT_ODOM, PF_ODOM, '/pf/pose', '/scan', '/drive',
            '/global_waypoints', '/local_waypoints',
            '/local_planner/mode', '/local_planner/compute_ms',
            '/ego_racecar/collision', '/ego_racecar/lap_count', '/ego_racecar/lap_time',
            '/tf', '/tf_static',
        ] + ([NOISY_ODOM] if loc['odom_noise'] else [])
    cfg['derived'] = d
    return cfg


def make_run_dir(cfg, now=None):
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now))
    run_dir = Path(cfg['run']['log_root']) / f'{stamp}_{cfg["run"]["tag"]}'
    (run_dir / 'ros_logs').mkdir(parents=True, exist_ok=True)
    return run_dir


def describe(cfg):
    """Short human summary for the console."""
    d, s, loc = cfg['derived'], cfg['stack'], cfg['localization']
    loc_txt = 'GT odometry' if loc['method'] == 'gt' else \
        f'particle filter on {Path(d["stack_map_yaml"]).name}' + \
        (' + odom noise' if loc['odom_noise'] else '')
    return [
        f'world map : {map_yaml(cfg["world"]["map"])}',
        f'stack map : {d["stack_map_yaml"]}  (frame {s["frame"]}, topic {d["stack_map_topic"]})',
        f'raceline  : {s["raceline"]}',
        f'localize  : {loc_txt}; planner+controller pose {d["pose_topic"]}',
        f'planner   : {cfg["planning"]["local_planner"]}',
        f'spawn     : {d["spawn"]} (gym)  ->  {d["initial_pose"]} ({s["frame"]})',
    ]
