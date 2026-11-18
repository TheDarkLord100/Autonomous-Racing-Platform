"""Turns a bringup scenario (or mapping) yaml into everything the launch files need.

No ROS imports: plain Python, testable offline. The launch files only turn the
resolved dict into launch actions.

Derived values (so they can never disagree with each other):
  - pose topic for planner + controller        <- localization.method
  - every localizer's odometry input           <- localization.odom_noise
  - localizer frames / initial pose / map      <- stack.map, stack.frame, stack.tf, sim spawn
  - whether a second map server is needed      <- stack.map vs world.map
  - static TF map -> stack.frame               <- stack.tf
  - who owns base_link (truth vs belief view)  <- run.tf_source
"""
import copy
import math
import os
import time
from pathlib import Path

import yaml

GT_ODOM = '/ego_racecar/odom'
NOISY_ODOM = '/ego_racecar/odom_noisy'
RELAY_ODOM = '/loc/odom'
ODOM_FRAME = 'odom'
BASE_FRAME = 'ego_racecar/base_link'

METHODS = ('gt', 'pf', 'carto', 'scan_matching')
PLANNERS = ('passthrough', 'frenet', 'rrt')
VIZ = ('rviz', 'foxglove')
TF_SOURCES = ('gym', 'localization')
CONTROLLERS = ('ftg', 'pid')

DEFAULTS = {
    'run': {
        'tag': 'auto',                 # auto: <scenario>_<method>[_noise]_<planner>
        'log_root': '~/f1tenth_simulation_logs',
        'viz': 'rviz',
        'tf_source': 'gym',
        'record_bag': True,
        'bag_topics': [],
        'monitor': True,
        'stage_timeout': 30.0,
    },
    'data_root': '~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/data',
    'world': {'map': ''},
    'stack': {'map': '', 'frame': 'map', 'tf': [0.0, 0.0, 0.0], 'raceline': '',
              'carto_state': ''},  # carto: sidecar yaml with pbstream; '' = find next to map
    'localization': {'method': 'gt', 'odom_noise': False},
    'planning': {'local_planner': 'frenet'},
    'params': {
        'sim': 'sim.yaml',
        'particle_filter': 'pf_params.yaml',
        'scan_matching': 'scan_matching_params.yaml',
        'carto': 'carto_params.yaml',
        'odom_noise': 'odom_noise_params.yaml',
        'local_planner': 'local_planner.yaml',
        'pure_pursuit': 'pure_pursuit.yaml',
    },
}

MAPPING_DEFAULTS = {
    'run': {'tag': 'auto', 'log_root': '~/f1tenth_simulation_logs', 'viz': 'rviz',
            'record_bag': False, 'bag_topics': [], 'monitor': True, 'stage_timeout': 30.0},
    'data_root': '~/f1tenth_sim_ws/src/Autonomous-Racing-Platform/data',
    'world': {'map': ''},
    'mapping': {'map_name': 'Spielberg', 'out_dir': 'Spielberg/carto_new',
                'odom_noise': False},
    'controller': {'type': 'ftg', 'max_speed': 3.0},
    'params': {'sim': 'sim.yaml', 'carto': 'carto_params.yaml',
               'odom_noise': 'odom_noise_params.yaml',
               'gap_follow': 'gap_follow.yaml', 'pid': 'pid_wall.yaml'},
}


# ---------------------------------------------------------------- helpers
def deep_merge(base, over):
    """Return a copy of ``base`` with ``over`` merged in recursively."""
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def expand(p):
    """Expand environment variables and ``~`` in a path."""
    return str(Path(os.path.expandvars(str(p))).expanduser())


def under(root, p):
    """Return ``p`` expanded, joined to ``root`` if it is relative."""
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
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def pose_in_frame(spawn, frame_tf):
    """Gym-frame pose -> pose in a frame whose pose in the gym frame is frame_tf."""
    sx, sy, st = spawn
    tx, ty, ta = frame_tf
    c, s = math.cos(ta), math.sin(ta)
    dx, dy = sx - tx, sy - ty
    return [round(c * dx + s * dy, 6) + 0.0, round(-s * dx + c * dy, 6) + 0.0,
            round(wrap(st - ta), 6) + 0.0]


def read_spawn(sim_params):
    """Return the spawn pose [sx, sy, stheta] from the simulator parameter file."""
    with open(sim_params) as f:
        bridge = yaml.safe_load(f)['bridge']['ros__parameters']
    return [float(bridge['sx']), float(bridge['sy']), float(bridge['stheta'])]


def check_exists(items):
    """Raise FileNotFoundError for the first (name, path) pair whose path does not exist."""
    for name, p in items:
        if not Path(p).exists():
            raise FileNotFoundError(f'{name}: {p} does not exist')


def find_carto_state(stack_map, explicit):
    """Cartographer sidecar (pbstream + origin) for the stack map."""
    if explicit:
        cands = [Path(explicit)]
    else:
        cands = sorted(Path(stack_map).parent.glob('*.yaml'))
    for c in cands:
        try:
            meta = yaml.safe_load(open(c)) or {}
        except Exception:
            continue
        if isinstance(meta, dict) and 'pbstream' in meta:
            pb = c.parent / meta['pbstream']
            if not pb.exists():
                raise FileNotFoundError(f'{c} points to {pb}, which does not exist')
            return str(c), str(pb), [float(v) for v in meta.get('origin', [0, 0, 0])]
    raise FileNotFoundError(
        f'localization.method carto needs a Cartographer sidecar (yaml with "pbstream") '
        f'next to {stack_map}; set stack.carto_state to point at it')


def apply_overrides(cfg, ov, keys):
    """Launch-argument overrides: {key: (section, field)}; empty values are ignored."""
    for k, (section, field) in keys.items():
        v = ov.get(k, '')
        if v == '' or v is None:
            continue
        if isinstance(DEFAULTS.get(section, {}).get(field), bool) or \
                isinstance(MAPPING_DEFAULTS.get(section, {}).get(field), bool):
            v = str(v).lower() in ('1', 'true', 'yes', 'on')
        elif isinstance(MAPPING_DEFAULTS.get(section, {}).get(field), float):
            v = float(v)
        cfg[section][field] = v
    return cfg


SCENARIO_ARGS = {'planner': ('planning', 'local_planner'),
                 'localization': ('localization', 'method'),
                 'odom_noise': ('localization', 'odom_noise'),
                 'tag': ('run', 'tag'),
                 'tf_source': ('run', 'tf_source')}
MAPPING_ARGS = {'controller': ('controller', 'type'),
                'max_speed': ('controller', 'max_speed'),
                'odom_noise': ('mapping', 'odom_noise'),
                'out_dir': ('mapping', 'out_dir'),
                'map_name': ('mapping', 'map_name'),
                'tag': ('run', 'tag')}


# ---------------------------------------------------------------- scenario
def resolve(config_path, params_dir, overrides=None):
    """Resolve a scenario file into the full run configuration.

    Merges the file over the defaults, applies launch-argument overrides, validates the
    choices, resolves paths and fills ``derived`` (topics, frames, spawn, TF settings) and
    ``params`` (per-node parameter files). Raises ValueError or FileNotFoundError on
    invalid input.
    """
    with open(config_path) as f:
        user = yaml.safe_load(f) or {}
    cfg = apply_overrides(deep_merge(DEFAULTS, user), overrides or {}, SCENARIO_ARGS)
    run, stack, loc = cfg['run'], cfg['stack'], cfg['localization']
    method, planner = loc['method'], cfg['planning']['local_planner']

    for name, val, allowed in [('localization.method', method, METHODS),
                               ('planning.local_planner', planner, PLANNERS),
                               ('run.viz', run['viz'], VIZ),
                               ('run.tf_source', run['tf_source'], TF_SOURCES)]:
        if val not in allowed:
            raise ValueError(f'{name} must be one of {allowed}, got {val!r}')
    for key in ('map', 'raceline'):
        if not stack[key]:
            raise ValueError(f'stack.{key} is required')
    if not cfg['world']['map']:
        raise ValueError('world.map is required (what the simulator raycasts against)')
    if method == 'gt' and loc['odom_noise']:
        raise ValueError('odom_noise needs a localizer (pf | carto | scan_matching), not gt')

    # ---- paths
    root = expand(cfg['data_root'])
    cfg['data_root'] = root
    cfg['world']['map'] = under(root, cfg['world']['map'])
    stack['map'] = under(root, stack['map'])
    stack['raceline'] = under(root, stack['raceline'])
    cfg['params'] = {k: under(params_dir, v) for k, v in cfg['params'].items()}
    run['log_root'] = expand(run['log_root'])
    check_exists([('world.map', map_yaml(cfg['world']['map'])),
                  ('stack.map', map_yaml(stack['map'])),
                  ('stack.raceline', stack['raceline'])] +
                 [(f'params.{k}', v) for k, v in cfg['params'].items()])

    if run['tag'] in ('', 'auto'):
        run['tag'] = f'{Path(config_path).stem}_{method}' + \
            ('_noise' if loc['odom_noise'] else '') + f'_{planner}'

    # ---- derived
    frame = stack['frame']
    world_yaml = map_yaml(cfg['world']['map']).resolve()
    stack_yaml = map_yaml(stack['map']).resolve()
    d = {'stack_map_yaml': str(stack_yaml), 'static_tf': frame != 'map',
         'stack_map_server': stack_yaml != world_yaml}
    d['stack_map_topic'] = ('/map' if not d['stack_map_server']
                            else '/stack_map' if frame == 'map' else f'/{frame}')
    if not d['stack_map_server'] and frame != 'map':
        raise ValueError('stack.frame differs from map but stack.map is the world map')

    spawn = read_spawn(cfg['params']['sim'])
    d['spawn'] = spawn
    d['initial_pose'] = pose_in_frame(spawn, stack['tf']) if frame != 'map' else spawn
    d['odom_in'] = NOISY_ODOM if loc['odom_noise'] else GT_ODOM

    # TF: 'localization' = REP 105 tree from the estimate; needs a localizer
    d['tf_note'] = ''
    d['tf_from_localization'] = run['tf_source'] == 'localization'
    if d['tf_from_localization'] and method == 'gt':
        d['tf_from_localization'] = False
        d['tf_note'] = 'tf_source localization needs a localizer; using the gym TF'
    d['sim_overrides'] = {'publish_pose_tf': not d['tf_from_localization']}
    d['rviz_file'] = 'sim_belief.rviz' if d['tf_from_localization'] else 'sim.rviz'
    tf_loc = d['tf_from_localization']

    # per-localizer settings
    d['pose_topic'] = GT_ODOM
    d['ready'] = None                    # (node name, log text) that starts stage 3
    if method == 'pf':
        d['pose_topic'] = '/pf/odom'
        d['pf_overrides'] = {
            'map_topic': d['stack_map_topic'], 'map_frame': frame,
            'odom_topic': d['odom_in'], 'odom_frame': ODOM_FRAME, 'publish_tf': tf_loc,
            'use_initial_pose_param': True, 'initial_pose': d['initial_pose']}
        d['ready'] = ('particle_filter', 'initialized at')
    elif method == 'scan_matching':
        # map-free: chains scan-to-scan motion from the spawn, in the gym frame
        d['pose_topic'] = RELAY_ODOM
        d['sm_overrides'] = {
            'odom_topic': d['odom_in'], 'map_frame': 'map', 'publish_tf': False,
            'initial_pose_x': spawn[0], 'initial_pose_y': spawn[1],
            'initial_pose_theta': spawn[2]}
        d['relay'] = {'input_topic': '/scan_match/odom', 'input_type': 'odometry',
                      'odom_topic': d['odom_in'], 'output_topic': RELAY_ODOM,
                      'publish_tf': tf_loc}
        d['ready'] = ('localization_relay', 'first estimate')
    elif method == 'carto':
        sidecar, pbstream, origin = find_carto_state(stack_yaml, stack['carto_state'])
        if frame != 'map' and max(abs(a - b) for a, b in zip(origin, stack['tf'])) > 1e-3:
            d['tf_note'] += (f' [carto origin {origin} in {Path(sidecar).name} differs from '
                             f'stack.tf {stack["tf"]}; using stack.tf]')
        d['carto'] = {'sidecar': sidecar, 'pbstream': pbstream}
        d['pose_topic'] = RELAY_ODOM
        d['carto_overrides'] = {
            'mode': 'localization', 'origin': [float(v) for v in stack['tf']],
            'map_frame': 'map', 'carto_map_frame': frame,
            'use_initial_pose_param': True, 'initial_pose': spawn}
        d['relay'] = {'input_topic': '/carto/pose', 'input_type': 'pose',
                      'odom_topic': d['odom_in'], 'output_topic': RELAY_ODOM,
                      'publish_tf': tf_loc}
        d['ready'] = ('localization_relay', 'first estimate')

    if not run['bag_topics']:
        run['bag_topics'] = [
            GT_ODOM, d['pose_topic'], '/scan', '/drive', '/global_waypoints',
            '/local_waypoints', '/local_planner/mode', '/local_planner/compute_ms',
            '/ego_racecar/collision', '/ego_racecar/lap_count', '/ego_racecar/lap_time',
            '/tf', '/tf_static'] + \
            {'pf': ['/pf/pose'], 'carto': ['/carto/pose'],
             'scan_matching': ['/scan_match/odom'], 'gt': []}[method] + \
            ([NOISY_ODOM] if loc['odom_noise'] else [])
        run['bag_topics'] = list(dict.fromkeys(run['bag_topics']))
    cfg['derived'] = d
    return cfg


def describe(cfg):
    """Return the summary lines printed at start-up for a resolved scenario."""
    d, s, loc = cfg['derived'], cfg['stack'], cfg['localization']
    names = {'gt': 'GT odometry (perfect pose)', 'pf': 'particle filter',
             'carto': 'Cartographer localization', 'scan_matching': 'scan matching (map-free)'}
    loc_txt = names[loc['method']] + (' + odom noise' if loc['odom_noise'] else '')
    tf_txt = (f'estimate (map -> {s["frame"]} -> odom -> base_link), GT drawn as an arrow'
              if d['tf_from_localization'] else 'ground truth from the gym')
    return [
        f'world map : {map_yaml(cfg["world"]["map"])}',
        f'stack map : {d["stack_map_yaml"]}  (frame {s["frame"]}, topic {d["stack_map_topic"]})',
        f'raceline  : {s["raceline"]}',
        f'localize  : {loc_txt}; planner + controller pose {d["pose_topic"]}',
        f'planner   : {cfg["planning"]["local_planner"]}',
        f'TF        : {tf_txt}' + (f'  [{d["tf_note"].strip()}]' if d['tf_note'] else ''),
        f'spawn     : {d["spawn"]} (gym)  ->  {d["initial_pose"]} ({s["frame"]})',
    ]


# ---------------------------------------------------------------- mapping
def resolve_mapping(config_path, params_dir, overrides=None):
    """Resolve a mapping file into the full run configuration.

    Merges the file over the mapping defaults, applies overrides, validates the controller
    and the world map, and fills ``derived`` and ``params``. Raises ValueError on invalid
    input.
    """
    with open(config_path) as f:
        user = yaml.safe_load(f) or {}
    cfg = apply_overrides(deep_merge(MAPPING_DEFAULTS, user), overrides or {}, MAPPING_ARGS)
    run, m, ctl = cfg['run'], cfg['mapping'], cfg['controller']
    if ctl['type'] not in CONTROLLERS:
        raise ValueError(f'controller.type must be one of {CONTROLLERS}, got {ctl["type"]!r}')
    ctl['max_speed'] = float(ctl['max_speed'])
    if ctl['max_speed'] <= 0:
        raise ValueError('controller.max_speed must be > 0')
    if not cfg['world']['map']:
        raise ValueError('world.map is required')

    root = expand(cfg['data_root'])
    cfg['data_root'] = root
    cfg['world']['map'] = under(root, cfg['world']['map'])
    m['out_dir'] = under(root, m['out_dir'])
    cfg['params'] = {k: under(params_dir, v) for k, v in cfg['params'].items()}
    run['log_root'] = expand(run['log_root'])
    check_exists([('world.map', map_yaml(cfg['world']['map']))] +
                 [(f'params.{k}', v) for k, v in cfg['params'].items()])
    if run['tag'] in ('', 'auto'):
        run['tag'] = f'mapping_{m["map_name"]}_{ctl["type"]}_{ctl["max_speed"]:g}mps' + \
            ('_noise' if m['odom_noise'] else '')

    spawn = read_spawn(cfg['params']['sim'])
    d = {'spawn': spawn, 'odom_in': NOISY_ODOM if m['odom_noise'] else GT_ODOM,
         'sim_overrides': {'publish_pose_tf': True}}
    existing = Path(m['out_dir']) / f'{m["map_name"]}.pbstream'
    d['overwrites'] = existing.exists()
    v = ctl['max_speed']
    if ctl['type'] == 'ftg':
        with open(cfg['params']['gap_follow']) as f:
            base = yaml.safe_load(f)['gap_follow_node']['ros__parameters']
        d['ctl_node'] = ('gap_follow', 'gap_follow_node', 'gap_follow_node', 'gap_follow')
        d['ctl_overrides'] = {'max_speed': v, 'min_speed': min(float(base['min_speed']), v)}
    else:
        with open(cfg['params']['pid']) as f:
            base = yaml.safe_load(f)['pid_node']['ros__parameters']
        d['ctl_node'] = ('pid_wall', 'pid_node', 'pid_node', 'pid')
        d['ctl_overrides'] = {'velocity': v, 'min_velocity': min(float(base['min_velocity']), v)}
    d['carto_overrides'] = {'mode': 'mapping', 'origin': spawn,
                            'map_dir': m['out_dir'], 'map_name': m['map_name']}
    d['image_out'] = str(Path(m['out_dir']) / f'{m["map_name"]}_map')
    cfg['derived'] = d
    return cfg


def describe_mapping(cfg):
    """Return the summary lines printed at start-up for a resolved mapping run."""
    d, m, c = cfg['derived'], cfg['mapping'], cfg['controller']
    return [
        f'world map : {map_yaml(cfg["world"]["map"])}',
        f'driver    : {c["type"]} at max {c["max_speed"]:g} m/s',
        f'odometry  : {d["odom_in"]}',
        f'output    : {m["out_dir"]}/{m["map_name"]}.pbstream + .yaml'
        + ('   (EXISTS: saving will overwrite it)' if d['overwrites'] else ''),
        f'origin    : {d["spawn"]} (car pose at mapping start, gym frame)',
    ]


# ---------------------------------------------------------------- run folder
def make_run_dir(cfg, now=None):
    """Create ``<log_root>/<YYYYmmdd_HHMMSS>_<tag>/ros_logs`` and return the run folder."""
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now))
    run_dir = Path(cfg['run']['log_root']) / f'{stamp}_{cfg["run"]["tag"]}'
    (run_dir / 'ros_logs').mkdir(parents=True, exist_ok=True)
    return run_dir
