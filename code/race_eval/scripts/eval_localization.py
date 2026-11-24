#!/usr/bin/env python3
"""Localization comparison: every estimate against simulator ground truth.

  python3 eval_localization.py <replay_run_dir> [<replay_run_dir> ...] [--out DIR]

Each replay run folder (from loc_replay_launch.py) holds bag_out/ (GT odom + estimate
[+ compute_ms]) and bringup_resolved.yaml (stack frame + tf). The method is read from
the folder name (..._locreplay_<method>).

Error = estimate moved into the gym's map frame vs GT interpolated at the estimate's
stamp. Absolute: map distortion is included (Cartographer / PF localize in the
Cartographer map). Laps are cut where GT crosses the start line (pose at first motion).

Latency = (receive time - stamp) of the estimate minus the same for the replayed GT
odometry (median), so the constant wall/sim offset cancels. Same measure for all three.

Writes to --out: summary.csv, laps.csv, overlay.png, error_vs_distance.png
"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np
import yaml

GT_TOPIC = '/ego_racecar/odom'
EST = {'pf': ('/pf/pose', '/pf/compute_ms'),
       'carto': ('/carto/pose', None),
       'scan_matching': ('/scan_match/odom', '/scan_match/compute_ms')}
LABEL = {'pf': 'PF', 'carto': 'Cartographer', 'scan_matching': 'PLICP'}
COLOR = {'pf': '#2a78d6', 'carto': '#eb6834', 'scan_matching': '#1baf7a'}
INK, MUTED, GRID = '#222222', '#6b6b6b', '#e4e4e4'


# ------------------------------------------------------------------ bag reading
def read_bag(path, topics):
    """{topic: [(t_recv_s, msg), ...]} for the topics present in the bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    cls = {t: get_message(types[t]) for t in want}
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    out = {t: [] for t in want}
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        out[topic].append((t_recv * 1e-9, deserialize_message(data, cls[topic])))
    return out


def yaw_of(q):
    """Yaw of a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def poses(msgs):
    """Odometry / PoseStamped list -> (t_recv, stamp, x, y, yaw) arrays and frame id."""
    rows, frame = [], ''
    for t_recv, m in msgs:
        p = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rows.append((t_recv, st, p.position.x, p.position.y, yaw_of(p.orientation)))
        frame = frame or m.header.frame_id
    return np.array(rows), frame


# ------------------------------------------------------------------ maths
def wrap(a):
    """Wrap angles to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def to_map(est, tf):
    """Move (x, y, yaw) columns from a frame whose pose in map is tf=(x, y, yaw)."""
    tx, ty, ta = tf
    c, s = math.cos(ta), math.sin(ta)
    out = est.copy()
    out[:, 2] = tx + c * est[:, 2] - s * est[:, 3]
    out[:, 3] = ty + s * est[:, 2] + c * est[:, 3]
    out[:, 4] = wrap(est[:, 4] + ta)
    return out


def lap_starts(gt, min_lap=50.0, lateral=2.0):
    """Stamps where GT crosses the start line (car pose at first motion), forward only."""
    xy = gt[:, 2:4]
    step = np.hypot(*np.diff(xy, axis=0).T)
    moving = np.nonzero(step > 1e-3)[0]
    i0 = int(moving[0]) if len(moving) else 0
    p0, h0 = xy[i0], np.array([math.cos(gt[i0, 4]), math.sin(gt[i0, 4])])
    rel = xy - p0
    along = rel @ h0
    lat = np.abs(rel[:, 0] * h0[1] - rel[:, 1] * h0[0])
    dist = np.concatenate([[0.0], np.cumsum(step)])
    starts, last = [gt[i0, 1]], dist[i0]
    for i in range(i0 + 1, len(gt)):
        if along[i - 1] < 0.0 <= along[i] and lat[i] < lateral and dist[i] - last > min_lap:
            starts.append(gt[i, 1])
            last = dist[i]
    return np.array(starts), dist


def errors(gt, est):
    """Interpolate GT at the estimate stamps -> stamp, e_pos, e_yaw, GT distance."""
    _, dist = lap_starts(gt)
    t = est[:, 1]
    keep = (t >= gt[0, 1]) & (t <= gt[-1, 1])
    t, e = t[keep], est[keep]
    gx = np.interp(t, gt[:, 1], gt[:, 2])
    gy = np.interp(t, gt[:, 1], gt[:, 3])
    gyaw = wrap(np.interp(t, gt[:, 1], np.unwrap(gt[:, 4])))
    e_pos = np.hypot(e[:, 2] - gx, e[:, 3] - gy)
    e_yaw = np.degrees(np.abs(wrap(e[:, 4] - gyaw)))
    return t, e_pos, e_yaw, np.interp(t, gt[:, 1], dist), keep


def stats(v):
    """mean, p95, max of an array (NaN if empty)."""
    if len(v) == 0:
        return float('nan'), float('nan'), float('nan')
    return float(np.mean(v)), float(np.percentile(v, 95)), float(np.max(v))


def rmse(v):
    """Root mean square."""
    return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float('nan')


# ------------------------------------------------------------------ one run
def evaluate(run_dir):
    """Load one replay run and return its metrics and series."""
    run_dir = Path(run_dir).expanduser()
    method = next((m for m in EST if run_dir.name.endswith(f'locreplay_{m}')), None)
    if method is None:
        raise ValueError(f'{run_dir.name}: expected a folder ending in locreplay_<method>')
    cfg = yaml.safe_load(open(run_dir / 'bringup_resolved.yaml'))
    stack_frame, stack_tf = cfg['stack']['frame'], cfg['stack']['tf']
    est_topic, ms_topic = EST[method]
    msgs = read_bag(run_dir / 'bag_out', [GT_TOPIC, est_topic] + ([ms_topic] if ms_topic else []))
    for topic in (GT_TOPIC, est_topic):
        if not msgs.get(topic):
            raise ValueError(f'{method}: no {topic} messages in {run_dir / "bag_out"}')
    gt, _ = poses(msgs[GT_TOPIC])
    est, frame = poses(msgs[est_topic])
    if frame == stack_frame and stack_frame != 'map':
        est = to_map(est, stack_tf)
    elif frame != 'map':
        raise ValueError(f'{method}: estimate frame {frame!r} is neither map nor {stack_frame!r}')
    compute = np.array([m.data for _, m in msgs.get(ms_topic, [])]) if ms_topic else np.array([])
    return method, gt, est, compute


def summarise(method, gt, est, compute):
    """Overall + per-lap metrics for one localizer."""
    t, e_pos, e_yaw, d, keep = errors(gt, est)
    offset = np.median(gt[:, 0] - gt[:, 1])                  # wall - sim, replayed GT
    latency_ms = 1e3 * ((est[keep, 0] - est[keep, 1]) - offset)
    starts, _ = lap_starts(gt)
    edges = list(starts) + [gt[-1, 1]]
    laps = []
    for k in range(len(edges) - 1):
        m = (t >= edges[k]) & (t < edges[k + 1])
        if not m.any():
            continue
        full = k + 1 < len(starts)
        laps.append({'method': method, 'lap': k + 1, 'complete': full,
                     'rmse_m': rmse(e_pos[m]), 'max_m': float(e_pos[m].max()),
                     'heading_rmse_deg': rmse(e_yaw[m]), 'end_err_m': float(e_pos[m][-1])})
    c_mean, c_p95, c_max = stats(compute)
    l_mean, l_p95, _ = stats(latency_ms)
    row = {'method': method, 'n_estimates': len(t), 'laps_complete': sum(l['complete'] for l in laps),
           'rmse_m': rmse(e_pos), 'max_m': float(e_pos.max()), 'mean_m': float(e_pos.mean()),
           'heading_rmse_deg': rmse(e_yaw), 'compute_mean_ms': c_mean, 'compute_p95_ms': c_p95,
           'compute_max_ms': c_max, 'latency_mean_ms': l_mean, 'latency_p95_ms': l_p95,
           'distance_m': float(d[-1] - d[0])}
    return row, laps, (d - d[0], e_pos, e_yaw), starts


# ------------------------------------------------------------------ plots
def style(ax):
    """Recessive axes: light grid, no top/right spines, muted ticks."""
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=10)


def plot_overlay(runs, path):
    """GT (one lap of each run is the same track) with every estimate on top."""
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 6.5), facecolor='#fcfcfb')
    ax.set_facecolor('#fcfcfb')
    gt = runs[0][1]
    ax.plot(gt[:, 2], gt[:, 3], color=INK, linewidth=3.0, alpha=0.25, label='Ground truth')
    for method, _, est, _ in runs:
        ax.plot(est[:, 2], est[:, 3], color=COLOR[method], linewidth=1.2, label=LABEL[method])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]', color=MUTED)
    ax.set_ylabel('y [m]', color=MUTED)
    style(ax)
    ax.legend(frameon=False, fontsize=10, loc='best')
    ax.set_title('Estimated trajectory vs ground truth (all laps, Spielberg)',
                 color=INK, fontsize=12, loc='left')
    fig.tight_layout()
    fig.savefig(path, dpi=200, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_errors(series, starts_d, path):
    """Position error (log y) and heading error against distance driven."""
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True, facecolor='#fcfcfb')
    for method, (d, e_pos, e_yaw) in series.items():
        a1.plot(d, np.maximum(e_pos, 1e-3), color=COLOR[method], linewidth=1.2, label=LABEL[method])
        a2.plot(d, e_yaw, color=COLOR[method], linewidth=1.2, label=LABEL[method])
    a1.set_yscale('log')
    a1.set_ylabel('position error [m]', color=MUTED)
    a2.set_ylabel('heading error [deg]', color=MUTED)
    a2.set_xlabel('distance driven [m]', color=MUTED)
    for a in (a1, a2):
        a.set_facecolor('#fcfcfb')
        style(a)
        for s in starts_d[1:]:
            a.axvline(s, color=MUTED, linewidth=0.8, linestyle=':')
    a1.legend(frameon=False, fontsize=10, ncol=3, loc='upper left')
    a1.set_title('Localization error vs distance (dotted: lap boundaries)',
                 color=INK, fontsize=12, loc='left')
    fig.tight_layout()
    fig.savefig(path, dpi=200, facecolor=fig.get_facecolor())
    plt.close(fig)


# ------------------------------------------------------------------ main
def write_csv(path, rows):
    """Write a list of dicts to CSV."""
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def report(rows, laps, out):
    """Print the slide table and write the CSVs."""
    write_csv(out / 'summary.csv', rows)
    write_csv(out / 'laps.csv', laps)
    f = lambda v, n=2: '–' if v != v else f'{v:.{n}f}'   # noqa: E731
    print(f'\n{"":24s}' + ''.join(f'{LABEL[r["method"]]:>14s}' for r in rows))
    for key, name, n in [('rmse_m', 'Position RMSE (m)', 3), ('max_m', 'Max error (m)', 3),
                         ('heading_rmse_deg', 'Heading RMSE (deg)', 2),
                         ('compute_mean_ms', 'Compute mean (ms)', 1),
                         ('compute_p95_ms', 'Compute p95 (ms)', 1),
                         ('latency_mean_ms', 'Latency mean (ms)', 1),
                         ('laps_complete', 'Complete laps', 0)]:
        print(f'{name:24s}' + ''.join(f'{f(float(r[key]), n):>14s}' for r in rows))
    print('\nPer lap (RMSE m / end-of-lap error m):')
    for r in rows:
        ls = [l for l in laps if l['method'] == r['method']]
        print(f'  {LABEL[r["method"]]:13s}' + '  '.join(
            f'L{l["lap"]}{"" if l["complete"] else "*"} {l["rmse_m"]:.3f}/{l["end_err_m"]:.3f}'
            for l in ls))
    print(f'\n(* incomplete lap)  written to {out}')


def main():
    """Parse arguments, evaluate every run, write tables and plots."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+', help='replay run folders (..._locreplay_<method>)')
    ap.add_argument('--out', default='loc_eval', help='output folder')
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    runs = [evaluate(r) for r in a.runs]
    order = list(EST)
    runs.sort(key=lambda r: order.index(r[0]))
    rows, laps, series, starts_d = [], [], {}, []
    for method, gt, est, compute in runs:
        row, lp, s, starts = summarise(method, gt, est, compute)
        rows.append(row)
        laps += lp
        series[method] = s
        if not starts_d:   # lap boundaries in distance, from the first run's GT
            _, dist = lap_starts(gt)
            starts_d = list(np.interp(starts, gt[:, 1], dist) - dist[0])
    report(rows, laps, out)
    plot_overlay(runs, out / 'overlay.png')
    plot_errors(series, starts_d, out / 'error_vs_distance.png')


if __name__ == '__main__':
    main()