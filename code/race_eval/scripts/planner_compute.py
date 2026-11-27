#!/usr/bin/env python3
"""Local-planner compute time per mode, from a run's bag.

  python3 planner_compute.py <run_dir> [<run_dir> ...]

Pairs each /local_planner/mode message with the /local_planner/compute_ms message that
follows it (the node publishes both once per cycle) and prints, per mode: cycle count,
share of cycles, and compute time mean / p95 / max in ms.
"""
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32, String

MODE, MS = '/local_planner/mode', '/local_planner/compute_ms'


def read_pairs(bag_dir):
    """Return a list of (mode, ms) pairs, one per planner cycle."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[MODE, MS]))
    pairs, mode = [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == MODE:
            mode = deserialize_message(data, String).data
        elif mode is not None:
            pairs.append((mode, deserialize_message(data, Float32).data))
            mode = None
    return pairs


def main():
    """Print the per-mode compute table for every run folder given."""
    for run in sys.argv[1:]:
        run = Path(run).expanduser()
        pairs = read_pairs(run / 'bag')
        if not pairs:
            print(f'{run.name}: no {MODE} / {MS} messages')
            continue
        modes = np.array([m for m, _ in pairs])
        ms = np.array([v for _, v in pairs])
        print(f'\n{run.name}  ({len(ms)} cycles)')
        print(f'  {"mode":10s}{"cycles":>8s}{"share":>8s}{"mean ms":>9s}{"p95 ms":>9s}{"max ms":>9s}')
        for m in list(dict.fromkeys(modes)) + ['ALL']:
            v = ms if m == 'ALL' else ms[modes == m]
            print(f'  {m:10s}{len(v):8d}{100 * len(v) / len(ms):7.1f}%{v.mean():9.2f}'
                  f'{np.percentile(v, 95):9.2f}{v.max():9.2f}')


if __name__ == '__main__':
    main()