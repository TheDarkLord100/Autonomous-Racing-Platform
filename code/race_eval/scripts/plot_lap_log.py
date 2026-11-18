#!/usr/bin/env python3
"""
Plot lap_logger CSVs against path arc length s: lateral error, speed (plan / actual /
command), curvature (plan vs actual = yaw rate / v), steering command and the estimated
gap between the car's side and each wall (logs that have gap columns only).
Several logs are overlaid in different colours, e.g. a crashing run vs a clean one.
Usage:
  python3 plot_lap_log.py ~/race_logs/run_a.csv ~/race_logs/run_b.csv
  python3 plot_lap_log.py run_a.csv --s-range 95 125          # zoom on one corner
  python3 plot_lap_log.py run_a.csv --lap 2                    # one lap only
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load(path):
    """Load a lap-logger CSV as a dict of column arrays."""
    d = np.genfromtxt(path, delimiter=',', names=True)
    return {k: d[k] for k in d.dtype.names}


def segments(d, lap, s_range):
    """Split rows into continuous per-lap pieces (so lines don't jump at the s wrap)."""
    keep = np.ones(len(d['s']), bool)
    if lap is not None:
        keep &= d['lap'] == lap
    if s_range is not None:
        keep &= (d['s'] >= s_range[0]) & (d['s'] <= s_range[1])
    idx = np.flatnonzero(keep)
    if len(idx) == 0:
        return []
    breaks = np.flatnonzero((np.diff(idx) > 1) | (np.abs(np.diff(d['s'][idx])) > 5.0)) + 1
    return np.split(idx, breaks)


def main():
    """Parse arguments and plot the given logs against arc length."""
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--lap', type=int, default=None, help='only this lap (0 = out-lap)')
    ap.add_argument('--s-range', type=float, nargs=2, default=None, metavar=('S0', 'S1'))
    ap.add_argument('--save', default=None, help='save the figure to this path')
    a = ap.parse_args()

    fig, ax = plt.subplots(5, 1, figsize=(16, 14), sharex=True, layout='constrained')
    colours = plt.rcParams['axes.prop_cycle'].by_key()['color']
    plan_drawn = False

    for k, path in enumerate(a.logs):
        d = load(path)
        c = colours[k % len(colours)]
        name = Path(path).stem
        for j, seg in enumerate(segments(d, a.lap, a.s_range)):
            lbl = name if j == 0 else None
            s = d['s'][seg]
            ax[0].plot(s, d['e_lat'][seg], c=c, lw=1, label=lbl)
            ax[1].plot(s, d['v_act'][seg], c=c, lw=1.2, label=f'{name}: actual' if j == 0 else None)
            ax[1].plot(s, d['v_cmd'][seg], c=c, lw=0.8, ls=':',
                       label=f'{name}: command' if j == 0 else None)
            ax[2].plot(s, d['kappa_act'][seg], c=c, lw=1, label=f'{name}: actual' if j == 0 else None)
            ax[3].plot(s, d['steer_cmd'][seg], c=c, lw=1, label=lbl)
            if 'gap_left' in d:
                ax[4].plot(s, d['gap_left'][seg], c=c, lw=1,
                           label=f'{name}: left' if j == 0 else None)
                ax[4].plot(s, d['gap_right'][seg], c=c, lw=1, ls='--',
                           label=f'{name}: right' if j == 0 else None)
            if not plan_drawn:                                    # plan from the first log
                ax[1].plot(s, d['v_plan'][seg], c='k', lw=1, ls='--',
                           label='plan' if j == 0 else None)
                ax[2].plot(s, d['kappa_plan'][seg], c='k', lw=1, ls='--',
                           label='plan' if j == 0 else None)
        plan_drawn = True

        last = len(d['s']) - 1                                   # where each run ended
        ax[0].plot(d['s'][last], d['e_lat'][last], 'x', c=c, ms=10)

    ax[0].axhline(0, c='k', lw=0.6)
    ax[0].set_ylabel('e_lat [m]  (+ = left of path)')
    ax[1].set_ylabel('v [m/s]')
    ax[2].set_ylabel('κ [1/m]  (+ = left)')
    ax[3].set_ylabel('steer cmd [rad]')
    ax[4].axhline(0, c='r', lw=0.8)
    ax[4].set_ylabel('wall gap [m]  (< 0 = contact)')
    ax[4].set_xlabel('s [m]')
    for x in ax:
        x.grid(alpha=0.3)
        x.legend(loc='upper right', fontsize=8)
    if a.s_range:
        ax[4].set_xlim(*a.s_range)
    fig.suptitle('lap log: tracking vs plan  (x = last sample of each log)')
    if a.save:
        fig.savefig(a.save, dpi=150)
        print(f'saved {a.save}')
    plt.show()


if __name__ == '__main__':
    main()