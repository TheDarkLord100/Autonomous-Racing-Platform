"""Short history of timestamped odometry poses. No ROS imports.

Lets the filter ask "where was the odometry at the scan's timestamp?" instead
of using whatever odom message arrived last. Poses between samples are
linearly interpolated (shortest angle for theta).
"""

import bisect
from collections import deque

import numpy as np

from .geometry import interpolate


class OdomBuffer:

    def __init__(self, max_age=1.0):
        self.max_age = max_age
        self._t = deque()
        self._p = deque()

    def __len__(self):
        return len(self._t)

    def add(self, t, pose):
        """Append a sample. Out-of-order samples (t <= newest) are dropped."""
        if self._t and t <= self._t[-1]:
            return
        self._t.append(float(t))
        self._p.append(np.asarray(pose, dtype=float))
        while len(self._t) > 2 and self._t[0] < t - self.max_age:
            self._t.popleft()
            self._p.popleft()

    def latest(self):
        return (self._t[-1], self._p[-1]) if self._t else (None, None)

    def pose_at(self, t):
        """Odom pose at time t. Clamped to the oldest/newest sample outside the
        buffered window. Returns (pose, status) with status 'ok', 'after' (t is
        newer than the latest odom) or 'before' (t is older than the buffer);
        (None, 'empty') if no samples."""
        if not self._t:
            return None, 'empty'
        if t >= self._t[-1]:
            return self._p[-1].copy(), ('ok' if t == self._t[-1] else 'after')
        if t <= self._t[0]:
            return self._p[0].copy(), ('ok' if t == self._t[0] else 'before')
        times = list(self._t)
        i = bisect.bisect_right(times, t)          # times[i-1] <= t < times[i]
        t0, t1 = times[i - 1], times[i]
        a = (t - t0) / (t1 - t0)
        return interpolate(self._p[i - 1], self._p[i], a), 'ok'