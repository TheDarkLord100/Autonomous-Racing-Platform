"""
corridor.py — geometry shared by every local planner (no ROS imports).

Frames
    map       : raceline, pose
    base_link : car (rear axle in f1tenth_gym), x forward, y left
    laser     : scan frame; its pose in base_link comes from TF

Everything the planners check against is the LIVE SCAN, never the static map:
the map is the thing we don't trust. Map widths (w_left_m / w_right_m) are only
a fallback for lateral bounds where the scan has no returns.
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


# --------------------------------------------------------------------------
# Raceline (closed loop, map frame)
# --------------------------------------------------------------------------
class Raceline:
    """Arrays built from race_msgs/WaypointArray."""

    def __init__(self, s, x, y, psi, kappa, vx, w_left, w_right):
        """Store the raceline columns as arrays, with s shifted to start at 0."""
        self.s = np.asarray(s, float) - float(s[0])
        self.xy = np.column_stack((x, y)).astype(float)
        self.psi = np.asarray(psi, float)
        self.kappa = np.asarray(kappa, float)
        self.vx = np.asarray(vx, float)
        self.w_left = np.asarray(w_left, float)
        self.w_right = np.asarray(w_right, float)
        self.n = len(self.s)
        self.length = self.s[-1] + float(np.linalg.norm(self.xy[0] - self.xy[-1]))
        self.mean_ds = self.length / self.n

    @classmethod
    def from_msg(cls, msg):
        """Build a Raceline from a race_msgs/WaypointArray."""
        w = msg.waypoints

        def col(field):
            """Return one waypoint field as a list."""
            return [getattr(p, field) for p in w]
        return cls(col('s_m'), col('x_m'), col('y_m'), col('psi_rad'),
                   col('kappa_radpm'), col('vx_mps'), col('w_left_m'), col('w_right_m'))

    def nearest_index(self, p_xy, last_idx=None, back_m=1.0, fwd_m=5.0,
                      relocalise_dist=2.0) -> int:
        """Windowed search around last_idx (so the hairpin can't make the index
        jump across the track); full search on the first call or if lost."""
        p = np.asarray(p_xy, float)
        if last_idx is not None:
            back = int(np.ceil(back_m / self.mean_ds))
            fwd = int(np.ceil(fwd_m / self.mean_ds))
            idx = (last_idx + np.arange(-back, fwd + 1)) % self.n
            d = np.linalg.norm(self.xy[idx] - p, axis=1)
            k = int(np.argmin(d))
            if d[k] < relocalise_dist:
                return int(idx[k])
        return int(np.argmin(np.linalg.norm(self.xy - p, axis=1)))

    def segment_ahead(self, start_idx: int, horizon: float):
        """Indices from start_idx forward over `horizon` metres (wraps the
        start line) and their arc length measured from start_idx (monotonic)."""
        order = (start_idx + np.arange(self.n)) % self.n
        rel_s = (self.s[order] - self.s[start_idx]) % self.length
        k = max(int(np.searchsorted(rel_s, horizon, side='right')), 2)
        return order[:k], rel_s[:k]


# --------------------------------------------------------------------------
# Frame transforms
# --------------------------------------------------------------------------
def map_to_car(pts_map, pose):
    """p_car = R(-yaw) (p_map - [x, y]);  pose = (x, y, yaw) of base_link in map."""
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    d = np.asarray(pts_map, float) - np.array([x, y])
    return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))


def car_to_map(pts_car, pose):
    """p_map = R(yaw) p_car + [x, y]."""
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    p = np.asarray(pts_car, float)
    return np.column_stack((x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]))


def scan_to_car(ranges, angle_min, angle_increment, range_min, range_max, laser_in_base):
    """LaserScan -> (M, 2) points in base_link. Drops inf / NaN / out-of-range beams."""
    r = np.asarray(ranges, float)
    ang = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    px, py = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
    lx, ly, lyaw = laser_in_base
    c, s = np.cos(lyaw), np.sin(lyaw)
    return np.column_stack((lx + c * px - s * py, ly + s * px + c * py))


class Obstacles:
    """Scan points in base_link + KD-tree. Built once per cycle, shared by
    the gate and both planners so they all see exactly the same world."""

    def __init__(self, pts_car):
        """Store the scan points (car frame) and build a KD-tree on them."""
        self.pts = np.asarray(pts_car, float).reshape(-1, 2)
        self.tree = cKDTree(self.pts) if len(self.pts) else None

    def clearance(self, pts):
        """Return the distance from each point to the nearest scan point (inf if none)."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        if self.tree is None:
            return np.full(len(pts), np.inf)
        d, _ = self.tree.query(pts)
        return d


def required_clearance(pts, threshold, near_threshold, near_radius):
    """Clearance each path point must keep. Within `near_radius` of the car the
    requirement drops to `near_threshold` (just no contact): if the car is already
    inside the margin, a path that leads OUT of it must still count as feasible."""
    r = np.linalg.norm(np.asarray(pts, float).reshape(-1, 2), axis=1)
    return np.where(r < near_radius, near_threshold, threshold)


# --------------------------------------------------------------------------
# Reference path: the raceline segment ahead, expressed in base_link
# --------------------------------------------------------------------------
class RefPath:
    """Raceline segment ahead of the car, expressed in the car frame (base_link)."""

    def __init__(self, raceline: Raceline, seg_idx, seg_rel_s, pose):
        """Transform the selected raceline points and headings into the car frame."""
        self.idx = np.asarray(seg_idx)
        self.s = np.asarray(seg_rel_s, float)
        self.xy = map_to_car(raceline.xy[self.idx], pose)
        psi = np.unwrap(raceline.psi[self.idx] - pose[2])
        # unwrap only fixes jumps AFTER the first point; bring the first into [-pi, pi]
        # too (car yaw and raceline psi each live in [-pi, pi], their difference doesn't)
        self.psi = psi - 2.0 * np.pi * np.round(psi[0] / (2.0 * np.pi))
        self.vx = raceline.vx[self.idx]
        self.w_left = raceline.w_left[self.idx]
        self.w_right = raceline.w_right[self.idx]

    @property
    def length(self):
        """Return the arc length of the segment."""
        return float(self.s[-1])

    def at(self, s):
        """Interpolated (x, y, psi) at arc length(s) s (clipped to the segment)."""
        s = np.clip(s, self.s[0], self.s[-1])
        return (np.interp(s, self.s, self.xy[:, 0]),
                np.interp(s, self.s, self.xy[:, 1]),
                np.interp(s, self.s, self.psi))

    def to_cartesian(self, s, d):
        """Frenet (s, d) -> base_link xy.  d > 0 = left of the raceline."""
        x, y, psi = self.at(s)
        return np.stack((x - d * np.sin(psi), y + d * np.cos(psi)), axis=-1)

    def car_frenet_state(self):
        """Car (origin of base_link, heading 0) in this Frenet frame:
        lateral offset d0 and its slope dd/ds = tan(heading error)."""
        x0, y0 = self.xy[0]
        psi0 = self.psi[0]
        d0 = -x0 * (-np.sin(psi0)) - y0 * np.cos(psi0)   # (car - ref) . normal
        dd0 = np.tan(np.clip(-psi0, -1.2, 1.2))
        return float(d0), float(dd0)

    def v_at(self, s):
        """Return the raceline speed interpolated at arc length ``s``."""
        return np.interp(s, self.s, self.vx)


def lateral_bounds(ref: RefPath, obstacles: Obstacles, stations, band=0.2,
                   max_lookout=2.5):
    """Sampling range across the track at each station s, from the scan: the
    OUTERMOST scan return within `max_lookout` on the left (+d) and right (-d) of
    the raceline, i.e. the walls. Obstacles between the walls (a box on the line)
    deliberately do not shrink the range; the clearance check rejects paths through
    them, while the free space on either side of them stays available.
    Where the scan has no return in that strip, fall back to the map widths.
    Returns (left, right) with left > 0 > right (metres of d)."""
    stations = np.asarray(stations, float)
    x, y, psi = ref.at(stations)
    t = np.stack((np.cos(psi), np.sin(psi)), 1)
    n = np.stack((-np.sin(psi), np.cos(psi)), 1)
    left = np.interp(stations, ref.s, ref.w_left)
    right = -np.interp(stations, ref.s, ref.w_right)
    if len(obstacles.pts):
        rel = obstacles.pts[None, :, :] - np.stack((x, y), 1)[:, None, :]   # (K, M, 2)
        a = np.einsum('kmj,kj->km', rel, t)
        b = np.einsum('kmj,kj->km', rel, n)
        near = np.abs(a) < band
        lb = np.where(near & (b > 0) & (b < max_lookout), b, -np.inf).max(1)
        rb = np.where(near & (b < 0) & (b > -max_lookout), b, np.inf).min(1)
        left = np.where(np.isfinite(lb), lb, left)
        right = np.where(np.isfinite(rb), rb, right)
    return left, right


# --------------------------------------------------------------------------
# The gate
# --------------------------------------------------------------------------
@dataclass
class CorridorResult:
    """Result of the scan-clearance gate on the raceline segment."""

    free: bool                 # True -> raceline ahead is clear
    first_blocked_s: float     # arc length (from car) of first violation, nan if free
    clearance: np.ndarray      # (K,) distance from each raceline point to nearest scan point


def check_corridor(ref: RefPath, obstacles: Obstacles, threshold: float) -> CorridorResult:
    """threshold = car half-width + clearance margin."""
    clearance = obstacles.clearance(ref.xy)
    blocked = clearance < threshold
    free = not bool(blocked.any())
    first = np.nan if free else float(ref.s[int(np.argmax(blocked))])
    return CorridorResult(free, first, clearance)


# --------------------------------------------------------------------------
# Path helpers shared by the planners
# --------------------------------------------------------------------------
def resample(pts, ds=0.1):
    """Resample a polyline at even arc-length spacing. Returns (pts, s)."""
    pts = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate(([True], seg > 1e-6))
    pts = pts[keep]
    s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
    if s[-1] < ds:
        return pts, s
    s_new = np.arange(0.0, s[-1] + 1e-9, ds)
    return np.column_stack((np.interp(s_new, s, pts[:, 0]),
                            np.interp(s_new, s, pts[:, 1]))), s_new


def heading_curvature(pts):
    """Heading and signed curvature of an evenly-ish spaced polyline."""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return np.zeros(len(pts)), np.zeros(len(pts))
    dx, dy = np.gradient(pts[:, 0]), np.gradient(pts[:, 1])
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    return psi, kappa


def speed_profile(v_ref, kappa, s, a_lat_max, a_brake, v_max, v_floor=0.5):
    """v = min(raceline speed, sqrt(a_lat / |kappa|), v_max), then a backward pass
    so the car can actually brake down to every point in time."""
    v = np.minimum(v_ref, v_max)
    v = np.minimum(v, np.sqrt(a_lat_max / np.maximum(np.abs(kappa), 1e-6)))
    for i in range(len(v) - 2, -1, -1):
        ds = s[i + 1] - s[i]
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2.0 * a_brake * ds))
    return np.maximum(v, v_floor)
