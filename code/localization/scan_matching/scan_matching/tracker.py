"""Pose bookkeeping around PL-ICP: keyframes, odometry initial guess,
odometry fallback, resets. No ROS imports, so it can be tested offline.

Frames
- base: the car (ego_racecar/base_link). Odometry and the output are in base.
- laser: where the scans are measured. Fixed offset laser_in_base.
- ICP works laser-to-laser, so the odometry guess is converted into the
  laser frame and the ICP result back into base:
      laser motion = inv(T_bl) ⊕ base motion ⊕ T_bl
  Skipping this gives a wrong guess whenever the car turns, because the
  laser (ahead of base_link) swings sideways as the car yaws.

Keyframes
- kf_dist_linear = kf_dist_angular = 0: every scan becomes the reference
  for the next one (pure frame-to-frame).
- Otherwise the reference scan is kept until the car has moved more than
  kf_dist_linear [m] or turned more than kf_dist_angular [rad] from it.
  Each new pose is keyframe pose ⊕ match, so error only builds up when the
  keyframe changes, not on every scan.
"""

import bisect
import math
import numpy as np

from .core import Scan, icp, se2_compose, se2_inverse, wrap_angle


def interpolate_pose(times, poses, t):
    """Linear interpolation of [x, y, theta] at time t (angle-aware).
    times: sorted list of floats; poses: matching list of arrays.
    Clamps to the first / last pose outside the buffered range."""
    if not times:
        return None
    k = bisect.bisect_left(times, t)
    if k == 0:
        return np.array(poses[0], dtype=float)
    if k >= len(times):
        return np.array(poses[-1], dtype=float)
    t0, t1 = times[k - 1], times[k]
    a, b = poses[k - 1], poses[k]
    w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
    return np.array([a[0] + w * (b[0] - a[0]),
                     a[1] + w * (b[1] - a[1]),
                     wrap_angle(a[2] + w * wrap_angle(b[2] - a[2]))])


class StepResult:
    def __init__(self):
        self.pose = None            # base pose in map after this scan
        self.base_delta = None      # base motion since the previous scan
        self.icp = None             # IcpResult (None for the first scan)
        self.used_fallback = False  # True: pose came from odometry, not ICP
        self.new_keyframe = False
        self.reason = ''


class ScanMatchTracker:

    def __init__(self, laser_in_base, scan_params=None, icp_params=None,
                 kf_dist_linear=0.0, kf_dist_angular=0.0,
                 max_correction_linear=0.5, max_correction_angular=0.3):
        """laser_in_base: [x, y, yaw] of the laser in the base frame.
        scan_params: kwargs for core.Scan (normal_length, normal_max_gap).
        icp_params: kwargs for core.icp.
        max_correction_*: if ICP moves the pose further than this from the
        odometry guess, the match is treated as wrong and odometry is used.
        """
        self.T_bl = np.array(laser_in_base, dtype=float)
        self.T_lb = se2_inverse(self.T_bl)
        self.scan_params = dict(scan_params or {})
        self.icp_params = dict(icp_params or {})
        self.kf_lin = kf_dist_linear
        self.kf_ang = kf_dist_angular
        self.max_corr_lin = max_correction_linear
        self.max_corr_ang = max_correction_angular
        self.pose = np.zeros(3)         # current base pose in map
        self.kf_scan = None             # reference Scan
        self.kf_pose = None             # base pose in map at the keyframe
        self.kf_odom = None             # odometry pose at the keyframe
        self.prev_pose = None

    def reset(self, map_pose):
        """New absolute pose (e.g. RViz 2D Pose Estimate). Drops the
        reference scan, so the next scan becomes the new keyframe."""
        self.pose = np.array(map_pose, dtype=float)
        self.kf_scan = None
        self.prev_pose = None

    def _set_keyframe(self, scan, odom_pose):
        self.kf_scan = scan
        self.kf_pose = self.pose.copy()
        self.kf_odom = np.array(odom_pose, dtype=float)

    def process(self, ranges, angle_min, angle_increment, range_min,
                range_max, odom_pose):
        """One scan. odom_pose: odometry base pose at the scan's timestamp."""
        out = StepResult()
        scan = Scan(ranges, angle_min, angle_increment, range_min, range_max,
                    **self.scan_params)
        if self.kf_scan is None:
            self._set_keyframe(scan, odom_pose)
            self.prev_pose = self.pose.copy()
            out.pose = self.pose.copy()
            out.base_delta = np.zeros(3)
            out.new_keyframe = True
            out.reason = 'first scan / reset'
            return out

        # Odometry guess: base motion since the keyframe, then in laser frame.
        odom_delta = se2_compose(se2_inverse(self.kf_odom), odom_pose)
        guess = se2_compose(se2_compose(self.T_lb, odom_delta), self.T_bl)

        res = icp(self.kf_scan, scan, guess, **self.icp_params)
        out.icp = res
        delta = None
        if res.valid:
            corr = se2_compose(se2_inverse(guess), res.pose)
            if (math.hypot(corr[0], corr[1]) > self.max_corr_lin
                    or abs(corr[2]) > self.max_corr_ang):
                out.reason = 'correction too large'
            else:
                delta = se2_compose(se2_compose(self.T_bl, res.pose), self.T_lb)
        else:
            out.reason = res.reason

        if delta is None:
            delta = odom_delta
            out.used_fallback = True

        self.pose = se2_compose(self.kf_pose, delta)
        out.base_delta = se2_compose(se2_inverse(self.prev_pose), self.pose)
        self.prev_pose = self.pose.copy()
        out.pose = self.pose.copy()

        # New keyframe: always after a fallback (the old reference did not
        # match), otherwise when far enough from the current one.
        if (out.used_fallback
                or math.hypot(delta[0], delta[1]) >= self.kf_lin
                or abs(delta[2]) >= self.kf_ang):
            self._set_keyframe(scan, odom_pose)
            out.new_keyframe = True
        return out