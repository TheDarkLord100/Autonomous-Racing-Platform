#!/usr/bin/env python3
"""
Picks a target angle within a found gap, then blends in a corridor-
centering correction that fades out as the demanded turn gets sharper,
and clamps to the physical steering limit. Also holds the Tweak 3
side-safety probe. Pure data in / data out -- no ROS node dependency.
"""

import math

import numpy as np


class TargetSelectorConfig:

    def __init__(self):
        self.center_gain = 0.35                # (rad per meter of left/right wall imbalance)
        self.center_blend_angle_deg = 15.0     # FTG angle (deg) where centering fades to zero
        self.side_probe_window_deg = 8.0       # (deg) window around +-90 deg for wall distance
        self.max_steering_angle_deg = 24.0     # physical steering limit (deg)
        self.max_range = 4.0                   # fallback when a side probe has no reading (m)
        self.side_safety_distance = 0.35       # (m) below this on the turning side -> go straight
        self.side_safety_probe_from_deg = 90.0   # start of the safety probe window (deg)
        self.side_safety_probe_to_deg = 130.0    # end of the safety probe window (deg)


class TargetSelector:

    def __init__(self, config: TargetSelectorConfig):
        self.config = config

    def best_point_in_gap(self, ranges, gap_start, gap_end):
        """
        Index of the farthest point within [gap_start, gap_end), tie-broken
        toward the middle of the beams at max range.
        """
        segment = ranges[gap_start:gap_end]
        candidates = np.flatnonzero(segment == segment.max())
        return gap_start + int(candidates[candidates.size // 2])

    def compute_wall_distance(self, ranges, angles, target_angle, window_rad):
        """
        Average range in a small angular window around target_angle -- used
        to measure true left/right wall distance from the unmutated scan.
        """
        sel = (ranges > 0.0) & (
            np.abs(angles.astype(np.float64) - target_angle) <= window_rad)
        count = int(np.count_nonzero(sel))
        if count == 0:
            return self.config.max_range  # no reading in window -> assume open
        return float(ranges[sel].astype(np.float64).sum()) / count

    def compute_final_angle(self, ftg_angle, left_dist, right_dist):
        """
        Blend the raw FTG target angle with a centering correction derived
        from left/right wall distance, then clamp to the steering limit.
        The centering term fades to zero as ftg_angle approaches
        center_blend_angle_deg, so aggressive turns are left entirely to
        the gap target.
        """
        blend_angle_rad = math.radians(self.config.center_blend_angle_deg)
        center_weight = 1.0 - min(abs(ftg_angle) / blend_angle_rad, 1.0)
        centering_correction = self.config.center_gain * (left_dist - right_dist)
        final_angle = ftg_angle + center_weight * centering_correction

        max_rad = self.max_steering_rad()
        return max(-max_rad, min(max_rad, final_angle))

    def side_probe_window_rad(self):
        return math.radians(self.config.side_probe_window_deg)

    def max_steering_rad(self):
        return math.radians(self.config.max_steering_angle_deg)

    def side_too_close(self, ranges, angles, turn_direction_sign):
        """
        Tweak 3 (lecture): scan beyond +-90 deg on whichever side the car
        is turning into. If anything reads below side_safety_distance
        there, the turn is unsafe regardless of what the gap target says.
        """
        from_rad = math.radians(self.config.side_safety_probe_from_deg)
        to_rad = math.radians(self.config.side_safety_probe_to_deg)

        r = ranges.astype(np.float64)
        a = angles.astype(np.float64) * turn_direction_sign
        hit = ((r > 0.0) & (a >= from_rad) & (a <= to_rad)
               & (r < self.config.side_safety_distance))
        return bool(np.any(hit))
