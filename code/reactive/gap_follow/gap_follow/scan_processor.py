#!/usr/bin/env python3
"""
Converts a raw LiDAR scan into a free-space representation the gap
finder can search safely: clips/cleans invalid readings, restricts to
the forward FOV, extends disparities so corners read as solid rather
than leaving a false gap right past their tip, and zeroes a safety
bubble around the single closest obstacle.

Pure data in / data out -- no ROS node dependency, so this is
independently unit-testable outside a running ROS graph.

Arrays are float32 throughout to match the LaserScan message (and the
original C++ implementation's std::vector<float>).
"""

import math

import numpy as np


class ScanProcessorConfig:
    """Tunables for turning a raw scan into free space (meters / degrees)."""

    def __init__(self):
        """Set the default scan-processing settings."""
        self.max_range = 4.0            # clip lidar beyond this (m)
        self.fov_deg = 180.0            # field of view considered, centered on front
        self.bubble_radius = 0.35       # safety bubble around closest point (m)
        self.disparity_threshold = 0.3  # (m) beam-to-beam jump that triggers disparity extension
        self.car_half_width = 0.20      # half the car's width, plus safety pad (m)


class ScanProcessor:
    """Clip and crop the scan, extend disparities and apply the safety bubble."""

    def __init__(self, config: ScanProcessorConfig):
        """Store the configuration."""
        self.config = config

    def preprocess(self, raw_ranges, angle_min, angle_increment):
        """
        Clip/clean raw ranges, compute per-beam angles, mark in-FOV beams.

        Returns (ranges, angles, mask): float32, float32, bool arrays.
        """
        max_range = np.float32(self.config.max_range)

        ranges = np.array(raw_ranges, dtype=np.float32)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = max_range
        ranges = np.clip(ranges, np.float32(0.0), max_range)

        n = ranges.shape[0]
        angles = (np.float32(angle_min)
                  + np.arange(n, dtype=np.float32) * np.float32(angle_increment))

        fov_rad_half = math.radians(self.config.fov_deg) / 2.0
        mask = np.abs(angles).astype(np.float64) <= fov_rad_half

        return ranges, angles, mask

    def extend_disparities(self, ranges, mask, angle_increment):
        """
        In-place: extend the nearer of each disparate beam pair outward by
        however many beams the car's half-width would sweep at that range,
        so corners/alcoves read as solid instead of leaving a false gap.

        Disparities are detected on the ORIGINAL ranges and extensions are
        written to a separate output. Detecting on the array being written
        lets an upward (increasing-index) extension create a fresh edge one
        step ahead of the loop, which then extends again -- cascading every
        obstacle's distance across the whole FOV to its left.
        """
        orig = ranges.tolist()
        r = list(orig)
        m = mask.tolist()
        n = len(r)
        threshold = self.config.disparity_threshold
        half_width = self.config.car_half_width

        for i in range(n - 1):
            if not m[i] or not m[i + 1]:
                continue

            r1 = orig[i]
            r2 = orig[i + 1]
            if r1 <= 0.0 or r2 <= 0.0:
                continue  # already blanked out

            diff = r2 - r1
            if abs(diff) < threshold:
                continue

            near_is_left = diff > 0.0  # r1 < r2 -> the near point is at i
            near_dist = r1 if near_is_left else r2
            near_idx = i if near_is_left else i + 1

            half_angle = math.atan2(half_width, near_dist)
            extend_pts = int(half_angle / angle_increment) + 1

            if near_is_left:
                hi = min(n, near_idx + extend_pts + 1)
                for k in range(near_idx + 1, hi):
                    if r[k] > near_dist:
                        r[k] = near_dist
            else:
                lo = max(0, near_idx - extend_pts)
                for k in range(lo, near_idx):
                    if r[k] > near_dist:
                        r[k] = near_dist

        ranges[:] = r

    def find_closest_point(self, ranges, mask):
        """
        Return (index, distance) of the closest valid in-FOV point.

        Zero ranges are invalid (NaN returns, or real-LiDAR dropouts) and
        are skipped -- otherwise a single bad beam reads as an obstacle at
        0 m and triggers a 180-degree safety bubble.
        """
        valid = mask & (ranges > 0.0)
        masked = np.where(valid, ranges, np.float32(np.inf))
        idx = int(np.argmin(masked))
        return idx, float(masked[idx])

    def zero_bubble(self, ranges, closest_idx, closest_dist, angle_increment):
        """In-place: zero a safety bubble of physical radius bubble_radius."""
        if np.float32(closest_dist) <= np.float32(0.05):
            half_angle = math.pi / 2.0  # obstacle basically on top of us -> wipe a wide chunk
        else:
            half_angle = math.atan2(self.config.bubble_radius, closest_dist)
        bubble_pts = int(half_angle / angle_increment)

        lo = max(0, closest_idx - bubble_pts)
        hi = min(ranges.shape[0], closest_idx + bubble_pts + 1)
        ranges[lo:hi] = 0.0