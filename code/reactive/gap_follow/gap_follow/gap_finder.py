#!/usr/bin/env python3
"""
Finds the largest contiguous run of free-space beams in a processed
scan, trims its edges to keep the chosen target away from the walls
that bound it, and measures its true physical width. Pure data in /
data out -- no ROS node dependency.
"""

import math

import numpy as np


class GapFinderConfig:
    """Settings for GapFinder."""

    def __init__(self):
        """Set the default gap-finding settings."""
        self.min_gap_points = 5         # ignore gaps smaller than this many beams
        self.gap_edge_margin = 0.10     # (m) extra clearance trimmed off each gap edge
        self.car_half_width = 0.20      # half the car's width, plus safety pad (m)


class GapFinder:
    """Find the largest gap in the processed scan and trim its edges for the car width."""

    def __init__(self, config: GapFinderConfig):
        """Store the configuration."""
        self.config = config

    def find_max_gap(self, ranges, mask):
        """
        Find the largest contiguous run of (ranges > 0 and mask) beams.

        Returns (gap_start, gap_end) with gap_end exclusive, or None if no
        gap reaches min_gap_points. Ties go to the first (lowest-index) run.
        """
        free = (ranges > 0.0) & mask
        padded = np.concatenate(([0], free.astype(np.int8), [0]))
        edges = np.diff(padded)
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)
        if starts.size == 0:
            return None

        lengths = ends - starts
        best = int(np.argmax(lengths))  # first max, same as strict '>' scan
        if lengths[best] < self.config.min_gap_points:
            return None
        return int(starts[best]), int(ends[best])

    def trim_gap_edges(self, ranges, gap_start, gap_end, angle_increment):
        """
        Erode both ends of [gap_start, gap_end) so a later target pick can't
        land right on the gap's boundary. Each side is sized from the range
        at its own boundary (in a hairpin the two walls of a gap are often
        at very different distances). Trims are scaled down proportionally
        -- never reverted to zero -- if together they'd eat past
        min_gap_points.

        Returns the trimmed (gap_start, gap_end).
        """
        gap_len = gap_end - gap_start
        max_total_trim = max(0, gap_len - self.config.min_gap_points)
        if max_total_trim == 0:
            return gap_start, gap_end

        clearance = self.config.car_half_width + self.config.gap_edge_margin

        def trim_for_dist(dist):
            """Return the number of beams to trim at an edge at distance ``dist``."""
            if np.float32(dist) <= np.float32(0.05):
                dist = 0.5  # fallback if that edge is also blanked
            half_angle = math.atan2(clearance, float(dist))
            return int(half_angle / angle_increment)

        trim_start = trim_for_dist(ranges[gap_start])
        trim_end = trim_for_dist(ranges[gap_end - 1])

        total_trim = trim_start + trim_end
        if total_trim > max_total_trim:
            scale = max_total_trim / total_trim
            trim_start = int(trim_start * scale)
            trim_end = int(trim_end * scale)

        return gap_start + trim_start, gap_end - trim_end

    def compute_gap_width(self, ranges, angles, start, end):
        """
        Physical (x, y) width of the gap's mouth, from the two boundary
        beams' polar coordinates -- not their angular separation, since a
        narrow needle-shaped opening can span a wide angle while still
        being tight.
        """
        r1, a1 = float(ranges[start]), float(angles[start])
        r2, a2 = float(ranges[end - 1]), float(angles[end - 1])
        x1, y1 = r1 * math.cos(a1), r1 * math.sin(a1)
        x2, y2 = r2 * math.cos(a2), r2 * math.sin(a2)
        return math.hypot(x2 - x1, y2 - y1)
