#!/usr/bin/env python3
"""
Pure LiDAR beam-lookup helpers for wall_follow's two-ray geometry.
No ROS/node dependency -- angle_min/angle_increment/num_ranges are
passed in per call rather than held as state, so this is independently
unit-testable outside a running ROS graph, same pattern as
gap_follow_pkg's ScanProcessor.
"""

import math


class ScanProcessor:

    def get_range(self, range_data, angle, angle_min, angle_increment, num_ranges):
        """
        Return the range measurement (meters) at `angle` (radians, in the
        standard LaserScan convention: 0 = straight ahead, + = CCW/left).

        Clips to the scan's actual angular limits and falls back to the
        nearest valid (non-NaN/inf) neighbouring reading if the exact
        index is bad, since a single dropped LiDAR sample shouldn't
        kill the control loop.
        """
        if angle_increment is None or num_ranges is None:
            return 0.0

        idx = int(round((angle - angle_min) / angle_increment))
        idx = max(0, min(num_ranges - 1, idx))

        r = range_data[idx]
        if math.isnan(r) or math.isinf(r):
            for offset in range(1, 20):
                for cand in (idx - offset, idx + offset):
                    if 0 <= cand < num_ranges:
                        rv = range_data[cand]
                        if not (math.isnan(rv) or math.isinf(rv)):
                            return rv
            return 0.0
        return r

    def get_range_median(self, range_data, angle, angle_min, angle_increment,
                          num_ranges, spread_deg=2.0):
        """
        Median of 3 beams centered on `angle` instead of a single beam.
        Cuts single-sample LiDAR noise and reduces the chance that one
        stray beam catches an upcoming curve/gap early.
        """
        spread = math.radians(spread_deg)
        samples = sorted(
            self.get_range(range_data, a, angle_min, angle_increment, num_ranges)
            for a in (angle - spread, angle, angle + spread)
        )
        return samples[1]