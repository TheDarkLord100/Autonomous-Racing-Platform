#!/usr/bin/env python3
"""Run monitor: the only node that talks to the terminal during a bringup run.

Every other node logs to files in the run folder. This one prints one line per
completed lap (lap time, collisions, local-planner mode split, planner compute time)
and every collision as it happens, and writes the same numbers to summary.csv.

Subscribes: /ego_racecar/lap_count (Int32), /ego_racecar/lap_time (Float32),
            /ego_racecar/collision (Bool), /local_planner/mode (String),
            /local_planner/compute_ms (Float32)
"""
import csv
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

MODES = ('pass', 'frenet', 'rrt', 'rrt_raw', 'fallback')


class RunMonitor(Node):
    """ROS 2 node: per-lap run summary from the simulator and local-planner topics.

    Counts collisions and local-planner modes, records compute time, and on each new lap
    logs a summary line and appends a row to ``summary_path``.
    """

    def __init__(self):
        """Declare parameters, open the summary CSV and create the subscribers."""
        super().__init__('run_monitor')
        self.declare_parameter('summary_path', '')
        self.declare_parameter('tag', '')
        path = self.get_parameter('summary_path').value
        self.tag = self.get_parameter('tag').value

        self.lap_count = 0
        self.last_lap_time = float('nan')
        self.collisions = 0
        self.in_collision = False
        self.modes = Counter()
        self.ms = []

        self.file = None
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.file = open(path, 'w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['lap', 'lap_time_s', 'collisions'] +
                                 [f'mode_{m}_pct' for m in MODES] +
                                 ['compute_mean_ms', 'compute_p95_ms', 'compute_max_ms'])
            self.file.flush()

        self.create_subscription(Int32, '/ego_racecar/lap_count', self.on_lap_count, 10)
        self.create_subscription(Float32, '/ego_racecar/lap_time', self.on_lap_time, 10)
        self.create_subscription(Bool, '/ego_racecar/collision', self.on_collision, 10)
        self.create_subscription(String, '/local_planner/mode', self.on_mode, 10)
        self.create_subscription(Float32, '/local_planner/compute_ms', self.on_ms, 10)
        self.get_logger().info(f'run "{self.tag}" running; one line per lap below')

    def on_lap_time(self, msg):
        """Store the last lap time."""
        self.last_lap_time = float(msg.data)

    def on_mode(self, msg):
        """Count one local-planner mode message."""
        self.modes[msg.data] += 1

    def on_ms(self, msg):
        """Record one local-planner compute time."""
        self.ms.append(float(msg.data))

    def on_collision(self, msg):
        """Count a collision on the rising edge of the collision flag."""
        if msg.data and not self.in_collision:
            self.collisions += 1
            self.get_logger().error(f'COLLISION #{self.collisions} (lap {self.lap_count + 1})')
        self.in_collision = bool(msg.data)

    def on_lap_count(self, msg):
        """On a new lap, log the lap summary and write a CSV row."""
        if msg.data <= self.lap_count:
            return
        self.lap_count = msg.data
        total = sum(self.modes.values()) or 1
        pct = {m: 100.0 * self.modes[m] / total for m in MODES}
        ms = np.array(self.ms) if self.ms else np.array([np.nan])
        mean, p95 = float(np.nanmean(ms)), float(np.nanpercentile(ms, 95))
        mx = float(np.nanmax(ms))
        active = ', '.join(f'{m} {pct[m]:.0f}%' for m in MODES if self.modes[m])
        self.get_logger().info(
            f'lap {self.lap_count}: {self.last_lap_time:.2f} s | collisions {self.collisions} | '
            f'modes: {active or "n/a"} | planner {mean:.1f} ms mean, {mx:.1f} ms max')
        if self.file:
            self.writer.writerow([self.lap_count, f'{self.last_lap_time:.3f}', self.collisions] +
                                 [f'{pct[m]:.1f}' for m in MODES] +
                                 [f'{mean:.2f}', f'{p95:.2f}', f'{mx:.2f}'])
            self.file.flush()
        # per-lap numbers: reset the counters
        self.collisions = 0
        self.modes.clear()
        self.ms = []

    def destroy_node(self):
        """Close the summary file and destroy the node."""
        if self.file:
            self.file.close()
        super().destroy_node()


def main(args=None):
    """Run the run monitor node until shutdown."""
    rclpy.init(args=args)
    node = RunMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
