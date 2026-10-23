#!/usr/bin/env python3
"""
PID wall-following controller for RoboRacer / F1TENTH -- thin ROS 2
orchestrator. All algorithm logic lives in the pure-logic modules this
node wires together (mirrors gap_follow_pkg's module split):

    ScanProcessor       -- LiDAR beam lookups (single + denoised median)
    WallErrorEstimator  -- two-ray wall geometry -> PID error term
    PIDController       -- P+I+D law, anti-windup, side-sign, clamp
    SteeringShaper       -- rate-limits the final steering command
    WallSpeedPlanner      -- forward-looking speed governor
    MarkerVisualizer      -- rviz debug markers

See WallErrorEstimator for the two-ray geometry derivation and the
steering sign-convention note.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

from pid_wall.scan_processor import ScanProcessor
from pid_wall.wall_error_estimator import WallErrorEstimator, WallErrorEstimatorConfig
from pid_wall.pid_controller import PIDController, PIDControllerConfig
from pid_wall.steering_shaper import SteeringShaper, SteeringShaperConfig
from pid_wall.speed_planner import WallSpeedPlanner, WallSpeedPlannerConfig
from pid_wall.marker_visualizer import MarkerVisualizer, MarkerVisualizerConfig


class PIDNode(Node):
    """Wire up the wall-following pipeline and own the ROS I/O."""

    def __init__(self):
        super().__init__('pid_node')

        # ---- Declare parameters (override via YAML or --ros-args -p) ----
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('wall_side', 'left')
        self.declare_parameter('theta_deg', 50.0)
        self.declare_parameter('lookahead_time', 0.3)
        self.declare_parameter('velocity', 1.0)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('integral_limit', 3.0)
        self.declare_parameter('front_cone_deg', 20.0)
        self.declare_parameter('min_front_distance', 1.2)
        self.declare_parameter('slow_front_distance', 3.0)
        self.declare_parameter('min_velocity', 0.8)
        self.declare_parameter('max_steering_rate', 3.0)
        self.declare_parameter('median_spread_deg', 2.0)
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')

        self.scan_processor = ScanProcessor()
        self._build_modules()
        self.add_post_set_parameters_callback(self._on_param_update)

        # ---- Subscribers / publishers ----
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/pid_wall/steering_marker', 10)
        self.debug_marker_pub = self.create_publisher(
            MarkerArray, '/pid_wall/debug_markers', 10)

        # ---- LiDAR geometry / timing, filled in on first scan ----
        self.angle_min = None
        self.angle_increment = None
        self.num_ranges = None
        self.prev_time = None

        self.get_logger().info(
            f'pid_node up: side={self.error_estimator.config.wall_side}, '
            f'desired_distance={self.error_estimator.config.desired_distance} m, '
            f'kp={self.pid_config.kp}, ki={self.pid_config.ki}, kd={self.pid_config.kd}')

    # ------------------------------------------------------------------
    # Parameters / module wiring
    # ------------------------------------------------------------------
    def _build_modules(self):
        wall_side = self.get_parameter('wall_side').value.lower()
        if wall_side not in ('left', 'right'):
            self.get_logger().warn(f"wall_side='{wall_side}' invalid, defaulting to 'left'")
            wall_side = 'left'

        error_cfg = WallErrorEstimatorConfig()
        error_cfg.wall_side = wall_side
        error_cfg.theta_deg = self.get_parameter('theta_deg').value
        error_cfg.desired_distance = self.get_parameter('desired_distance').value
        error_cfg.lookahead_time = self.get_parameter('lookahead_time').value
        error_cfg.median_spread_deg = self.get_parameter('median_spread_deg').value
        self.error_estimator = WallErrorEstimator(error_cfg, self.scan_processor)

        self.pid_config = PIDControllerConfig()
        self.pid_config.kp = self.get_parameter('kp').value
        self.pid_config.ki = self.get_parameter('ki').value
        self.pid_config.kd = self.get_parameter('kd').value
        self.pid_config.integral_limit = self.get_parameter('integral_limit').value
        self.pid_config.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.pid_config.side_sign = self.error_estimator.side_sign
        # Preserve integral/prev_error state across a live param tweak
        # instead of resetting the controller every time.
        if hasattr(self, 'pid_controller'):
            self.pid_controller.config = self.pid_config
        else:
            self.pid_controller = PIDController(self.pid_config)

        shaper_cfg = SteeringShaperConfig()
        shaper_cfg.max_steering_rate = self.get_parameter('max_steering_rate').value
        if hasattr(self, 'steering_shaper'):
            self.steering_shaper.config = shaper_cfg
        else:
            self.steering_shaper = SteeringShaper(shaper_cfg)

        speed_cfg = WallSpeedPlannerConfig()
        speed_cfg.cruise_velocity = self.get_parameter('velocity').value
        speed_cfg.min_velocity = self.get_parameter('min_velocity').value
        speed_cfg.front_cone_deg = self.get_parameter('front_cone_deg').value
        speed_cfg.min_front_distance = self.get_parameter('min_front_distance').value
        speed_cfg.slow_front_distance = self.get_parameter('slow_front_distance').value
        self.speed_planner = WallSpeedPlanner(speed_cfg, self.scan_processor)

        viz_cfg = MarkerVisualizerConfig()
        viz_cfg.base_frame_id = self.get_parameter('base_frame_id').value
        self.visualizer = MarkerVisualizer(viz_cfg)

    def _on_param_update(self, params):
        self._build_modules()
        self.get_logger().info('Params updated: ' + ', '.join(f'{p.name}={p.value}' for p in params))
    
    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def scan_callback(self, scan_msg: LaserScan):
        self.angle_min = scan_msg.angle_min
        self.angle_increment = scan_msg.angle_increment
        self.num_ranges = len(scan_msg.ranges)

        velocity = self.speed_planner.compute_velocity(
            scan_msg.ranges, self.angle_min, self.angle_increment, self.num_ranges)

        error, alpha, Dt, a, b = self.error_estimator.compute_error(
            scan_msg.ranges, self.angle_min, self.angle_increment, self.num_ranges, velocity)

        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.0 if self.prev_time is None else now - self.prev_time
        self.prev_time = now

        raw_steering = self.pid_controller.compute(error, dt)
        steering_angle = self.steering_shaper.shape(raw_steering, dt)

        self._publish_drive(steering_angle, velocity)
        self._publish_markers(a, b, error, alpha, Dt, steering_angle, velocity,
                               scan_msg.header.stamp)

    def _publish_drive(self, steering_angle, velocity):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def _publish_markers(self, a, b, error, alpha, Dt, steering_angle, velocity, stamp):
        debug_markers = MarkerArray()
        debug_markers.markers.append(
            self.visualizer.make_ray_marker(
                self.error_estimator.a_angle, a, 1, 1.0, 1.0, 0.0, stamp))
        debug_markers.markers.append(
            self.visualizer.make_ray_marker(
                self.error_estimator.b_angle, b, 2, 0.0, 1.0, 1.0, stamp))
        debug_markers.markers.append(
            self.visualizer.make_target_line_marker(
                self.error_estimator.config.wall_side,
                self.error_estimator.config.desired_distance, stamp))
        debug_markers.markers.append(
            self.visualizer.make_status_text_marker(error, alpha, Dt, velocity, stamp))
        self.debug_marker_pub.publish(debug_markers)

        arrow = self.visualizer.make_steering_arrow_marker(
            steering_angle, self.pid_config.max_steering_angle, stamp)
        self.marker_pub.publish(arrow)


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()