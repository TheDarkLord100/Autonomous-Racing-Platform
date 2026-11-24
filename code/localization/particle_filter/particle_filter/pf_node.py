"""ROS 2 wrapper around the ROS-free particle filter core.

Subscribes:  map (OccupancyGrid, transient-local), scan (LaserScan),
             odom (Odometry), initialpose (PoseWithCovarianceStamped)
Publishes:   pose_topic      PoseStamped   filter estimate at scan rate (use for evaluation)
             odom_out_topic  Odometry      estimate (+) odometry since last scan, at odom rate
                                           (use for planning/control)
             particles_topic PoseArray     particle cloud (throttled, for RViz)
             TF map->odom    only if publish_tf (off in sim: the bridge already
                             publishes map->base_link and there is no odom frame)
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
import tf2_ros
from std_msgs.msg import Float32

from geometry_msgs.msg import (Pose, PoseArray, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from .geometry import compose, inverse, quat_to_yaw, relative_pose, yaw_to_quat
from .occupancy_map import OccupancyMap
from .motion_model import OdometryMotionModel
from .sensor_model import LikelihoodFieldModel
from .filter import ParticleFilter, RecoveryConfig, ScaleConfig
from .odom_buffer import OdomBuffer


def _stamp_to_sec(stamp):
    """Return a builtin_interfaces/Time as float seconds."""
    return stamp.sec + 1e-9 * stamp.nanosec


def _pose_from_msg(p):
    """Return (x, y, yaw) of a geometry_msgs/Pose as an array."""
    o = p.orientation
    return np.array([p.position.x, p.position.y, quat_to_yaw(o.x, o.y, o.z, o.w)])


def _fill_pose(msg_pose, pose):
    """Write (x, y, yaw) into a geometry_msgs/Pose in place."""
    msg_pose.position.x = float(pose[0])
    msg_pose.position.y = float(pose[1])
    msg_pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quat(pose[2])
    msg_pose.orientation.x = qx
    msg_pose.orientation.y = qy
    msg_pose.orientation.z = qz
    msg_pose.orientation.w = qw


class ParticleFilterNode(Node):
    """ROS 2 node running the particle filter on the map, scan and odometry.

    Publishes the estimate on ``pose_topic`` and ``odom_out_topic``, the particles on
    ``particles_topic`` and, if enabled, the ``map -> odom`` transform.
    """

    def __init__(self):
        """Declare parameters, build the motion model, create publishers, subscribers and TF."""
        super().__init__('particle_filter')

        # ---------------- parameters
        p = self.declare_parameter
        self.scan_topic = p('scan_topic', '/scan').value
        self.odom_topic = p('odom_topic', '/ego_racecar/odom').value
        self.map_topic = p('map_topic', '/map').value
        self.initialpose_topic = p('initialpose_topic', '/initialpose').value
        self.pose_topic = p('pose_topic', '/pf/pose').value
        self.odom_out_topic = p('odom_out_topic', '/pf/odom').value
        self.particles_topic = p('particles_topic', '/pf/particles').value

        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'ego_racecar/base_link').value
        self.publish_tf = p('publish_tf', False).value
        self.publish_on_odom = p('publish_on_odom', True).value

        self.num_particles = p('num_particles', 2000).value
        self.resample_threshold = p('resample_threshold', 0.5).value
        self.update_min_d = p('update_min_d', 0.02).value
        self.update_min_a = p('update_min_a', 0.02).value

        self.alpha_trans_trans = p('motion.alpha_trans_trans', 0.1).value
        self.alpha_trans_rot = p('motion.alpha_trans_rot', 0.05).value
        self.alpha_lat = p('motion.alpha_lat', 0.3).value
        self.alpha_rot_trans = p('motion.alpha_rot_trans', 0.1).value
        self.alpha_rot_rot = p('motion.alpha_rot_rot', 0.1).value
        self.min_sigma_xy = p('motion.min_sigma_xy', 0.005).value
        self.min_sigma_theta = p('motion.min_sigma_theta', 0.005).value

        self.z_hit = p('sensor.z_hit', 0.9).value
        self.z_rand = p('sensor.z_rand', 0.1).value
        self.sigma_hit = p('sensor.sigma_hit', 0.1).value
        self.effective_beams = p('sensor.effective_beams', 20.0).value
        self.num_beams = p('sensor.num_beams', 60).value
        self.max_occ_dist = p('sensor.max_occ_dist', 2.0).value
        self.occupied_threshold = p('sensor.occupied_threshold', 65).value
        self.free_threshold = p('sensor.free_threshold', 25).value
        self.penalize_non_free = p('sensor.penalize_non_free', True).value
        self.laser_offset_fallback = list(p('laser_offset_fallback', [0.275, 0.0, 0.0]).value)

        self.recovery = RecoveryConfig(
            enabled=p('recovery.enabled', True).value,
            alpha_slow=p('recovery.alpha_slow', 0.001).value,
            alpha_fast=p('recovery.alpha_fast', 0.1).value,
            min_inject_ratio=p('recovery.min_inject_ratio', 0.12).value,
            max_inject_ratio=p('recovery.max_inject_ratio', 0.25).value,
            local_std_xy=p('recovery.local_std_xy', 1.0).value,
            local_std_theta=p('recovery.local_std_theta', 0.4).value,
            global_fraction=p('recovery.global_fraction', 0.0).value)

        self.scale_cfg = ScaleConfig(
            enabled=p('odom_scale.enabled', True).value,
            init_std=p('odom_scale.init_std', 0.05).value,
            walk_std_per_m=p('odom_scale.walk_std_per_m', 0.002).value,
            min_scale=p('odom_scale.min', 0.7).value,
            max_scale=p('odom_scale.max', 1.3).value)

        self.odom_buffer_sec = p('odom_buffer_sec', 1.0).value
        self.scan_wait_sec = p('scan_wait_sec', 0.05).value

        self.use_initial_pose_param = p('use_initial_pose_param', False).value
        self.initial_pose = list(p('initial_pose', [0.0, 0.0, 0.0]).value)
        self.initial_std_xy = p('initial_std_xy', 0.25).value
        self.initial_std_theta = p('initial_std_theta', 0.1).value

        self.publish_particles_every = p('publish_particles_every', 2).value
        self.max_viz_particles = p('max_viz_particles', 500).value
        seed = p('random_seed', -1).value

        # ---------------- state
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.motion_model = OdometryMotionModel(
            self.alpha_trans_trans, self.alpha_trans_rot, self.alpha_lat,
            self.alpha_rot_trans, self.alpha_rot_rot,
            self.min_sigma_xy, self.min_sigma_theta, rng=self.rng)
        self.map = None
        self.sensor_model = None
        self.pf = None
        self.pending_init = None          # initial pose received before the map
        self.odom_buffer = OdomBuffer(self.odom_buffer_sec)
        self.filter_odom_pose = None      # odom pose at the last filter step (scan stamp)
        self.pending_scan = None          # scan newer than the latest odom, waiting for it
        self.scan_layout = None
        self.laser_offset_ready = False
        self.offset_attempts = 0
        self.scan_count = 0
        self.stats = self._empty_stats()

        # ---------------- ROS interfaces
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(PoseWithCovarianceStamped, self.initialpose_topic,
                                 self.initialpose_cb, 1)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_out_topic, 10)
        self.particles_pub = self.create_publisher(PoseArray, self.particles_topic, 1)
        self.ms_pub = self.create_publisher(Float32, '/pf/compute_ms', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f'particle_filter up: {self.num_particles} particles, scan={self.scan_topic}, '
            f'odom={self.odom_topic}, waiting for map on {self.map_topic}')

    # ------------------------------------------------------------ callbacks
    def map_cb(self, msg):
        """Build the occupancy map, sensor model and filter from a received map."""
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        origin = _pose_from_msg(info.origin)
        t0 = time.perf_counter()
        self.map = OccupancyMap(data, info.resolution, origin,
                                self.occupied_threshold, self.free_threshold,
                                self.max_occ_dist)
        self.sensor_model = LikelihoodFieldModel(
            self.map, self.z_hit, self.z_rand, self.sigma_hit, self.effective_beams,
            self.num_beams, self.laser_offset_fallback, self.penalize_non_free)
        self.pf = ParticleFilter(self.motion_model, self.sensor_model, self.num_particles,
                                 self.resample_threshold, self.update_min_d,
                                 self.update_min_a, recovery=self.recovery,
                                 occ_map=self.map, scale=self.scale_cfg, rng=self.rng)
        self.scan_layout = None
        self.pending_scan = None
        self.laser_offset_ready = False
        self.offset_attempts = 0
        self.get_logger().info(
            f'map {info.width}x{info.height} @ {info.resolution:.3f} m, distance field '
            f'built in {1e3 * (time.perf_counter() - t0):.0f} ms')

        if self.pending_init is not None:
            self._initialize(self.pending_init)
        elif self.use_initial_pose_param:
            self._initialize(np.array(self.initial_pose, dtype=float))
        else:
            self.get_logger().info('waiting for an initial pose (RViz "2D Pose Estimate")')

    def initialpose_cb(self, msg):
        """Initialize the filter at a received pose, or store it until the map arrives."""
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'initial pose is in "{msg.header.frame_id}", expected "{self.map_frame}"; using as-is')
        pose = _pose_from_msg(msg.pose.pose)
        if self.pf is None:
            self.pending_init = pose
            self.get_logger().info('initial pose stored until the map arrives')
        else:
            self._initialize(pose)

    def _initialize(self, pose):
        """Spread the particles around ``pose`` and anchor it to the newest odometry."""
        self.pf.initialize(pose, self.initial_std_xy, self.initial_std_theta,
                           is_free=self.map.is_free)
        # the pose is "now": anchor it to the newest odom (set at the first scan if none yet)
        self.filter_odom_pose = self.odom_buffer.latest()[1]
        self.pending_scan = None
        self.pending_init = None
        self.get_logger().info(
            f'initialized at x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={pose[2]:.2f}')

    def odom_cb(self, msg):
        """Buffer odometry, release a waiting scan, and publish the extrapolated estimate.

        With ``publish_on_odom`` the estimate at the last scan is moved forward by the
        odometry since that scan and published on every odometry message.
        """
        pose = _pose_from_msg(msg.pose.pose)
        t = _stamp_to_sec(msg.header.stamp)
        self.odom_buffer.add(t, pose)

        # a scan was waiting for odometry that covers its timestamp: now it is
        if self.pending_scan is not None and t >= _stamp_to_sec(self.pending_scan.header.stamp):
            scan, self.pending_scan = self.pending_scan, None
            self._process_scan(scan)

        if (self.publish_on_odom and self.pf is not None and self.pf.initialized
                and self.filter_odom_pose is not None):
            # filter estimate (at the last scan stamp) + scale-corrected odometry since then
            est = self.pf.extrapolate(relative_pose(self.filter_odom_pose, pose))
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.map_frame
            out.child_frame_id = self.base_frame
            _fill_pose(out.pose.pose, est)
            out.pose.covariance = self._covariance6()
            out.twist = msg.twist
            self.odom_pub.publish(out)

    def scan_cb(self, msg):
        """Process a scan now, or hold it until odometry covers its stamp."""
        if self.pf is None or not self.pf.initialized:
            return
        if not self.laser_offset_ready:
            self._lookup_laser_offset(msg.header.frame_id)

        latest_t, _ = self.odom_buffer.latest()
        if latest_t is None:
            self.get_logger().warn('no odometry yet; skipping scan', throttle_duration_sec=2.0)
            return
        t_scan = _stamp_to_sec(msg.header.stamp)
        if 0.0 < t_scan - latest_t <= self.scan_wait_sec:
            # Normal: the scan is stamped a few ms after the newest odom we have.
            # Hold it until the next odom message covers its stamp (<= one odom period).
            if self.pending_scan is not None:
                self._process_scan(self.pending_scan)   # superseded: use it clamped
            self.pending_scan = msg
            self.stats['deferred'] += 1
            return
        self._process_scan(msg)

    def _process_scan(self, msg):
        """Run predict and update for one scan, then publish the estimate and stats."""
        layout = (len(msg.ranges), msg.angle_min, msg.angle_increment)
        if layout != self.scan_layout:
            self.sensor_model.set_scan_geometry(msg.angle_min, msg.angle_increment, len(msg.ranges))
            self.scan_layout = layout

        # odometry exactly at the scan's timestamp (interpolated between odom messages)
        odom_at_scan, status = self.odom_buffer.pose_at(_stamp_to_sec(msg.header.stamp))
        if status == 'after':
            self.stats['late'] += 1     # scan newer than all odom by more than scan_wait_sec
        elif status == 'before':
            self.stats['old'] += 1      # scan older than the odom buffer
        if self.filter_odom_pose is None:
            self.filter_odom_pose = odom_at_scan
        delta = relative_pose(self.filter_odom_pose, odom_at_scan)
        self.filter_odom_pose = odom_at_scan

        t0 = time.perf_counter()
        self.pf.predict(delta)
        updated = self.pf.update(np.asarray(msg.ranges, dtype=float),
                                 msg.range_min, msg.range_max)
        dt = time.perf_counter() - t0

        if updated:
            self._log_stats(dt)
            self.ms_pub.publish(Float32(data=1e3 * dt))

        self._publish_pose(msg.header.stamp)
        self.scan_count += 1
        if self.publish_particles_every > 0 and self.scan_count % self.publish_particles_every == 0:
            self._publish_particles(msg.header.stamp)
        if self.publish_tf:
            self._publish_map_to_odom(msg.header.stamp)

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _empty_stats():
        """Return a fresh statistics dictionary."""
        return {'times': [], 'neff': [], 'resampled': 0, 'injected': 0,
                'inject_events': 0, 'deferred': 0, 'late': 0, 'old': 0}

    def _log_stats(self, dt):
        """Accumulate per-update statistics and log a summary every 100 updates."""
        s = self.stats
        s['times'].append(dt)
        s['neff'].append(self.pf.last_neff / self.pf.n)
        s['resampled'] += int(self.pf.resampled)
        s['injected'] += self.pf.last_injected
        s['inject_events'] += int(self.pf.last_injected > 0)
        k = len(s['times'])
        if k >= 100:
            t = 1e3 * np.asarray(s['times'])
            msg = (f'[{k} updates] time mean {t.mean():.1f} / max {t.max():.1f} ms | '
                   f'N_eff/N mean {np.mean(s["neff"]):.2f} min {np.min(s["neff"]):.2f} | '
                   f'resampled {100 * s["resampled"] / k:.0f}% | '
                   f'recovery: {s["inject_events"]} events, {s["injected"]} particles | '
                   f'odom scale {self.pf.odom_scale:.3f} | '
                   f'scans waited for odom: {s["deferred"]}')
            if s['late'] or s['old']:
                msg += (f' | WARNING {s["late"]} scans newer and {s["old"]} older than the odom '
                        f'buffer (clock mismatch or large latency?)')
            self.get_logger().info(msg)
            self.stats = self._empty_stats()

    def _lookup_laser_offset(self, laser_frame):
        """Look up the laser pose in the base frame from TF and pass it to the sensor model."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, laser_frame, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            offset = (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))
            self.sensor_model.laser_offset = np.asarray(offset)
            self.laser_offset_ready = True
            self.get_logger().info(
                f'laser offset from TF {self.base_frame}->{laser_frame}: '
                f'x={offset[0]:.3f}, y={offset[1]:.3f}, yaw={offset[2]:.3f}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.offset_attempts += 1
            if self.offset_attempts >= 20:
                self.laser_offset_ready = True
                self.get_logger().warn(
                    f'no TF {self.base_frame}->{laser_frame} ({e}); '
                    f'using laser_offset_fallback {self.laser_offset_fallback}')

    def _covariance6(self):
        """Return the (x, y, yaw) covariance as a row-major 6x6 list."""
        c = self.pf.covariance
        cov = [0.0] * 36
        for i, a in enumerate((0, 1, 5)):      # x, y, yaw in the 6x6 layout
            for j, b in enumerate((0, 1, 5)):
                cov[a * 6 + b] = float(c[i, j])
        return cov

    def _publish_pose(self, stamp):
        """Publish the estimate as a PoseStamped."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        _fill_pose(msg.pose, self.pf.estimate)
        self.pose_pub.publish(msg)

    def _publish_particles(self, stamp):
        """Publish (a subsample of) the particles as a PoseArray."""
        parts = self.pf.particles
        if parts.shape[0] > self.max_viz_particles:
            parts = parts[self.rng.choice(parts.shape[0], self.max_viz_particles, replace=False)]
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        for q in parts:
            pose = Pose()
            _fill_pose(pose, q)
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

    def _publish_map_to_odom(self, stamp):
        """Publish the map -> odom transform from the estimate and the odometry at the scan."""
        if self.filter_odom_pose is None:
            return
        # T_map_odom = T_map_base (+) inverse(T_odom_base), both at the scan stamp
        m2o = compose(self.pf.estimate, inverse(self.filter_odom_pose))
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(m2o[0])
        tf.transform.translation.y = float(m2o[1])
        qx, qy, qz, qw = yaw_to_quat(m2o[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    """Run the particle filter node until shutdown."""
    rclpy.init(args=args)
    node = ParticleFilterNode()
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