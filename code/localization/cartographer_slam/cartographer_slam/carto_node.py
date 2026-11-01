"""Thin wrapper around stock cartographer_ros. No SLAM code here.

Both modes
- static TF map -> carto_map (origin = car pose in map when mapping started)
- /tracked_pose (carto_map) -> /carto/pose and /carto/path (map)

Localization
- /initialpose (map) or the initial_pose param -> finish the current trajectory,
  start a new one with the pose relative to the frozen map (trajectory 0)

Mapping
- /carto/save_map (std_srvs/Trigger) -> finish trajectory 0, wait for the final
  optimization, write <map_dir>/<map_name>.pbstream and <map_name>.yaml (origin)
"""

import math
import os
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cartographer_ros_msgs.msg import StatusCode
from cartographer_ros_msgs.srv import (FinishTrajectory, StartTrajectory,
                                       TrajectoryQuery, WriteState)
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


# ---------------------------------------------------------------- SE(2) ----

def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    """a ⊕ b: apply b in a's frame. Poses are (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            wrap_angle(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1],
            s * a[0] - c * a[1],
            wrap_angle(-a[2]))


def pose_to_xyyaw(p):
    q = p.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (p.position.x, p.position.y, yaw)


def xyyaw_to_pose(x, y, yaw):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z = math.sin(yaw / 2.0)
    p.orientation.w = math.cos(yaw / 2.0)
    return p


# ----------------------------------------------------------------- node ----

class CartoNode(Node):

    def __init__(self):
        super().__init__('carto_node')

        self.mode = self.declare_parameter('mode', 'localization').value
        tracked_topic = self.declare_parameter('tracked_pose_topic', '/tracked_pose').value
        initialpose_topic = self.declare_parameter('initialpose_topic', '/initialpose').value
        pose_topic = self.declare_parameter('pose_topic', '/carto/pose').value
        path_topic = self.declare_parameter('path_topic', '/carto/path').value
        self.path_max_length = self.declare_parameter('path_max_length', 3000).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.carto_frame = self.declare_parameter('carto_map_frame', 'carto_map').value
        self.origin = tuple(self.declare_parameter('origin', [0.0, 0.0, 0.0]).value)
        self.map_dir = self.declare_parameter('map_dir', '').value
        self.map_name = self.declare_parameter('map_name', '').value
        self.config_dir = self.declare_parameter('configuration_directory', '').value
        self.config_base = self.declare_parameter(
            'configuration_basename', 'carto_localization.lua').value
        use_init_param = self.declare_parameter('use_initial_pose_param', False).value
        self.initial_pose = tuple(self.declare_parameter('initial_pose', [0.0, 0.0, 0.0]).value)
        self.timeout = self.declare_parameter('service_timeout_sec', 10.0).value
        self.opt_wait = self.declare_parameter('optimization_wait_sec', 5.0).value

        if self.mode not in ('mapping', 'localization'):
            raise ValueError(f"mode must be 'mapping' or 'localization', got '{self.mode}'")

        cb = ReentrantCallbackGroup()   # service calls block inside callbacks
        self.lock = threading.Lock()    # one trajectory restart / save at a time

        # map -> carto_map
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.carto_frame
        o = xyyaw_to_pose(*self.origin)
        t.transform.translation.x = o.position.x
        t.transform.translation.y = o.position.y
        t.transform.rotation = o.orientation
        self.static_tf.sendTransform(t)

        # pose output
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        # default (mutually exclusive) group: poses are handled one at a time, in order
        self.create_subscription(PoseStamped, tracked_topic, self.tracked_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, initialpose_topic,
                                 self.initialpose_cb, 10, callback_group=cb)

        # cartographer_node services
        self.start_cli = self.create_client(StartTrajectory, 'start_trajectory', callback_group=cb)
        self.finish_cli = self.create_client(FinishTrajectory, 'finish_trajectory',
                                             callback_group=cb)
        self.query_cli = self.create_client(TrajectoryQuery, 'trajectory_query',
                                            callback_group=cb)
        self.write_cli = self.create_client(WriteState, 'write_state', callback_group=cb)

        self.first_node = None   # global pose of the frozen map's first node (carto_map)
        if self.mode == 'mapping':
            self.active_traj = 0  # started by cartographer_node with default topics
            self.create_service(Trigger, '/carto/save_map', self.save_map_cb, callback_group=cb)
        else:
            self.active_traj = None  # started on the first initial pose
            if use_init_param:
                self.init_timer = self.create_timer(1.0, self.init_from_param, callback_group=cb)

        self.get_logger().info(
            f'carto_node ({self.mode}): {tracked_topic} ({self.carto_frame}) -> {pose_topic} '
            f'({self.map_frame}); origin {self.origin}')
        if self.mode == 'localization' and not use_init_param:
            self.get_logger().info('waiting for an initial pose on ' + initialpose_topic)

    # ------------------------------------------------------------ outputs --

    def tracked_cb(self, msg):
        x, y, yaw = compose(self.origin, pose_to_xyyaw(msg.pose))
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose = xyyaw_to_pose(x, y, yaw)
        self.pose_pub.publish(out)

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(out)
        if len(self.path.poses) > self.path_max_length:
            del self.path.poses[:len(self.path.poses) - self.path_max_length]
        self.path_pub.publish(self.path)

    # ------------------------------------------------------- localization --

    def init_from_param(self):
        self.init_timer.cancel()
        self.restart_trajectory(self.initial_pose)

    def initialpose_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'initial pose in frame {msg.header.frame_id}, expected {self.map_frame}; ignored')
            return
        pose = pose_to_xyyaw(msg.pose.pose)
        if self.mode == 'mapping':
            self.get_logger().warn('initial pose ignored in mapping mode (moving the car '
                                   'during mapping corrupts the map)')
            return
        self.restart_trajectory(pose)

    def restart_trajectory(self, pose_map):
        with self.lock:
            if self.first_node is None:
                res = self.call(self.query_cli, TrajectoryQuery.Request(trajectory_id=0))
                if not self.ok(res, 'trajectory_query(0)') or not res.trajectory:
                    self.get_logger().error('frozen map (trajectory 0) not available')
                    return
                self.first_node = pose_to_xyyaw(res.trajectory[0].pose)

            if self.active_traj is not None:
                res = self.call(self.finish_cli,
                                FinishTrajectory.Request(trajectory_id=self.active_traj))
                self.ok(res, f'finish_trajectory({self.active_traj})')
                self.active_traj = None

            # Cartographer takes the initial pose relative to the global pose of
            # trajectory 0's first node.
            pose_carto = compose(inverse(self.origin), pose_map)
            rel = compose(inverse(self.first_node), pose_carto)
            req = StartTrajectory.Request()
            req.configuration_directory = self.config_dir
            req.configuration_basename = self.config_base
            req.use_initial_pose = True
            req.initial_pose = xyyaw_to_pose(*rel)
            req.relative_to_trajectory_id = 0
            res = self.call(self.start_cli, req)
            if not self.ok(res, 'start_trajectory'):
                return
            self.active_traj = res.trajectory_id
            self.path.poses.clear()
            self.get_logger().info(
                f'trajectory {self.active_traj} started at map ({pose_map[0]:.2f}, '
                f'{pose_map[1]:.2f}, {pose_map[2]:.2f})')

    # ------------------------------------------------------------ mapping --

    def save_map_cb(self, request, response):
        with self.lock:
            if not self.map_name or not self.map_dir:
                response.success = False
                response.message = 'map_dir and map_name must be set'
                return response
            os.makedirs(self.map_dir, exist_ok=True)
            pbstream = os.path.join(self.map_dir, self.map_name + '.pbstream')
            sidecar = os.path.join(self.map_dir, self.map_name + '.yaml')

            if self.active_traj is not None:
                res = self.call(self.finish_cli,
                                FinishTrajectory.Request(trajectory_id=self.active_traj))
                if not self.ok(res, f'finish_trajectory({self.active_traj})'):
                    response.success = False
                    response.message = 'finish_trajectory failed'
                    return response
                self.active_traj = None
                # finishing queues a final optimization that write_state does not wait for
                self.get_logger().info(f'waiting {self.opt_wait:.1f} s for the final optimization')
                time.sleep(self.opt_wait)

            res = self.call(self.write_cli,
                            WriteState.Request(filename=pbstream, include_unfinished_submaps=True))
            if not self.ok(res, 'write_state'):
                response.success = False
                response.message = 'write_state failed'
                return response

            with open(sidecar, 'w') as f:
                f.write('# cartographer_slam map\n')
                f.write(f'pbstream: {self.map_name}.pbstream\n')
                f.write('# pose of carto_map in the simulator map frame (car pose when mapping '
                        'started): [x, y, yaw]\n')
                f.write(f'origin: [{self.origin[0]!r}, {self.origin[1]!r}, {self.origin[2]!r}]\n')

            response.success = True
            response.message = f'saved {pbstream} and {sidecar}'
            self.get_logger().info(response.message)
            return response

    # ------------------------------------------------------------ helpers --

    def call(self, client, request):
        """Blocking service call; needs the multithreaded executor."""
        if not client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().error(f'service {client.srv_name} not available')
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.timeout):
            self.get_logger().error(f'service {client.srv_name} timed out')
            return None
        return future.result()

    def ok(self, res, what):
        if res is None:
            return False
        if res.status.code != StatusCode.OK:
            self.get_logger().error(f'{what}: {res.status.message}')
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CartoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()