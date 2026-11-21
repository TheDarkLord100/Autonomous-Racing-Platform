"""Web bridge: drive the car and watch telemetry from any browser.

Serves a single page (share/car_bringup/web/index.html) and a WebSocket at /ws.
ROS/DDS stays on the Jetson; only JSON over the WebSocket crosses the network.

Browser -> car (JSON):
    {"type": "cmd", "throttle": -1..1, "steer": -1..1}   sent at ~20 Hz
    {"type": "speed", "value": m/s}                       speed cap (clamped to max_speed_limit)
    {"type": "arm"} / {"type": "estop"}
    {"type": "claim"} / {"type": "release"}               take / give up driving
    {"type": "ping", "t": ms}                             echoed back as "pong"

Car -> browser: {"type": "telemetry", ...} at telemetry_rate.

States (reported to the browser as "state"):
    SAFE     disarmed, nothing published. ARM is needed to leave this state.
    ARMED    armed, throttle at zero, zero speed published.
    DRIVING  armed and the driver is commanding motion.
    HOLD     armed, but no cmd for cmd_timeout s (link stall): zero speed is published.
             Fresh cmds resume driving, but only after the throttle returns to
             neutral, so the car never lurches off a thumb held during the stall.

Safety:
    - Only one client (the driver) can command.
    - HOLD lasting link_lost_timeout s, driver disconnect, STOP, or the page going
      to the background -> SAFE. Zero speed is published for stop_hold s, then
      publishing stops.
    - Speed is always clamped to max_speed_limit, whatever the browser sends.
"""

import asyncio
import json
import math
import os
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
from aiohttp import web, WSMsgType
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from vesc_msgs.msg import VescStateStamped


def clamp(value, low, high):
    return max(low, min(high, value))


class WebBridge(Node):

    def __init__(self):
        super().__init__('web_bridge_node')
        p = self.declare_parameter
        self.host = p('host', '0.0.0.0').value
        self.port = p('port', 8080).value
        self.token = p('token', '').value
        self.max_speed_limit = p('max_speed_limit', 1.5).value
        self.max_steering_angle = p('max_steering_angle', 0.35).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.link_lost_timeout = p('link_lost_timeout', 3.0).value
        self.stop_hold = p('stop_hold', 0.5).value
        self.publish_rate = p('publish_rate', 50.0).value
        self.telemetry_rate = p('telemetry_rate', 10.0).value
        self.scan_points = p('scan_points', 180).value
        self.scan_yaw_offset = p('scan_yaw_offset', math.pi).value
        self.frame_id = p('frame_id', 'base_link').value

        # Shared between the ROS executor thread and the asyncio thread.
        self.lock = threading.Lock()
        self.armed = False
        self.hold = False          # armed, but commands have stalled
        self.need_neutral = False  # ignore throttle until it returns to ~0
        self.disarm_reason = 'not armed yet'
        self.throttle = 0.0
        self.steer = 0.0
        self.last_cmd_time = 0.0
        self.speed_cap = clamp(p('default_speed', 0.5).value, 0.0, self.max_speed_limit)
        self.stop_until = 0.0
        self.out_speed = 0.0
        self.out_steer = 0.0
        self.voltage = None
        self.erpm = None
        self.odom_speed = None
        self.scan = None

        # Only touched from the asyncio thread.
        self.ws_clients = set()
        self.driver = None

        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_subscription(VescStateStamped, 'sensors/core', self.on_core, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / self.publish_rate, self.on_publish_timer)

        self.index_path = os.path.join(
            get_package_share_directory('car_bringup'), 'web', 'index.html')

    # ---------------- ROS side (executor thread) ----------------

    def on_publish_timer(self):
        now = time.monotonic()
        with self.lock:
            if self.armed:
                age = now - self.last_cmd_time
                if age > self.link_lost_timeout:
                    self._disarm('link lost')
                elif age > self.cmd_timeout and not self.hold:
                    self.hold = True
                    self.need_neutral = True
                    self.throttle, self.steer = 0.0, 0.0
                    self.get_logger().warn('Link stalled: holding at zero speed')
            if self.armed:
                speed = self.throttle * self.speed_cap
                steer = self.steer * self.max_steering_angle
            elif now < self.stop_until:
                speed, steer = 0.0, 0.0
            else:
                self.out_speed, self.out_steer = 0.0, 0.0
                return  # idle: publish nothing, leave the VESC alone
            self.out_speed, self.out_steer = speed, steer
        self.publish_drive(speed, steer)

    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)

    def _disarm(self, reason):
        """Caller must hold self.lock."""
        if self.armed:
            self.stop_until = time.monotonic() + self.stop_hold
            self.get_logger().warn(f'Disarmed: {reason}')
        self.armed = False
        self.hold = False
        self.disarm_reason = reason
        self.throttle = 0.0
        self.steer = 0.0

    def on_core(self, msg):
        with self.lock:
            self.voltage = msg.state.voltage_input
            self.erpm = msg.state.speed

    def on_odom(self, msg):
        with self.lock:
            self.odom_speed = msg.twist.twist.linear.x

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        step = max(1, n // self.scan_points)
        ranges = []
        for r in msg.ranges[::step]:
            ok = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            ranges.append(round(r, 2) if ok else 0)
        with self.lock:
            self.scan = {
                'angle_min': msg.angle_min + self.scan_yaw_offset,
                'angle_inc': msg.angle_increment * step,
                'ranges': ranges,
            }

    # ---------------- Web side (asyncio thread) ----------------

    def authorised(self, request):
        return not self.token or request.query.get('token') == self.token

    async def handle_index(self, request):
        if not self.authorised(request):
            return web.Response(status=403, text='bad token')
        return web.FileResponse(self.index_path)

    async def handle_ws(self, request):
        if not self.authorised(request):
            return web.Response(status=403, text='bad token')
        ws = web.WebSocketResponse(heartbeat=5.0)  # dead-socket check only
        await ws.prepare(request)
        self.ws_clients.add(ws)
        if self.driver is None:
            self.driver = ws
        self.get_logger().info(f'Client connected: {request.remote} '
                               f'({"driver" if self.driver is ws else "viewer"})')
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    await self.handle_message(ws, json.loads(msg.data))
                except (ValueError, TypeError, KeyError):
                    pass  # ignore malformed messages
        finally:
            self.ws_clients.discard(ws)
            if self.driver is ws:
                self.driver = None
                with self.lock:
                    self._disarm('driver disconnected')
            self.get_logger().info(f'Client disconnected: {request.remote}')
        return ws

    async def handle_message(self, ws, m):
        kind = m.get('type')
        if kind == 'ping':
            await ws.send_json({'type': 'pong', 't': m['t']})
            return
        if kind == 'claim' and self.driver is None:
            self.driver = ws
            return
        if ws is not self.driver:
            return  # viewers cannot command

        now = time.monotonic()
        with self.lock:
            if kind == 'cmd':
                t, s = float(m['throttle']), float(m['steer'])
                if math.isfinite(t) and math.isfinite(s):
                    self.last_cmd_time = now
                    if self.armed:
                        if self.hold:
                            self.hold = False
                            self.get_logger().info('Link back: release throttle to resume')
                        if self.need_neutral and abs(t) < 0.05:
                            self.need_neutral = False
                        self.throttle = 0.0 if self.need_neutral else clamp(t, -1.0, 1.0)
                        self.steer = clamp(s, -1.0, 1.0)
            elif kind == 'speed':
                v = float(m['value'])
                if math.isfinite(v):
                    self.speed_cap = clamp(v, 0.0, self.max_speed_limit)
            elif kind == 'arm':
                self.throttle, self.steer = 0.0, 0.0
                self.last_cmd_time = now
                self.armed = True
                self.hold = False
                self.need_neutral = True
                self.disarm_reason = ''
                self.get_logger().info('Armed')
            elif kind == 'estop':
                self._disarm('E-STOP')
            elif kind == 'release':
                self._disarm('driver released control')
                self.driver = None

    def state(self):
        """Caller must hold self.lock."""
        if not self.armed:
            return 'SAFE'
        if self.hold:
            return 'HOLD'
        return 'DRIVING' if self.throttle != 0.0 else 'ARMED'

    def telemetry(self):
        with self.lock:
            age = time.monotonic() - self.last_cmd_time
            return {
                'type': 'telemetry',
                'state': self.state(),
                'armed': self.armed,
                'need_neutral': self.armed and self.need_neutral,
                'cmd_age_ms': round(age * 1000) if self.armed else None,
                'reason': self.disarm_reason,
                'speed_cap': self.speed_cap,
                'speed_limit': self.max_speed_limit,
                'cmd_speed': self.out_speed,
                'cmd_steer': self.out_steer,
                'voltage': self.voltage,
                'erpm': self.erpm,
                'odom_speed': self.odom_speed,
                'scan': self.scan,
            }

    async def telemetry_loop(self):
        period = 1.0 / self.telemetry_rate
        while rclpy.ok():
            await asyncio.sleep(period)
            if not self.ws_clients:
                continue
            data = self.telemetry()
            data['driver_free'] = self.driver is None
            for ws in list(self.ws_clients):
                data['role'] = 'driver' if ws is self.driver else 'viewer'
                try:
                    await ws.send_str(json.dumps(data))
                except (ConnectionResetError, RuntimeError):
                    pass

    async def serve(self):
        app = web.Application()
        app.router.add_get('/', self.handle_index)
        app.router.add_get('/ws', self.handle_ws)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, self.host, self.port).start()
        self.get_logger().info(f'Web bridge on http://{self.host}:{self.port}/')
        try:
            await self.telemetry_loop()
        finally:
            await runner.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = WebBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        asyncio.run(node.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)  # last word is always "stop"
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()