import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class TwistToAckermann(Node):
    """Converts teleop Twist (/cmd_vel) into AckermannDriveStamped (/drive).

    linear.x  -> speed (m/s)
    angular.z -> steering angle (rad), used directly as an angle, not a yaw rate
    """

    def __init__(self):
        super().__init__('twist_to_ackermann_node')
        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.max_steering_angle = self.declare_parameter('max_steering_angle', 0.35).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def cmd_vel_callback(self, msg):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.drive.speed = self.clamp(msg.linear.x, self.max_speed)
        out.drive.steering_angle = self.clamp(msg.angular.z, self.max_steering_angle)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()