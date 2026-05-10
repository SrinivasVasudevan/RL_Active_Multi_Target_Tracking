"""Track planar world-frame velocity commands (Vector3Stamped) with diff-drive style cmd_vel."""
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class TargetVelocityFollower(Node):
    def __init__(self):
        super().__init__("mbam_target_velocity_follower")

        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("desired_vel_topic", "mbam/target_0/desired_world_velocity")
        self.declare_parameter("cmd_vel_out", "cmd_vel_raw")
        self.declare_parameter("max_linear_x", 0.15)
        self.declare_parameter("max_angular_z", 0.45)
        self.declare_parameter("angular_gain", 1.8)
        self.declare_parameter("stop_speed", 0.02)
        self.declare_parameter("vel_timeout_sec", 0.75)

        self._max_v = float(self.get_parameter("max_linear_x").value)
        self._max_w = float(self.get_parameter("max_angular_z").value)
        self._k_w = float(self.get_parameter("angular_gain").value)
        self._stop = float(self.get_parameter("stop_speed").value)
        self._vel_timeout = float(self.get_parameter("vel_timeout_sec").value)

        self._yaw: Optional[float] = None
        self._last_vel: Optional[Vector3Stamped] = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        vel_topic = str(self.get_parameter("desired_vel_topic").value)
        out_topic = str(self.get_parameter("cmd_vel_out").value)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(Vector3Stamped, vel_topic, self._vel_cb, 20)
        self._pub = self.create_publisher(Twist, out_topic, 20)
        self._timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"TargetVelocityFollower odom={odom_topic} desired={vel_topic} out={out_topic}"
        )

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._yaw = float(_quat_to_yaw(q.x, q.y, q.z, q.w))

    def _vel_cb(self, msg: Vector3Stamped):
        self._last_vel = msg

    def _tick(self):
        msg = Twist()
        if self._yaw is None or self._last_vel is None:
            self._pub.publish(msg)
            return

        now = self.get_clock().now()
        age = (now - Time.from_msg(self._last_vel.header.stamp)).nanoseconds * 1e-9
        if age > self._vel_timeout:
            self._pub.publish(msg)
            return

        vx_w = float(self._last_vel.vector.x)
        vy_w = float(self._last_vel.vector.y)
        speed = math.hypot(vx_w, vy_w)
        if speed < self._stop:
            self._pub.publish(msg)
            return

        heading = math.atan2(vy_w, vx_w)
        err = _wrap(heading - self._yaw)
        lin = speed * max(0.0, math.cos(err))
        ang = self._k_w * err
        lin = max(-self._max_v, min(self._max_v, lin))
        ang = max(-self._max_w, min(self._max_w, ang))
        msg.linear.x = lin
        msg.angular.z = ang
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetVelocityFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
