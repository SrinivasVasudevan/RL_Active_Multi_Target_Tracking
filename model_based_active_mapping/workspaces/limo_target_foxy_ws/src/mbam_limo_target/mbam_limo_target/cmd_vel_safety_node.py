"""Clamp cmd_vel and stop if obstacles are too close in a forward lidar wedge."""
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class CmdVelSafetyNode(Node):
    def __init__(self):
        super().__init__("mbam_cmd_vel_safety")

        self.declare_parameter("cmd_vel_in", "cmd_vel_raw")
        self.declare_parameter("cmd_vel_out", "cmd_vel")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("max_linear_x", 0.2)
        self.declare_parameter("max_angular_z", 0.5)
        self.declare_parameter("min_range_m", 0.45)
        self.declare_parameter("front_angle_deg", 70.0)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("halt_on_scan_timeout", False)

        self._max_v = float(self.get_parameter("max_linear_x").value)
        self._max_w = float(self.get_parameter("max_angular_z").value)
        self._min_range = float(self.get_parameter("min_range_m").value)
        half = math.radians(float(self.get_parameter("front_angle_deg").value) * 0.5)
        self._front_min = -half
        self._front_max = half
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self._halt_on_timeout = bool(self.get_parameter("halt_on_scan_timeout").value)

        self._last_scan: Optional[LaserScan] = None
        self._last_cmd = Twist()

        in_topic = str(self.get_parameter("cmd_vel_in").value)
        out_topic = str(self.get_parameter("cmd_vel_out").value)
        scan_topic = str(self.get_parameter("scan_topic").value)

        self.create_subscription(Twist, in_topic, self._cmd_cb, 20)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 20)
        self._pub = self.create_publisher(Twist, out_topic, 20)
        self._timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"CmdVelSafety in={in_topic} out={out_topic} scan={scan_topic} "
            f"max_v={self._max_v} max_w={self._max_w} min_range={self._min_range}"
        )

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg

    def _scan_cb(self, msg: LaserScan):
        self._last_scan = msg

    def _forward_clear(self) -> bool:
        if self._last_scan is None:
            return not self._halt_on_timeout
        now = self.get_clock().now()
        age = (now - Time.from_msg(self._last_scan.header.stamp)).nanoseconds * 1e-9
        if age > self._scan_timeout:
            return not self._halt_on_timeout

        ranges = np.asarray(self._last_scan.ranges, dtype=np.float64)
        n = ranges.size
        if n == 0 or self._last_scan.angle_increment == 0.0:
            return True

        angles = self._last_scan.angle_min + np.arange(n, dtype=np.float64) * self._last_scan.angle_increment
        mask = (angles >= self._front_min) & (angles <= self._front_max)
        r = ranges[mask]
        r = r[np.isfinite(r)]
        if r.size == 0:
            return True
        rmin = float(np.min(r))
        if rmin < self._min_range and self._last_cmd.linear.x > 0.01:
            return False
        return True

    def _tick(self):
        ok = self._forward_clear()
        out = Twist()
        v = max(-self._max_v, min(self._max_v, self._last_cmd.linear.x))
        w = max(-self._max_w, min(self._max_w, self._last_cmd.angular.z))
        if not ok:
            v = 0.0
            w = 0.0
        out.linear.x = v
        out.angular.z = w
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
