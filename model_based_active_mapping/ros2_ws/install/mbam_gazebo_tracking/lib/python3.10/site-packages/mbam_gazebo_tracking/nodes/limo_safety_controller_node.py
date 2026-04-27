import math
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LimoSafetyController(Node):
    def __init__(self):
        super().__init__("limo_safety_controller")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("input_cmd_topic", "")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter("stop_on_missing_scan", True)
        self.declare_parameter("min_valid_range_m", 0.15)
        self.declare_parameter("max_linear_speed_mps", 0.6)
        self.declare_parameter("max_angular_speed_rps", 1.2)
        self.declare_parameter("front_sector_half_angle_deg", 25.0)
        self.declare_parameter("front_diagonal_center_angle_deg", 45.0)
        self.declare_parameter("front_diagonal_half_angle_deg", 22.0)
        self.declare_parameter("side_sector_center_angle_deg", 90.0)
        self.declare_parameter("side_sector_half_angle_deg", 20.0)
        self.declare_parameter("rear_sector_half_angle_deg", 30.0)
        self.declare_parameter("forward_emergency_stop_distance_m", 0.45)
        self.declare_parameter("forward_slowdown_distance_m", 0.90)
        self.declare_parameter("reverse_emergency_stop_distance_m", 0.30)
        self.declare_parameter("reverse_slowdown_distance_m", 0.60)
        self.declare_parameter("turn_clearance_distance_m", 0.35)
        self.declare_parameter("side_clearance_distance_m", 0.28)
        self.declare_parameter("escape_turn_speed_rps", 0.55)
        self.declare_parameter("turn_bias_margin_m", 0.08)

        robot_name = str(self.get_parameter("robot_name").value).strip()
        if not robot_name:
            robot_name = self.get_namespace().strip("/")
        self.robot_name = robot_name or "limo"

        input_cmd_topic = str(self.get_parameter("input_cmd_topic").value).strip()
        if not input_cmd_topic:
            input_cmd_topic = f"/{self.robot_name}/mbam_cmd_vel"

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.input_cmd_topic = input_cmd_topic
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self.stop_on_missing_scan = bool(self.get_parameter("stop_on_missing_scan").value)
        self.min_valid_range_m = float(self.get_parameter("min_valid_range_m").value)
        self.max_linear_speed_mps = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_angular_speed_rps = float(self.get_parameter("max_angular_speed_rps").value)
        self.front_sector_half_angle_deg = float(self.get_parameter("front_sector_half_angle_deg").value)
        self.front_diagonal_center_angle_deg = float(self.get_parameter("front_diagonal_center_angle_deg").value)
        self.front_diagonal_half_angle_deg = float(self.get_parameter("front_diagonal_half_angle_deg").value)
        self.side_sector_center_angle_deg = float(self.get_parameter("side_sector_center_angle_deg").value)
        self.side_sector_half_angle_deg = float(self.get_parameter("side_sector_half_angle_deg").value)
        self.rear_sector_half_angle_deg = float(self.get_parameter("rear_sector_half_angle_deg").value)
        self.forward_emergency_stop_distance_m = float(
            self.get_parameter("forward_emergency_stop_distance_m").value
        )
        self.forward_slowdown_distance_m = float(
            self.get_parameter("forward_slowdown_distance_m").value
        )
        self.reverse_emergency_stop_distance_m = float(
            self.get_parameter("reverse_emergency_stop_distance_m").value
        )
        self.reverse_slowdown_distance_m = float(
            self.get_parameter("reverse_slowdown_distance_m").value
        )
        self.turn_clearance_distance_m = float(self.get_parameter("turn_clearance_distance_m").value)
        self.side_clearance_distance_m = float(self.get_parameter("side_clearance_distance_m").value)
        self.escape_turn_speed_rps = float(self.get_parameter("escape_turn_speed_rps").value)
        self.turn_bias_margin_m = float(self.get_parameter("turn_bias_margin_m").value)

        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time_sec = 0.0
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time_sec = 0.0
        self.warn_times: Dict[str, float] = {}

        self.create_subscription(Twist, self.input_cmd_topic, self._cmd_cb, 20)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 20)
        self.timer = self.create_timer(self.control_period_sec, self._control_loop)

        self.get_logger().info(
            f"LIMO safety controller ready for robot='{self.robot_name}' "
            f"input='{self.input_cmd_topic}' output='{self.output_cmd_topic}' scan='{self.scan_topic}'"
        )

    def _cmd_cb(self, msg: Twist):
        self.latest_cmd = msg
        self.latest_cmd_time_sec = self._now_sec()

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time_sec = self._now_sec()

    def _control_loop(self):
        now_sec = self._now_sec()
        if self.latest_cmd is None or (now_sec - self.latest_cmd_time_sec) > self.cmd_timeout_sec:
            self._publish_zero()
            self._warn_throttle("cmd_timeout", "Planner command timed out; publishing zero velocity.")
            return

        if self.latest_scan is None or (now_sec - self.latest_scan_time_sec) > self.scan_timeout_sec:
            if self.stop_on_missing_scan:
                self._publish_zero()
                self._warn_throttle("scan_timeout", "Laser scan timed out; publishing zero velocity.")
                return
            sector_mins = self._default_sector_mins()
        else:
            sector_mins = self._compute_sector_mins(self.latest_scan)

        safe_cmd = self._filter_command(self.latest_cmd, sector_mins)
        self.cmd_pub.publish(safe_cmd)

    def _filter_command(self, cmd: Twist, sector_mins: Dict[str, float]) -> Twist:
        safe = Twist()
        safe.linear.x = self._clip(cmd.linear.x, -self.max_linear_speed_mps, self.max_linear_speed_mps)
        safe.angular.z = self._clip(cmd.angular.z, -self.max_angular_speed_rps, self.max_angular_speed_rps)

        front_min = min(sector_mins["front"], sector_mins["front_left"], sector_mins["front_right"])
        rear_min = sector_mins["rear"]
        left_turn_clearance = min(sector_mins["left"], sector_mins["front_left"])
        right_turn_clearance = min(sector_mins["right"], sector_mins["front_right"])

        if safe.linear.x > 0.0:
            safe.linear.x = self._scaled_linear_velocity(
                velocity=safe.linear.x,
                clearance=front_min,
                emergency=self.forward_emergency_stop_distance_m,
                slowdown=self.forward_slowdown_distance_m,
            )
            if front_min <= self.forward_emergency_stop_distance_m:
                if abs(safe.angular.z) < 1e-3:
                    safe.angular.z = self._escape_turn(left_clearance=left_turn_clearance, right_turn_clearance=right_turn_clearance)
                elif safe.angular.z > 0.0 and left_turn_clearance <= self.turn_clearance_distance_m:
                    safe.angular.z = 0.0
                elif safe.angular.z < 0.0 and right_turn_clearance <= self.turn_clearance_distance_m:
                    safe.angular.z = 0.0

        elif safe.linear.x < 0.0:
            safe.linear.x = -self._scaled_linear_velocity(
                velocity=abs(safe.linear.x),
                clearance=rear_min,
                emergency=self.reverse_emergency_stop_distance_m,
                slowdown=self.reverse_slowdown_distance_m,
            )

        if safe.angular.z > 0.0 and left_turn_clearance <= self.side_clearance_distance_m:
            safe.angular.z = 0.0
        elif safe.angular.z < 0.0 and right_turn_clearance <= self.side_clearance_distance_m:
            safe.angular.z = 0.0

        if safe.linear.x > 0.0 and front_min <= self.forward_emergency_stop_distance_m:
            safe.linear.x = 0.0
            if abs(safe.angular.z) < 1e-3:
                safe.angular.z = self._escape_turn(left_clearance=left_turn_clearance, right_turn_clearance=right_turn_clearance)

        if safe.linear.x < 0.0 and rear_min <= self.reverse_emergency_stop_distance_m:
            safe.linear.x = 0.0

        if abs(safe.linear.x) < 1e-4:
            safe.linear.x = 0.0
        if abs(safe.angular.z) < 1e-4:
            safe.angular.z = 0.0

        return safe

    def _scaled_linear_velocity(self, velocity: float, clearance: float, emergency: float, slowdown: float) -> float:
        if clearance <= emergency:
            return 0.0
        if slowdown <= emergency:
            return velocity
        if clearance >= slowdown:
            return velocity
        scale = (clearance - emergency) / max(1e-6, slowdown - emergency)
        return velocity * self._clip(scale, 0.0, 1.0)

    def _escape_turn(self, left_clearance: float, right_turn_clearance: float) -> float:
        if left_clearance > right_turn_clearance + self.turn_bias_margin_m:
            return abs(self.escape_turn_speed_rps)
        if right_turn_clearance > left_clearance + self.turn_bias_margin_m:
            return -abs(self.escape_turn_speed_rps)
        return abs(self.escape_turn_speed_rps)

    def _compute_sector_mins(self, scan: LaserScan) -> Dict[str, float]:
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        if ranges.size == 0:
            return self._default_sector_mins()

        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        valid = np.isfinite(ranges)
        valid &= ranges >= max(float(scan.range_min), self.min_valid_range_m)
        valid &= ranges <= float(scan.range_max)
        if not np.any(valid):
            return self._default_sector_mins()

        sectors = {
            "front": self._sector_min(
                ranges=ranges,
                wrapped_angles=wrapped,
                valid=valid,
                center_deg=0.0,
                half_deg=self.front_sector_half_angle_deg,
            ),
            "front_left": self._sector_min(
                ranges=ranges,
                wrapped_angles=wrapped,
                valid=valid,
                center_deg=self.front_diagonal_center_angle_deg,
                half_deg=self.front_diagonal_half_angle_deg,
            ),
            "front_right": self._sector_min(
                ranges=ranges,
                wrapped_angles=wrapped,
                valid=valid,
                center_deg=-self.front_diagonal_center_angle_deg,
                half_deg=self.front_diagonal_half_angle_deg,
            ),
            "left": self._sector_min(
                ranges=ranges,
                wrapped_angles=wrapped,
                valid=valid,
                center_deg=self.side_sector_center_angle_deg,
                half_deg=self.side_sector_half_angle_deg,
            ),
            "right": self._sector_min(
                ranges=ranges,
                wrapped_angles=wrapped,
                valid=valid,
                center_deg=-self.side_sector_center_angle_deg,
                half_deg=self.side_sector_half_angle_deg,
            ),
            "rear": min(
                self._sector_min(
                    ranges=ranges,
                    wrapped_angles=wrapped,
                    valid=valid,
                    center_deg=180.0,
                    half_deg=self.rear_sector_half_angle_deg,
                ),
                self._sector_min(
                    ranges=ranges,
                    wrapped_angles=wrapped,
                    valid=valid,
                    center_deg=-180.0,
                    half_deg=self.rear_sector_half_angle_deg,
                ),
            ),
        }
        return sectors

    def _sector_min(
        self,
        ranges: np.ndarray,
        wrapped_angles: np.ndarray,
        valid: np.ndarray,
        center_deg: float,
        half_deg: float,
    ) -> float:
        center = math.radians(center_deg)
        half = math.radians(max(0.1, half_deg))
        delta = np.arctan2(np.sin(wrapped_angles - center), np.cos(wrapped_angles - center))
        mask = valid & (np.abs(delta) <= half)
        if not np.any(mask):
            return float("inf")
        return float(np.min(ranges[mask]))

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _warn_throttle(self, key: str, message: str, period_sec: float = 1.0):
        now_sec = self._now_sec()
        last_time = self.warn_times.get(key, 0.0)
        if now_sec - last_time >= period_sec:
            self.get_logger().warn(message)
            self.warn_times[key] = now_sec

    def _default_sector_mins(self) -> Dict[str, float]:
        return {
            "front": float("inf"),
            "front_left": float("inf"),
            "front_right": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
            "rear": float("inf"),
        }

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _clip(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = LimoSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
