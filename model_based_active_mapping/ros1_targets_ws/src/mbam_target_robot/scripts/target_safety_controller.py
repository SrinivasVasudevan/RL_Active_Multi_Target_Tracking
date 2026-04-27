#!/usr/bin/env python3

import math

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TargetSafetyController:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "target")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")

        input_cmd_topic = rospy.get_param("~input_cmd_topic", "")
        if not input_cmd_topic:
            input_cmd_topic = f"/{self.robot_name}/target_cmd_vel_raw"
        self.input_cmd_topic = input_cmd_topic
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel")

        self.control_period_sec = float(rospy.get_param("~control_period_sec", 0.05))
        self.cmd_timeout_sec = float(rospy.get_param("~cmd_timeout_sec", 0.5))
        self.scan_timeout_sec = float(rospy.get_param("~scan_timeout_sec", 0.5))
        self.stop_on_missing_scan = bool(rospy.get_param("~stop_on_missing_scan", True))
        self.min_valid_range_m = float(rospy.get_param("~min_valid_range_m", 0.15))
        self.max_linear_speed_mps = float(rospy.get_param("~max_linear_speed_mps", 0.35))
        self.max_angular_speed_rps = float(rospy.get_param("~max_angular_speed_rps", 0.90))

        self.front_sector_half_angle_deg = float(rospy.get_param("~front_sector_half_angle_deg", 25.0))
        self.front_diagonal_center_angle_deg = float(
            rospy.get_param("~front_diagonal_center_angle_deg", 45.0)
        )
        self.front_diagonal_half_angle_deg = float(
            rospy.get_param("~front_diagonal_half_angle_deg", 22.0)
        )
        self.side_sector_center_angle_deg = float(rospy.get_param("~side_sector_center_angle_deg", 90.0))
        self.side_sector_half_angle_deg = float(rospy.get_param("~side_sector_half_angle_deg", 20.0))
        self.rear_sector_half_angle_deg = float(rospy.get_param("~rear_sector_half_angle_deg", 30.0))

        self.forward_emergency_stop_distance_m = float(
            rospy.get_param("~forward_emergency_stop_distance_m", 0.45)
        )
        self.forward_slowdown_distance_m = float(
            rospy.get_param("~forward_slowdown_distance_m", 0.90)
        )
        self.reverse_emergency_stop_distance_m = float(
            rospy.get_param("~reverse_emergency_stop_distance_m", 0.30)
        )
        self.reverse_slowdown_distance_m = float(
            rospy.get_param("~reverse_slowdown_distance_m", 0.60)
        )
        self.turn_clearance_distance_m = float(rospy.get_param("~turn_clearance_distance_m", 0.35))
        self.side_clearance_distance_m = float(rospy.get_param("~side_clearance_distance_m", 0.28))
        self.escape_turn_speed_rps = float(rospy.get_param("~escape_turn_speed_rps", 0.55))
        self.turn_bias_margin_m = float(rospy.get_param("~turn_bias_margin_m", 0.08))

        self.latest_cmd = None
        self.latest_cmd_time = rospy.Time(0)
        self.latest_scan = None
        self.latest_scan_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.input_cmd_topic, Twist, self._cmd_cb, queue_size=20)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(max(0.02, self.control_period_sec)), self._tick)

        rospy.on_shutdown(self._publish_zero)
        rospy.loginfo(
            "Target safety controller ready for robot='%s' input='%s' output='%s' scan='%s'",
            self.robot_name,
            self.input_cmd_topic,
            self.output_cmd_topic,
            self.scan_topic,
        )

    def _cmd_cb(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = rospy.Time.now()

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = rospy.Time.now()

    def _tick(self, _event):
        now = rospy.Time.now()
        if self.latest_cmd is None or (now - self.latest_cmd_time).to_sec() > self.cmd_timeout_sec:
            self._publish_zero()
            rospy.logwarn_throttle(1.0, "Target safety controller: planner command timed out; publishing zero velocity.")
            return

        if self.latest_scan is None or (now - self.latest_scan_time).to_sec() > self.scan_timeout_sec:
            if self.stop_on_missing_scan:
                self._publish_zero()
                rospy.logwarn_throttle(1.0, "Target safety controller: laser scan timed out; publishing zero velocity.")
                return
            sector_mins = self._default_sector_mins()
        else:
            sector_mins = self._compute_sector_mins(self.latest_scan)

        self.cmd_pub.publish(self._filter_command(self.latest_cmd, sector_mins))

    def _filter_command(self, cmd, sector_mins):
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
                    safe.angular.z = self._escape_turn(left_turn_clearance, right_turn_clearance)
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
                safe.angular.z = self._escape_turn(left_turn_clearance, right_turn_clearance)

        if safe.linear.x < 0.0 and rear_min <= self.reverse_emergency_stop_distance_m:
            safe.linear.x = 0.0

        if abs(safe.linear.x) < 1e-4:
            safe.linear.x = 0.0
        if abs(safe.angular.z) < 1e-4:
            safe.angular.z = 0.0

        return safe

    def _scaled_linear_velocity(self, velocity, clearance, emergency, slowdown):
        if clearance <= emergency:
            return 0.0
        if slowdown <= emergency:
            return velocity
        if clearance >= slowdown:
            return velocity
        scale = (clearance - emergency) / max(1e-6, slowdown - emergency)
        return velocity * self._clip(scale, 0.0, 1.0)

    def _escape_turn(self, left_turn_clearance, right_turn_clearance):
        if left_turn_clearance > right_turn_clearance + self.turn_bias_margin_m:
            return abs(self.escape_turn_speed_rps)
        if right_turn_clearance > left_turn_clearance + self.turn_bias_margin_m:
            return -abs(self.escape_turn_speed_rps)
        return abs(self.escape_turn_speed_rps)

    def _compute_sector_mins(self, scan):
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

        return {
            "front": self._sector_min(ranges, wrapped, valid, 0.0, self.front_sector_half_angle_deg),
            "front_left": self._sector_min(
                ranges,
                wrapped,
                valid,
                self.front_diagonal_center_angle_deg,
                self.front_diagonal_half_angle_deg,
            ),
            "front_right": self._sector_min(
                ranges,
                wrapped,
                valid,
                -self.front_diagonal_center_angle_deg,
                self.front_diagonal_half_angle_deg,
            ),
            "left": self._sector_min(
                ranges,
                wrapped,
                valid,
                self.side_sector_center_angle_deg,
                self.side_sector_half_angle_deg,
            ),
            "right": self._sector_min(
                ranges,
                wrapped,
                valid,
                -self.side_sector_center_angle_deg,
                self.side_sector_half_angle_deg,
            ),
            "rear": min(
                self._sector_min(ranges, wrapped, valid, 180.0, self.rear_sector_half_angle_deg),
                self._sector_min(ranges, wrapped, valid, -180.0, self.rear_sector_half_angle_deg),
            ),
        }

    def _sector_min(self, ranges, wrapped_angles, valid, center_deg, half_deg):
        center = math.radians(center_deg)
        half = math.radians(max(0.1, half_deg))
        delta = np.arctan2(np.sin(wrapped_angles - center), np.cos(wrapped_angles - center))
        mask = valid & (np.abs(delta) <= half)
        if not np.any(mask):
            return float("inf")
        return float(np.min(ranges[mask]))

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _default_sector_mins():
        return {
            "front": float("inf"),
            "front_left": float("inf"),
            "front_right": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
            "rear": float("inf"),
        }

    @staticmethod
    def _clip(value, lo, hi):
        return max(lo, min(hi, float(value)))


if __name__ == "__main__":
    rospy.init_node("target_safety_controller")
    TargetSafetyController()
    rospy.spin()
