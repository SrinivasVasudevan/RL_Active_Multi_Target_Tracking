#!/usr/bin/env python3

import random

import rospy
from geometry_msgs.msg import Twist


class TargetMotionController:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "target")
        output_cmd_topic = rospy.get_param("~output_cmd_topic", "")
        if not output_cmd_topic:
            output_cmd_topic = f"/{self.robot_name}/target_cmd_vel_raw"

        self.motion_mode = rospy.get_param("~motion_mode", "constant").strip().lower()
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.constant_linear_velocity = float(rospy.get_param("~constant_linear_velocity", 0.20))
        self.constant_angular_velocity = float(rospy.get_param("~constant_angular_velocity", 0.0))
        self.max_linear_speed_mps = float(rospy.get_param("~max_linear_speed_mps", 0.35))
        self.max_angular_speed_rps = float(rospy.get_param("~max_angular_speed_rps", 0.80))
        self.command_hold_time_min_sec = float(rospy.get_param("~command_hold_time_min_sec", 1.5))
        self.command_hold_time_max_sec = float(rospy.get_param("~command_hold_time_max_sec", 4.0))
        self.wander_forward_velocity_min_mps = float(
            rospy.get_param("~wander_forward_velocity_min_mps", 0.12)
        )
        self.wander_forward_velocity_max_mps = float(
            rospy.get_param("~wander_forward_velocity_max_mps", 0.28)
        )
        self.wander_reverse_velocity_max_mps = float(
            rospy.get_param("~wander_reverse_velocity_max_mps", 0.10)
        )
        self.wander_angular_speed_max_rps = float(
            rospy.get_param("~wander_angular_speed_max_rps", 0.70)
        )
        self.reverse_probability = float(rospy.get_param("~reverse_probability", 0.05))
        self.stop_probability = float(rospy.get_param("~stop_probability", 0.05))
        self.in_place_turn_probability = float(rospy.get_param("~in_place_turn_probability", 0.15))

        random_seed = int(rospy.get_param("~random_seed", -1))
        if random_seed >= 0:
            random.seed(random_seed)

        self.cmd_pub = rospy.Publisher(output_cmd_topic, Twist, queue_size=10)
        self.current_cmd = Twist()
        self.next_resample_time = rospy.Time(0)
        self.timer = rospy.Timer(rospy.Duration(max(0.02, 1.0 / max(1e-3, self.publish_rate_hz))), self._tick)

        rospy.on_shutdown(self._publish_zero)
        rospy.loginfo(
            "Target motion controller ready for robot='%s' mode='%s' output='%s'",
            self.robot_name,
            self.motion_mode,
            output_cmd_topic,
        )

    def _tick(self, _event):
        now = rospy.Time.now()
        if self.motion_mode == "constant":
            cmd = Twist()
            cmd.linear.x = self._clip(self.constant_linear_velocity, -self.max_linear_speed_mps, self.max_linear_speed_mps)
            cmd.angular.z = self._clip(self.constant_angular_velocity, -self.max_angular_speed_rps, self.max_angular_speed_rps)
            self.current_cmd = cmd
        else:
            if now >= self.next_resample_time:
                self.current_cmd = self._sample_wander_command()
                hold_sec = random.uniform(
                    min(self.command_hold_time_min_sec, self.command_hold_time_max_sec),
                    max(self.command_hold_time_min_sec, self.command_hold_time_max_sec),
                )
                self.next_resample_time = now + rospy.Duration(max(0.1, hold_sec))

        self.cmd_pub.publish(self.current_cmd)

    def _sample_wander_command(self):
        cmd = Twist()
        mode_sample = random.random()

        if mode_sample < self.stop_probability:
            return cmd

        if mode_sample < self.stop_probability + self.in_place_turn_probability:
            cmd.angular.z = self._sample_turn_speed()
            return cmd

        if random.random() < self.reverse_probability:
            linear_speed = -random.uniform(0.0, max(0.0, self.wander_reverse_velocity_max_mps))
        else:
            linear_speed = random.uniform(
                min(self.wander_forward_velocity_min_mps, self.wander_forward_velocity_max_mps),
                max(self.wander_forward_velocity_min_mps, self.wander_forward_velocity_max_mps),
            )

        cmd.linear.x = self._clip(linear_speed, -self.max_linear_speed_mps, self.max_linear_speed_mps)
        cmd.angular.z = self._sample_turn_speed()
        return cmd

    def _sample_turn_speed(self):
        max_turn = max(0.0, self.wander_angular_speed_max_rps)
        if max_turn <= 1e-6:
            return 0.0
        return self._clip(random.uniform(-max_turn, max_turn), -self.max_angular_speed_rps, self.max_angular_speed_rps)

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _clip(value, lo, hi):
        return max(lo, min(hi, float(value)))


if __name__ == "__main__":
    rospy.init_node("target_motion_controller")
    TargetMotionController()
    rospy.spin()
