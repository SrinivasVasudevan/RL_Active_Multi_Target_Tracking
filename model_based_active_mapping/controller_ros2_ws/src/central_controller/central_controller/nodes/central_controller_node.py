import json
import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CentralControllerNode(Node):
    """Receive LIMO observations and publish per-robot desired velocity commands."""

    def __init__(self) -> None:
        super().__init__('central_controller')

        self.declare_parameter('robot_ids', 'limo_1,limo_2')
        self.declare_parameter('observation_topic', 'local_observation')
        self.declare_parameter('ack_topic', 'observation_ack')
        self.declare_parameter('controller_cmd_vel_topic', 'controller_cmd_vel')
        self.declare_parameter('control_period_sec', 0.2)
        self.declare_parameter('observation_timeout_sec', 1.0)
        self.declare_parameter('policy_mode', 'target_or_search')
        self.declare_parameter('search_angular_radps', 0.25)
        self.declare_parameter('target_linear_mps', 0.16)
        self.declare_parameter('target_angular_gain', 0.8)
        self.declare_parameter('max_linear_mps', 0.18)
        self.declare_parameter('max_angular_radps', 0.55)
        self.declare_parameter('target_stop_range_m', 0.9)
        self.declare_parameter('min_detection_score', 0.35)

        self.robot_ids = self._get_robot_ids()
        observation_topic = str(self.get_parameter('observation_topic').value)
        ack_topic = str(self.get_parameter('ack_topic').value)
        controller_cmd_vel_topic = str(self.get_parameter('controller_cmd_vel_topic').value)
        control_period_sec = float(self.get_parameter('control_period_sec').value)

        self.observation_timeout_sec = float(self.get_parameter('observation_timeout_sec').value)
        self.policy_mode = str(self.get_parameter('policy_mode').value)
        self.search_angular_radps = float(self.get_parameter('search_angular_radps').value)
        self.target_linear_mps = float(self.get_parameter('target_linear_mps').value)
        self.target_angular_gain = float(self.get_parameter('target_angular_gain').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.target_stop_range_m = float(self.get_parameter('target_stop_range_m').value)
        self.min_detection_score = float(self.get_parameter('min_detection_score').value)

        self._ack_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self._cmd_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self._latest_observations: Dict[str, Dict[str, object]] = {}
        self._latest_observation_time: Dict[str, rclpy.time.Time] = {}

        for robot_id in self.robot_ids:
            observation_name = f'/{robot_id}/{observation_topic}'
            ack_name = f'/{robot_id}/{ack_topic}'
            cmd_name = f'/{robot_id}/{controller_cmd_vel_topic}'

            self.create_subscription(
                String,
                observation_name,
                self._make_observation_callback(robot_id, ack_name),
                10,
            )
            self._ack_publishers[robot_id] = self.create_publisher(String, ack_name, 10)
            self._cmd_publishers[robot_id] = self.create_publisher(Twist, cmd_name, 10)

        self.create_timer(control_period_sec, self._control_loop)

        self.get_logger().info(
            'Central controller ready for robots: ' + ', '.join(self.robot_ids)
        )
        self.get_logger().info(
            f'Publishing desired velocity commands with policy_mode={self.policy_mode}'
        )

    def _get_robot_ids(self) -> List[str]:
        value = self.get_parameter('robot_ids').value
        if isinstance(value, str):
            return [item.strip() for item in value.split(',') if item.strip()]
        return [str(item) for item in value]

    def _make_observation_callback(self, expected_robot_id: str, ack_topic: str):
        def callback(msg: String) -> None:
            received_at = self.get_clock().now()
            stamp = received_at.to_msg()
            try:
                observation = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().warning(
                    f'Ignoring malformed observation on {expected_robot_id}: {exc}'
                )
                return

            reported_robot_id = str(observation.get('robot_id', ''))
            if reported_robot_id != expected_robot_id:
                self.get_logger().warning(
                    'Observation arrived on '
                    f'{expected_robot_id} topic but payload robot_id={reported_robot_id!r}'
                )
                return

            self._latest_observations[expected_robot_id] = observation
            self._latest_observation_time[expected_robot_id] = received_at

            ack = {
                'robot_id': expected_robot_id,
                'acknowledged': True,
                'ack_message': 'local observation received',
                'observation_seq': observation.get('seq'),
                'observation_stamp': observation.get('stamp'),
                'controller_stamp': {
                    'sec': stamp.sec,
                    'nanosec': stamp.nanosec,
                },
            }

            self._ack_publishers[expected_robot_id].publish(String(data=json.dumps(ack)))

        return callback

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        for robot_id in self.robot_ids:
            cmd = Twist()
            observation_time = self._latest_observation_time.get(robot_id)
            observation = self._latest_observations.get(robot_id)

            if observation_time is None or observation is None:
                self._cmd_publishers[robot_id].publish(cmd)
                continue

            age = (now - observation_time).nanoseconds * 1e-9
            if age > self.observation_timeout_sec:
                self._cmd_publishers[robot_id].publish(cmd)
                continue

            cmd = self._compute_command(robot_id, observation)
            self._cmd_publishers[robot_id].publish(cmd)

    def _compute_command(self, robot_id: str, observation: Dict[str, object]) -> Twist:
        cmd = Twist()
        if self.policy_mode == 'stop':
            return cmd

        target = self._select_target_detection(observation)
        if target is not None:
            bearing = float(target.get('bearing_rad', target.get('bearing', 0.0)))
            distance = self._optional_float(target.get('range_m', target.get('distance_m')))
            if distance is None or distance > self.target_stop_range_m:
                cmd.linear.x = self.target_linear_mps
            cmd.angular.z = self.target_angular_gain * bearing
        else:
            direction = 1.0 if (self.robot_ids.index(robot_id) % 2 == 0) else -1.0
            cmd.angular.z = direction * self.search_angular_radps

        cmd.linear.x = self._clamp(cmd.linear.x, -self.max_linear_mps, self.max_linear_mps)
        cmd.angular.z = self._clamp(cmd.angular.z, -self.max_angular_radps, self.max_angular_radps)
        return cmd

    def _select_target_detection(self, observation: Dict[str, object]) -> Optional[Dict[str, object]]:
        payload = observation.get('observation', {})
        if not isinstance(payload, dict):
            return None
        detections = payload.get('target_detections', [])
        if not isinstance(detections, list):
            return None

        candidates = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            score = float(detection.get('score', detection.get('confidence', 1.0)))
            if score < self.min_detection_score:
                continue
            candidates.append(detection)

        if not candidates:
            return None

        def _sort_key(item):
            distance = self._optional_float(item.get('range_m', item.get('distance_m')))
            bearing = abs(float(item.get('bearing_rad', item.get('bearing', 0.0))))
            if distance is None or not math.isfinite(distance):
                distance = 1e6
            return (distance, bearing)

        return sorted(candidates, key=_sort_key)[0]

    @staticmethod
    def _optional_float(value) -> Optional[float]:
        if value is None:
            return None
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(result):
            return None
        return result

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CentralControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
