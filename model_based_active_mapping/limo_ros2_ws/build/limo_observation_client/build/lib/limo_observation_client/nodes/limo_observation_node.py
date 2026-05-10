import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LimoObservationNode(Node):
    """Publish LIMO observations and safely execute controller velocity commands."""

    def __init__(self) -> None:
        super().__init__('limo_observation_node')

        self.declare_parameter('robot_id', 'limo_1')
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('observation_topic', 'local_observation')
        self.declare_parameter('ack_topic', 'observation_ack')
        self.declare_parameter('controller_cmd_vel_topic', 'controller_cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_detection_topic', 'target_detections')
        self.declare_parameter('cmd_timeout_sec', 0.6)
        self.declare_parameter('require_scan_for_motion', True)
        self.declare_parameter('test_speed_scale', 0.35)
        self.declare_parameter('max_linear_mps', 0.18)
        self.declare_parameter('max_angular_radps', 0.55)
        self.declare_parameter('front_stop_distance_m', 0.55)
        self.declare_parameter('rear_stop_distance_m', 0.35)
        self.declare_parameter('side_stop_distance_m', 0.30)
        self.declare_parameter('slow_distance_m', 1.10)
        self.declare_parameter('min_valid_scan_range_m', 0.05)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        observation_topic = str(self.get_parameter('observation_topic').value)
        ack_topic = str(self.get_parameter('ack_topic').value)
        controller_cmd_vel_topic = str(self.get_parameter('controller_cmd_vel_topic').value)
        output_cmd_vel_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        target_detection_topic = str(self.get_parameter('target_detection_topic').value)

        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.require_scan_for_motion = bool(self.get_parameter('require_scan_for_motion').value)
        self.test_speed_scale = float(self.get_parameter('test_speed_scale').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.front_stop_distance_m = float(self.get_parameter('front_stop_distance_m').value)
        self.rear_stop_distance_m = float(self.get_parameter('rear_stop_distance_m').value)
        self.side_stop_distance_m = float(self.get_parameter('side_stop_distance_m').value)
        self.slow_distance_m = float(self.get_parameter('slow_distance_m').value)
        self.min_valid_scan_range_m = float(self.get_parameter('min_valid_scan_range_m').value)

        self.seq = 0
        self.last_ack_seq = None
        self.last_odom: Optional[Odometry] = None
        self.last_scan: Optional[LaserScan] = None
        self.last_target_detections: List[Dict[str, object]] = []
        self.last_controller_cmd = Twist()
        self.last_controller_cmd_time = None
        self.last_safety_status: Dict[str, object] = {'state': 'waiting_for_scan'}

        self.observation_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/{observation_topic}',
            10,
        )
        self.safety_status_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/safety_status',
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, output_cmd_vel_topic, 10)

        self.create_subscription(
            String,
            f'/{self.robot_id}/{ack_topic}',
            self._ack_callback,
            10,
        )
        self.create_subscription(
            Twist,
            f'/{self.robot_id}/{controller_cmd_vel_topic}',
            self._controller_cmd_callback,
            10,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 20)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            f'/{self.robot_id}/{target_detection_topic}',
            self._target_detection_callback,
            10,
        )

        self.create_timer(self.publish_period_sec, self._publish_observation)
        self.create_timer(0.05, self._publish_safe_cmd)

        self.get_logger().info(
            f'{self.robot_id} publishes /{self.robot_id}/{observation_topic}, '
            f'listens on /{self.robot_id}/{controller_cmd_vel_topic}, '
            f'and publishes safe commands to {output_cmd_vel_topic}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _target_detection_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Ignoring malformed target detections: {exc}')
            return

        detections = payload.get('detections', payload)
        if isinstance(detections, list):
            self.last_target_detections = detections

    def _controller_cmd_callback(self, msg: Twist) -> None:
        self.last_controller_cmd = msg
        self.last_controller_cmd_time = self.get_clock().now()

    def _publish_observation(self) -> None:
        now = self.get_clock().now().to_msg()
        observation = {
            'robot_id': self.robot_id,
            'seq': self.seq,
            'stamp': {
                'sec': now.sec,
                'nanosec': now.nanosec,
            },
            'observation': self._read_local_observation(),
            'safety': self.last_safety_status,
        }

        self.observation_pub.publish(String(data=json.dumps(observation)))
        self.seq += 1

    def _read_local_observation(self) -> Dict[str, object]:
        return {
            'source': 'limo_safety_gateway',
            'pose': self._pose_dict(),
            'scan_summary': self._scan_summary(),
            'target_detections': self.last_target_detections,
        }

    def _pose_dict(self) -> Optional[Dict[str, float]]:
        if self.last_odom is None:
            return None
        pose = self.last_odom.pose.pose
        q = pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        return {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': float(yaw),
        }

    def _scan_summary(self) -> Dict[str, Optional[float]]:
        if self.last_scan is None:
            return {
                'front_min_m': None,
                'left_min_m': None,
                'right_min_m': None,
                'rear_min_m': None,
            }
        return {
            'front_min_m': self._sector_min(self.last_scan, -0.45, 0.45),
            'left_min_m': self._sector_min(self.last_scan, 0.45, 1.57),
            'right_min_m': self._sector_min(self.last_scan, -1.57, -0.45),
            'rear_min_m': self._sector_min(self.last_scan, 2.55, -2.55),
        }

    def _publish_safe_cmd(self) -> None:
        safe_cmd, status = self._limited_command()
        self.last_safety_status = status
        self.cmd_vel_pub.publish(safe_cmd)
        self.safety_status_pub.publish(String(data=json.dumps(status)))

    def _limited_command(self) -> Tuple[Twist, Dict[str, object]]:
        cmd = Twist()
        status = {
            'state': 'ok',
            'reason': '',
            'scale': self.test_speed_scale,
            'front_min_m': None,
            'left_min_m': None,
            'right_min_m': None,
            'rear_min_m': None,
        }

        if self.last_controller_cmd_time is None:
            status['state'] = 'stopped'
            status['reason'] = 'waiting_for_controller_command'
            return cmd, status

        age = (self.get_clock().now() - self.last_controller_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout_sec:
            status['state'] = 'stopped'
            status['reason'] = 'controller_command_timeout'
            return cmd, status

        raw_linear = self._clamp(
            float(self.last_controller_cmd.linear.x),
            -self.max_linear_mps,
            self.max_linear_mps,
        )
        raw_angular = self._clamp(
            float(self.last_controller_cmd.angular.z),
            -self.max_angular_radps,
            self.max_angular_radps,
        )

        scan_summary = self._scan_summary()
        status.update(scan_summary)
        if self.last_scan is None and self.require_scan_for_motion:
            status['state'] = 'stopped'
            status['reason'] = 'scan_required'
            return cmd, status

        scale = self._clamp(self.test_speed_scale, 0.0, 1.0)
        front_min = scan_summary['front_min_m']
        rear_min = scan_summary['rear_min_m']
        left_min = scan_summary['left_min_m']
        right_min = scan_summary['right_min_m']

        if raw_linear > 0.0 and front_min is not None:
            if front_min <= self.front_stop_distance_m:
                raw_linear = 0.0
                status['state'] = 'stopped'
                status['reason'] = 'front_obstacle'
            elif front_min < self.slow_distance_m:
                scale *= max(0.15, (front_min - self.front_stop_distance_m) / (self.slow_distance_m - self.front_stop_distance_m))
                status['state'] = 'slowed'
                status['reason'] = 'front_obstacle_near'

        if raw_linear < 0.0 and rear_min is not None and rear_min <= self.rear_stop_distance_m:
            raw_linear = 0.0
            status['state'] = 'stopped'
            status['reason'] = 'rear_obstacle'

        if raw_angular > 0.0 and left_min is not None and left_min <= self.side_stop_distance_m:
            raw_angular = 0.0
            status['state'] = 'limited'
            status['reason'] = 'left_obstacle'
        elif raw_angular < 0.0 and right_min is not None and right_min <= self.side_stop_distance_m:
            raw_angular = 0.0
            status['state'] = 'limited'
            status['reason'] = 'right_obstacle'

        cmd.linear.x = raw_linear * scale
        cmd.angular.z = raw_angular * scale
        status['scale'] = float(scale)
        status['cmd_linear_x'] = float(cmd.linear.x)
        status['cmd_angular_z'] = float(cmd.angular.z)
        return cmd, status

    def _sector_min(self, scan: LaserScan, start_angle: float, end_angle: float) -> Optional[float]:
        values = []
        for idx, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < max(scan.range_min, self.min_valid_scan_range_m) or distance > scan.range_max:
                continue
            angle = scan.angle_min + idx * scan.angle_increment
            if self._angle_in_sector(angle, start_angle, end_angle):
                values.append(float(distance))
        if not values:
            return None
        return min(values)

    @staticmethod
    def _angle_in_sector(angle: float, start_angle: float, end_angle: float) -> bool:
        angle = math.atan2(math.sin(angle), math.cos(angle))
        start_angle = math.atan2(math.sin(start_angle), math.cos(start_angle))
        end_angle = math.atan2(math.sin(end_angle), math.cos(end_angle))
        if start_angle <= end_angle:
            return start_angle <= angle <= end_angle
        return angle >= start_angle or angle <= end_angle

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _ack_callback(self, msg: String) -> None:
        try:
            ack = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Ignoring malformed ack: {exc}')
            return

        if ack.get('robot_id') != self.robot_id:
            self.get_logger().warning(
                f"Ignoring ack for robot_id={ack.get('robot_id')!r}"
            )
            return

        self.last_ack_seq = ack.get('observation_seq')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
