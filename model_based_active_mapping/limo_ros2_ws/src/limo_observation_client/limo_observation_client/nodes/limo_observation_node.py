import json
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LimoObservationNode(Node):
    """Publish a simple local observation and log the controller ack for this robot."""

    def __init__(self) -> None:
        super().__init__('limo_observation_node')

        self.declare_parameter('robot_id', 'limo_1')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('observation_topic', 'local_observation')
        self.declare_parameter('ack_topic', 'observation_ack')

        self.robot_id = str(self.get_parameter('robot_id').value)
        publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        observation_topic = str(self.get_parameter('observation_topic').value)
        ack_topic = str(self.get_parameter('ack_topic').value)

        self.seq = 0
        self.last_ack_seq = None

        self.observation_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/{observation_topic}',
            10,
        )
        self.create_subscription(
            String,
            f'/{self.robot_id}/{ack_topic}',
            self._ack_callback,
            10,
        )
        self.create_timer(publish_period_sec, self._publish_observation)

        self.get_logger().info(
            f'{self.robot_id} publishing /{self.robot_id}/{observation_topic} '
            f'and waiting for /{self.robot_id}/{ack_topic}'
        )

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
        }

        self.observation_pub.publish(String(data=json.dumps(observation)))
        self.get_logger().info(f'Published local observation seq={self.seq}')
        self.seq += 1

    def _read_local_observation(self) -> Dict[str, object]:
        # Replace this placeholder with LIMO odom, lidar, camera, or fused local state.
        return {
            'source': 'network_smoke_test',
            'status': 'ok',
        }

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
        self.get_logger().info(
            f"Received controller ack for seq={self.last_ack_seq}: {ack.get('ack_message')}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
