import json
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CentralControllerNode(Node):
    """Receive local observations from known robots and publish per-robot acks."""

    def __init__(self) -> None:
        super().__init__('central_controller')

        self.declare_parameter('robot_ids', 'limo_1,limo_2')
        self.declare_parameter('observation_topic', 'local_observation')
        self.declare_parameter('ack_topic', 'observation_ack')

        self.robot_ids = self._get_robot_ids()
        observation_topic = self.get_parameter('observation_topic').value
        ack_topic = self.get_parameter('ack_topic').value

        self._ack_publishers: Dict[str, rclpy.publisher.Publisher] = {}

        for robot_id in self.robot_ids:
            observation_name = f'/{robot_id}/{observation_topic}'
            ack_name = f'/{robot_id}/{ack_topic}'

            self.create_subscription(
                String,
                observation_name,
                self._make_observation_callback(robot_id, ack_name),
                10,
            )
            self._ack_publishers[robot_id] = self.create_publisher(String, ack_name, 10)

        self.get_logger().info(
            'Central controller ready for robots: ' + ', '.join(self.robot_ids)
        )

    def _get_robot_ids(self) -> List[str]:
        value = self.get_parameter('robot_ids').value
        if isinstance(value, str):
            return [item.strip() for item in value.split(',') if item.strip()]
        return [str(item) for item in value]

    def _make_observation_callback(self, expected_robot_id: str, ack_topic: str):
        def callback(msg: String) -> None:
            received_at = self.get_clock().now().to_msg()
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

            ack = {
                'robot_id': expected_robot_id,
                'acknowledged': True,
                'ack_message': 'local observation received',
                'observation_seq': observation.get('seq'),
                'observation_stamp': observation.get('stamp'),
                'controller_stamp': {
                    'sec': received_at.sec,
                    'nanosec': received_at.nanosec,
                },
            }

            self._ack_publishers[expected_robot_id].publish(String(data=json.dumps(ack)))
            self.get_logger().info(
                f"ACK sent to {expected_robot_id} on {ack_topic} for seq={ack['observation_seq']}"
            )

        return callback


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CentralControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
