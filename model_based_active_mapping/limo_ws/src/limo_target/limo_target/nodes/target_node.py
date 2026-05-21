import math
import random
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class ScanFrame:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


class TargetNode(Node):
    """
    Runs on each LIMO target robot.

    The target follows an autonomous random-walk motion model matching the
    near-constant-velocity Gaussian model used by the Kalman filter:
        pos_{k+1} = pos_k + v_k * tau + noise
        v_{k+1} = decay * v_k + sigma * random_unit

    This is implemented via SE(2) unicycle kinematics: maintain a desired
    heading derived from (vx, vy) and drive at the desired speed.
    Includes LiDAR-based safety stopping.
    """

    def __init__(self):
        super().__init__('limo_target')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('tau', 2.0)
        self.declare_parameter('max_linear_vel', 0.08)
        self.declare_parameter('max_angular_vel', 0.50)
        self.declare_parameter('vel_decay', 0.75)
        self.declare_parameter('vel_noise_sigma', 0.04)
        self.declare_parameter('kp_yaw', 1.20)
        self.declare_parameter('safety_stop_dist', 0.30)
        self.declare_parameter('safety_slow_dist', 0.55)
        self.declare_parameter('safety_forward_half_angle', 0.52)
        self.declare_parameter('seed', -1)

        rid = int(self.get_parameter('robot_id').value)
        self.robot_id = rid
        self.tau = float(self.get_parameter('tau').value)
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        self.vel_decay = float(self.get_parameter('vel_decay').value)
        self.vel_sigma = float(self.get_parameter('vel_noise_sigma').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.safety_stop = float(self.get_parameter('safety_stop_dist').value)
        self.safety_slow = float(self.get_parameter('safety_slow_dist').value)
        self.safety_fwd_half = float(self.get_parameter('safety_forward_half_angle').value)

        seed = int(self.get_parameter('seed').value)
        if seed >= 0:
            random.seed(seed + rid)
            np.random.seed(seed + rid)

        ns = f'target_{rid}'

        # Velocity state in Cartesian body frame
        self.vx = (random.random() - 0.5) * self.max_lin * 2.0
        self.vy = (random.random() - 0.5) * self.max_lin * 2.0

        self.odom: Optional[tuple] = None  # (x, y, yaw)
        self.scan: Optional[ScanFrame] = None
        self._recovering = False  # backing/turning flag after obstacle hit

        self.create_subscription(Odometry, f'/{ns}/odom', self._odom_cb, 20)
        self.create_subscription(LaserScan, f'/{ns}/scan', self._scan_cb, 20)

        self.cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        self.timer = self.create_timer(self.tau, self._loop)
        self.get_logger().info(f'TargetNode ready: robot_id={rid} tau={self.tau}s')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self.odom = (float(p.x), float(p.y), float(yaw))

    def _scan_cb(self, msg: LaserScan):
        self.scan = ScanFrame(
            ranges=np.asarray(msg.ranges, dtype=np.float32),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
        )

    def _loop(self):
        cmd = Twist()

        # Check for obstacles first
        obstacle, min_range, obstacle_bearing = self._check_obstacle()

        if obstacle:
            # Turn away from obstacle: rotate toward the clear half
            turn_dir = -1.0 if obstacle_bearing > 0 else 1.0
            cmd.linear.x = 0.0
            cmd.angular.z = turn_dir * self.max_ang
            self._recovering = True
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(f'Target {self.robot_id}: obstacle at {min_range:.2f}m, turning')
            return

        self._recovering = False

        # Evolve velocity via OU-like process (decayed random walk)
        self.vx = self.vel_decay * self.vx + np.random.normal(0.0, self.vel_sigma)
        self.vy = self.vel_decay * self.vy + np.random.normal(0.0, self.vel_sigma)

        # Clip speed to max
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_lin:
            self.vx *= self.max_lin / speed
            self.vy *= self.max_lin / speed
            speed = self.max_lin

        if speed < 1e-4:
            self.cmd_pub.publish(cmd)
            return

        desired_yaw = math.atan2(self.vy, self.vx)

        if self.odom is not None:
            _, _, current_yaw = self.odom
            yaw_err = self._wrap(desired_yaw - current_yaw)

            if abs(yaw_err) > math.pi * 0.6:
                # Large heading error: rotate in place
                cmd.linear.x = 0.0
                cmd.angular.z = float(np.clip(self.kp_yaw * yaw_err, -self.max_ang, self.max_ang))
            else:
                cmd.linear.x = float(speed * max(0.0, math.cos(yaw_err)))
                cmd.angular.z = float(np.clip(self.kp_yaw * yaw_err, -self.max_ang, self.max_ang))

        # Apply slow-zone scale
        if min_range is not None and min_range < self.safety_slow:
            scale = (min_range - self.safety_stop) / (self.safety_slow - self.safety_stop)
            scale = max(0.0, min(1.0, scale))
            cmd.linear.x *= scale

        self.cmd_pub.publish(cmd)

    def _check_obstacle(self):
        """Returns (obstacle_present, min_range, bearing_to_obstacle)."""
        scan = self.scan
        if scan is None or scan.ranges.size == 0:
            return False, None, 0.0

        min_r = float('inf')
        min_bearing = 0.0
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > self.safety_fwd_half:
                continue
            if r < scan.range_min:
                continue
            if r < min_r:
                min_r = r
                min_bearing = angle

        if not math.isfinite(min_r):
            return False, None, 0.0

        return min_r < self.safety_stop, min_r, min_bearing

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _wrap(v):
        return (v + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = TargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
