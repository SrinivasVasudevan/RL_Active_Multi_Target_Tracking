import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from mbam_interfaces.msg import AgentObservation, TargetBeliefs


@dataclass
class ScanFrame:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


class AgentNode(Node):
    """
    Runs on each agent LIMO robot.

    Responsibilities:
      - Process camera (orange blob detection) + LiDAR to locate targets
      - Publish AgentObservation (world-frame detections) to central controller
      - Receive cmd_vel_desired from central controller, apply safety filter,
        and forward as cmd_vel to hardware
    """

    def __init__(self):
        super().__init__('limo_agent')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('max_num_targets', 7)
        self.declare_parameter('tau', 1.0)
        self.declare_parameter('camera_half_window_px', 8)
        self.declare_parameter('camera_min_pixels_per_col', 8)
        self.declare_parameter('lidar_index_half_window', 2)
        self.declare_parameter('lidar_range_gate', 1.5)
        self.declare_parameter('lidar_min_valid_range', 0.25)
        self.declare_parameter('fov_half_angle', 1.3963)   # ~80 deg, matches FoV psi
        self.declare_parameter('safety_stop_dist', 0.30)
        self.declare_parameter('safety_slow_dist', 0.55)
        self.declare_parameter('safety_forward_half_angle', 0.52)  # ~30 deg cone

        rid = int(self.get_parameter('robot_id').value)
        self.robot_id = rid
        self.max_num_targets = int(self.get_parameter('max_num_targets').value)
        tau = float(self.get_parameter('tau').value)
        self.cam_half_win = int(self.get_parameter('camera_half_window_px').value)
        self.cam_min_pix = int(self.get_parameter('camera_min_pixels_per_col').value)
        self.lidar_half_win = int(self.get_parameter('lidar_index_half_window').value)
        self.lidar_range_gate = float(self.get_parameter('lidar_range_gate').value)
        self.lidar_min_valid = float(self.get_parameter('lidar_min_valid_range').value)
        self.fov_half_angle = float(self.get_parameter('fov_half_angle').value)
        self.safety_stop = float(self.get_parameter('safety_stop_dist').value)
        self.safety_slow = float(self.get_parameter('safety_slow_dist').value)
        self.safety_fwd_half = float(self.get_parameter('safety_forward_half_angle').value)

        ns = f'agent_{rid}'

        # --- Sensor state ---
        self.odom: Optional[tuple] = None          # (x, y, yaw)
        self.scan: Optional[ScanFrame] = None
        self.camera_cols: Optional[np.ndarray] = None
        self.cam_width: int = 0
        self.cam_hfov: float = 1.3963

        # Latest belief from central controller
        self.beliefs: Optional[TargetBeliefs] = None

        # Latest desired velocity from central controller
        self.desired_cmd: Optional[Twist] = None

        # --- Subscribers ---
        self.create_subscription(Odometry, f'/{ns}/odom', self._odom_cb, 20)
        self.create_subscription(LaserScan, f'/{ns}/scan', self._scan_cb, 20)
        self.create_subscription(Image, f'/{ns}/camera/image_raw', self._image_cb, 10)
        self.create_subscription(CameraInfo, f'/{ns}/camera/camera_info', self._caminfo_cb, 10)
        self.create_subscription(TargetBeliefs, '/target_beliefs', self._beliefs_cb, 10)
        self.create_subscription(Twist, f'/{ns}/cmd_vel_desired', self._desired_cmd_cb, 10)

        # --- Publishers ---
        self.obs_pub = self.create_publisher(AgentObservation, f'/{ns}/observation', 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        self.timer = self.create_timer(tau, self._loop)
        self.get_logger().info(f'AgentNode ready: robot_id={rid}')

    # ---- Sensor callbacks -----------------------------------------------

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

    def _caminfo_cb(self, msg: CameraInfo):
        w = int(msg.width)
        fx = float(msg.k[0]) if len(msg.k) > 0 and msg.k[0] != 0.0 else 0.0
        if w > 0 and fx > 1e-6:
            self.cam_hfov = 2.0 * math.atan(w / (2.0 * fx))
            self.cam_width = w

    def _image_cb(self, msg: Image):
        ch = self._encoding_channels(msg.encoding)
        if ch <= 0:
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * ch
        if frame.size < expected:
            return
        frame = frame[:expected].reshape((msg.height, msg.width, ch))
        if msg.encoding in ('rgb8', 'rgba8'):
            rgb = frame[:, :, :3]
        elif msg.encoding in ('bgr8', 'bgra8'):
            rgb = frame[:, :, :3][:, :, ::-1]
        elif msg.encoding == 'mono8':
            rgb = np.stack([frame[:, :, 0]] * 3, axis=-1)
        else:
            return
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        # Orange target blob segmentation (HSV)
        lower = np.array([5, 90, 60], dtype=np.uint8)
        upper = np.array([25, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        col_scores = np.sum(mask > 0, axis=0)
        self.camera_cols = col_scores >= self.cam_min_pix
        self.cam_width = int(msg.width)

    def _beliefs_cb(self, msg: TargetBeliefs):
        self.beliefs = msg

    def _desired_cmd_cb(self, msg: Twist):
        self.desired_cmd = msg

    # ---- Main loop -------------------------------------------------------

    def _loop(self):
        # Publish sensor observations to central controller
        z_x, z_y, vis = self._fuse_observations()
        obs_msg = AgentObservation()
        obs_msg.header.stamp = self.get_clock().now().to_msg()
        obs_msg.robot_id = self.robot_id
        obs_msg.pose_x = float(self.odom[0]) if self.odom else 0.0
        obs_msg.pose_y = float(self.odom[1]) if self.odom else 0.0
        obs_msg.pose_yaw = float(self.odom[2]) if self.odom else 0.0
        obs_msg.max_num_targets = self.max_num_targets
        obs_msg.z_world_x = [float(v) for v in z_x]
        obs_msg.z_world_y = [float(v) for v in z_y]
        obs_msg.visible = [bool(v) for v in vis]
        self.obs_pub.publish(obs_msg)

        # Forward cmd_vel with safety filter
        cmd = Twist()
        if self.desired_cmd is not None:
            cmd.linear.x = self.desired_cmd.linear.x
            cmd.angular.z = self.desired_cmd.angular.z
        self._apply_safety(cmd)
        self.cmd_pub.publish(cmd)

    # ---- Sensor fusion ---------------------------------------------------

    def _fuse_observations(self):
        z_x = np.zeros(self.max_num_targets)
        z_y = np.zeros(self.max_num_targets)
        vis = np.zeros(self.max_num_targets, dtype=bool)

        if (self.beliefs is None or self.odom is None
                or self.scan is None or self.camera_cols is None):
            return z_x, z_y, vis

        rx, ry, ryaw = self.odom
        width = self.cam_width
        hfov = self.cam_hfov

        n = min(self.beliefs.max_num_targets, self.max_num_targets)
        for t in range(n):
            if not self.beliefs.active_mask[t]:
                continue
            tx = self.beliefs.mean_x[t]
            ty = self.beliefs.mean_y[t]

            dx, dy = tx - rx, ty - ry
            bearing = self._wrap(math.atan2(dy, dx) - ryaw)
            gt_dist = math.hypot(dx, dy)

            # FoV check
            if abs(bearing) > hfov * 0.5:
                continue

            # Camera blob check
            if width <= 1:
                continue
            col = int(round((bearing + hfov * 0.5) / hfov * (width - 1)))
            c0 = max(0, col - self.cam_half_win)
            c1 = min(width, col + self.cam_half_win + 1)
            if c0 >= c1 or not np.any(self.camera_cols[c0:c1]):
                continue

            # LiDAR range check (try both bearing signs for frame convention)
            lidar_r = self._lidar_at_bearing(bearing, gt_dist)
            if lidar_r is None:
                lidar_r = self._lidar_at_bearing(-bearing, gt_dist)
            if lidar_r is None or abs(lidar_r - gt_dist) > self.lidar_range_gate:
                continue

            # World-frame position from lidar range
            rel_x = lidar_r * math.cos(bearing)
            rel_y = lidar_r * math.sin(bearing)
            c_yaw, s_yaw = math.cos(ryaw), math.sin(ryaw)
            z_x[t] = rx + rel_x * c_yaw - rel_y * s_yaw
            z_y[t] = ry + rel_x * s_yaw + rel_y * c_yaw
            vis[t] = True

        return z_x, z_y, vis

    def _lidar_at_bearing(self, bearing: float, expected_dist: float) -> Optional[float]:
        scan = self.scan
        if scan is None or scan.angle_increment == 0.0 or scan.ranges.size == 0:
            return None
        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        i0 = max(0, idx - self.lidar_half_win)
        i1 = min(scan.ranges.size, idx + self.lidar_half_win + 1)
        if i0 >= i1:
            return None
        window = scan.ranges[i0:i1]
        valid = window[np.isfinite(window)]
        min_v = max(scan.range_min, self.lidar_min_valid)
        valid = valid[(valid >= min_v) & (valid <= scan.range_max)]
        if valid.size == 0:
            return None
        return float(valid[np.argmin(np.abs(valid - expected_dist))])

    # ---- Safety filter ---------------------------------------------------

    def _apply_safety(self, cmd: Twist):
        """Reduce or stop forward motion when an obstacle is too close."""
        scan = self.scan
        if scan is None or scan.ranges.size == 0:
            return
        # Extract forward sector
        fwd_idxs = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= self.safety_fwd_half:
                fwd_idxs.append(r)
        if not fwd_idxs:
            return
        fwd_ranges = np.array(fwd_idxs, dtype=np.float32)
        valid = fwd_ranges[np.isfinite(fwd_ranges) & (fwd_ranges > scan.range_min)]
        if valid.size == 0:
            return
        min_range = float(valid.min())

        if min_range < self.safety_stop:
            # Full stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif min_range < self.safety_slow:
            # Scale down linearly
            scale = (min_range - self.safety_stop) / (self.safety_slow - self.safety_stop)
            cmd.linear.x *= scale

    # ---- Utilities -------------------------------------------------------

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _wrap(v):
        return (v + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _encoding_channels(enc: str) -> int:
        if enc in ('rgb8', 'bgr8'):
            return 3
        if enc in ('rgba8', 'bgra8'):
            return 4
        if enc == 'mono8':
            return 1
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
