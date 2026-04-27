import math
import socket
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

from mbam_gazebo_tracking.core.network_utils import create_udp_socket
from mbam_gazebo_tracking.core.perception import (
    CameraCalibration,
    MultiTargetDetector,
    ScanFrame,
    project_detection_to_ground,
)
from mbam_gazebo_tracking.core.real_world_types import RobotReport, TargetDetection


@dataclass
class RobotPose:
    x: float
    y: float
    yaw: float
    frame_id: str


class LimoObservationReporter(Node):
    def __init__(self):
        super().__init__("limo_observer")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("report_transport_mode", "ros")
        self.declare_parameter("report_topic", "/mbam/robot_reports")
        self.declare_parameter("controller_host", "127.0.0.1")
        self.declare_parameter("controller_report_port", 15000)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("report_period_sec", 0.25)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("camera_height_m", 0.32)
        self.declare_parameter("camera_pitch_rad", 0.30)
        self.declare_parameter("camera_forward_offset_m", 0.0)
        self.declare_parameter("camera_lateral_offset_m", 0.0)
        self.declare_parameter("target_ground_z_m", 0.0)
        self.declare_parameter("min_ground_range_m", 0.35)
        self.declare_parameter("max_ground_range_m", 8.0)
        self.declare_parameter("enable_scan_fusion", True)
        self.declare_parameter("scan_window_half_width", 2)
        self.declare_parameter("scan_min_valid_range_m", 0.20)
        self.declare_parameter("scan_range_gate_m", 2.0)
        self.declare_parameter("enable_people_detection", True)
        self.declare_parameter("enable_robot_detection", True)
        self.declare_parameter("yolo_model_path", "")
        self.declare_parameter("yolo_device", "")
        self.declare_parameter("lower_body_crop_top_fraction", 0.35)
        self.declare_parameter("people_confidence_threshold", 0.35)
        self.declare_parameter("robot_min_area_px", 250)
        self.declare_parameter("max_detections", 12)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")

        robot_name = str(self.get_parameter("robot_name").value).strip()
        if not robot_name:
            robot_name = self.get_namespace().strip("/")
        if not robot_name:
            robot_name = "limo"
        self.robot_name = robot_name

        self.report_transport_mode = str(self.get_parameter("report_transport_mode").value).strip().lower()
        self.report_topic = str(self.get_parameter("report_topic").value)
        self.controller_host = str(self.get_parameter("controller_host").value)
        self.controller_report_port = int(self.get_parameter("controller_report_port").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.camera_height_m = float(self.get_parameter("camera_height_m").value)
        self.camera_pitch_rad = float(self.get_parameter("camera_pitch_rad").value)
        self.camera_forward_offset_m = float(self.get_parameter("camera_forward_offset_m").value)
        self.camera_lateral_offset_m = float(self.get_parameter("camera_lateral_offset_m").value)
        self.target_ground_z_m = float(self.get_parameter("target_ground_z_m").value)
        self.min_ground_range_m = float(self.get_parameter("min_ground_range_m").value)
        self.max_ground_range_m = float(self.get_parameter("max_ground_range_m").value)
        self.enable_scan_fusion = bool(self.get_parameter("enable_scan_fusion").value)
        self.scan_window_half_width = int(self.get_parameter("scan_window_half_width").value)
        self.scan_min_valid_range_m = float(self.get_parameter("scan_min_valid_range_m").value)
        self.scan_range_gate_m = float(self.get_parameter("scan_range_gate_m").value)
        self.report_period_sec = float(self.get_parameter("report_period_sec").value)

        self.detector = MultiTargetDetector(
            enable_people_detection=bool(self.get_parameter("enable_people_detection").value),
            enable_robot_detection=bool(self.get_parameter("enable_robot_detection").value),
            yolo_model_path=str(self.get_parameter("yolo_model_path").value),
            yolo_device=str(self.get_parameter("yolo_device").value),
            lower_body_crop_top_fraction=float(self.get_parameter("lower_body_crop_top_fraction").value),
            people_confidence_threshold=float(self.get_parameter("people_confidence_threshold").value),
            robot_min_area_px=int(self.get_parameter("robot_min_area_px").value),
            max_detections=int(self.get_parameter("max_detections").value),
            aruco_dictionary=str(self.get_parameter("aruco_dictionary").value),
        )

        self.latest_pose: Optional[RobotPose] = None
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_scan: Optional[ScanFrame] = None
        self.calibration: Optional[CameraCalibration] = None
        self.warn_times: Dict[str, float] = {}
        self.report_pub = None
        self.report_socket: Optional[socket.socket] = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        if self.enable_scan_fusion and scan_topic:
            self.create_subscription(LaserScan, scan_topic, self._scan_cb, 20)

        if self.report_transport_mode == "ros":
            self.report_pub = self.create_publisher(String, self.report_topic, 10)
        elif self.report_transport_mode == "udp":
            self.report_socket = create_udp_socket()
        else:
            raise ValueError(f"unsupported report_transport_mode: {self.report_transport_mode}")

        self.timer = self.create_timer(self.report_period_sec, self._process_and_publish)

        if self.report_transport_mode == "ros":
            self.get_logger().info(
                f"LIMO observer ready for robot='{self.robot_name}' report_topic='{self.report_topic}' "
                f"detectors={self.detector.describe_backends()}"
            )
        else:
            self.get_logger().info(
                f"LIMO observer ready for robot='{self.robot_name}' "
                f"udp_report_target={self.controller_host}:{self.controller_report_port} "
                f"detectors={self.detector.describe_backends()}"
            )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        frame_id = msg.header.frame_id.strip() or self.world_frame
        self.latest_pose = RobotPose(x=float(p.x), y=float(p.y), yaw=float(yaw), frame_id=frame_id)

    def _camera_info_cb(self, msg: CameraInfo):
        if msg.k[0] == 0.0 or msg.k[4] == 0.0:
            return
        self.calibration = CameraCalibration(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            width=int(msg.width),
            height=int(msg.height),
        )

    def _image_cb(self, msg: Image):
        channels = self._encoding_channels(msg.encoding)
        if channels <= 0:
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * channels
        if frame.size < expected:
            return
        frame = frame[:expected].reshape((msg.height, msg.width, channels))

        if msg.encoding in ("rgb8", "rgba8"):
            rgb = frame[:, :, :3]
        elif msg.encoding in ("bgr8", "bgra8"):
            rgb = frame[:, :, :3][:, :, ::-1]
        elif msg.encoding == "mono8":
            rgb = np.stack([frame[:, :, 0], frame[:, :, 0], frame[:, :, 0]], axis=-1)
        else:
            return

        self.latest_rgb = rgb.copy()

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = ScanFrame(
            ranges=np.asarray(msg.ranges, dtype=np.float32),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
        )

    def _process_and_publish(self):
        pose = self.latest_pose
        rgb = self.latest_rgb
        calibration = self.calibration
        if pose is None:
            self._warn_throttle("pose_ready", "Waiting for robot pose before publishing reports.")
            return

        if rgb is None or calibration is None:
            missing = []
            if rgb is None:
                missing.append("camera image")
            if calibration is None:
                missing.append("camera calibration")
            self._warn_throttle(
                "perception_ready",
                f"Publishing pose-only reports while waiting for: {', '.join(missing)}.",
            )
            detections = []
        else:
            detections = self.detector.detect(rgb)

        target_detections = []
        for det in detections:
            projection = project_detection_to_ground(
                detection=det,
                calibration=calibration,
                camera_height_m=self.camera_height_m,
                camera_pitch_rad=self.camera_pitch_rad,
                target_ground_z_m=self.target_ground_z_m,
                min_range_m=self.min_ground_range_m,
                max_range_m=self.max_ground_range_m,
                camera_forward_offset_m=self.camera_forward_offset_m,
                camera_lateral_offset_m=self.camera_lateral_offset_m,
                scan=self.latest_scan if self.enable_scan_fusion else None,
                scan_window_half_width=self.scan_window_half_width,
                scan_min_valid_range_m=self.scan_min_valid_range_m,
                scan_range_gate_m=self.scan_range_gate_m,
            )
            if projection is None:
                continue

            rel_x, rel_y, bearing, range_m = projection
            world_x = pose.x + rel_x * math.cos(pose.yaw) - rel_y * math.sin(pose.yaw)
            world_y = pose.y + rel_x * math.sin(pose.yaw) + rel_y * math.cos(pose.yaw)
            target_detections.append(
                TargetDetection(
                    label=det.label,
                    confidence=det.confidence,
                    world_x=world_x,
                    world_y=world_y,
                    robot_x=rel_x,
                    robot_y=rel_y,
                    bearing_rad=bearing,
                    range_m=range_m,
                    detector=det.detector,
                    bbox_xyxy=(det.x0, det.y0, det.x1, det.y1),
                )
            )

        stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        report = RobotReport(
            robot_name=self.robot_name,
            stamp_sec=stamp_sec,
            world_frame=pose.frame_id or self.world_frame,
            pose_x=pose.x,
            pose_y=pose.y,
            pose_yaw=pose.yaw,
            detections=target_detections,
        )

        if self.report_transport_mode == "ros":
            msg = String()
            msg.data = report.to_json()
            self.report_pub.publish(msg)
            return

        try:
            self.report_socket.sendto(
                report.to_json().encode("utf-8"),
                (self.controller_host, self.controller_report_port),
            )
        except OSError as exc:
            self._warn_throttle("udp_report_send", f"Failed to send UDP robot report: {exc}")

    def _warn_throttle(self, key: str, message: str, period_sec: float = 2.0):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        last_time = self.warn_times.get(key, 0.0)
        if now_sec - last_time >= period_sec:
            self.get_logger().warn(message)
            self.warn_times[key] = now_sec

    @staticmethod
    def _encoding_channels(encoding: str) -> int:
        if encoding in ("rgb8", "bgr8"):
            return 3
        if encoding in ("rgba8", "bgra8"):
            return 4
        if encoding == "mono8":
            return 1
        return 0

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LimoObservationReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
