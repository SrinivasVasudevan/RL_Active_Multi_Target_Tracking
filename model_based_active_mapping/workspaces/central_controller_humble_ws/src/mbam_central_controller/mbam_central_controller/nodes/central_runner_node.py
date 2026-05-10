"""Central controller for real LIMO agents: policy + fusion + target velocity broadcast.

Adapted from mbam_gazebo_tracking episode_runner_node (Gazebo-specific pieces removed).
Target poses for lidar gating come from each target's SLAM odometry. Target motion is
synchronized by publishing planar world-frame velocities that mbam_limo_target consumes.
"""
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from mbam_central_controller.core.episode_sampler import EpisodeSampler
from mbam_central_controller.core.model_based_agent_att_ros import ModelBasedAgentAttRos
from mbam_central_controller.core.tracking_stats import TrackingStatistics, aggregate_by_target_count


@dataclass
class ScanFrame:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass
class AgentPose:
    x: float
    y: float
    yaw: float


class MBAMCentralRunner(Node):
    def __init__(self):
        super().__init__("mbam_central_runner")

        pkg_share = get_package_share_directory("mbam_central_controller")

        default_params = os.path.join(pkg_share, "config", "params_compare.yaml")
        default_ckpt = os.path.join(pkg_share, "checkpoints", "best_model_seed42.pth")

        self.declare_parameter("params_file", default_params)
        self.declare_parameter("model_path", default_ckpt)
        self.declare_parameter("seed", 42)
        self.declare_parameter("network_type", 1)
        self.declare_parameter("num_robots", 2)
        self.declare_parameter("max_num_landmarks", 7)
        self.declare_parameter("num_clusters", 2)
        self.declare_parameter("num_test_trials", -1)
        self.declare_parameter("output_dir", os.path.join(os.getcwd(), "mbam_real_test_results"))
        self.declare_parameter("camera_half_window_px", 8)
        self.declare_parameter("camera_min_pixels_per_col", 8)
        self.declare_parameter("lidar_index_half_window", 2)
        self.declare_parameter("lidar_range_gate", 1.5)
        self.declare_parameter("lidar_min_valid_range", 0.25)
        self.declare_parameter("marker_frame", "map")
        self.declare_parameter("debug_sensor_fusion", False)
        self.declare_parameter("fixed_num_landmarks", 5)
        self.declare_parameter("fixed_horizon", 250)
        self.declare_parameter("use_random_episode_schedule", False)
        self.declare_parameter("agent_odom_topic_template", "agent_{id}/odom")
        self.declare_parameter("agent_scan_topic_template", "agent_{id}/scan")
        self.declare_parameter("agent_camera_topic_template", "agent_{id}/camera/image_raw")
        self.declare_parameter("agent_camera_info_topic_template", "agent_{id}/camera/camera_info")
        self.declare_parameter("agent_cmd_topic_template", "agent_{id}/cmd_vel")
        self.declare_parameter("target_odom_topic_template", "target_{id}/odom")
        self.declare_parameter(
            "target_world_vel_topic_template", "mbam/target_{id}/desired_world_velocity"
        )
        self.declare_parameter("velocity_stamp_frame", "odom")
        self.declare_parameter("publish_target_velocities", True)
        self.declare_parameter("exclude_agent_green_sticky", True)
        self.declare_parameter("green_hsv_lower", [35, 40, 40])
        self.declare_parameter("green_hsv_upper", [95, 255, 255])
        self.declare_parameter("orange_hsv_lower", [5, 90, 60])
        self.declare_parameter("orange_hsv_upper", [25, 255, 255])

        params_file = self.get_parameter("params_file").get_parameter_value().string_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.seed = int(self.get_parameter("seed").value)
        self.network_type = int(self.get_parameter("network_type").value)
        self.num_robots = int(self.get_parameter("num_robots").value)
        self.max_num_landmarks = int(self.get_parameter("max_num_landmarks").value)
        self.num_clusters = int(self.get_parameter("num_clusters").value)
        self.num_test_trials_override = int(self.get_parameter("num_test_trials").value)
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.camera_half_window_px = int(self.get_parameter("camera_half_window_px").value)
        self.camera_min_pixels_per_col = int(self.get_parameter("camera_min_pixels_per_col").value)
        self.lidar_index_half_window = int(self.get_parameter("lidar_index_half_window").value)
        self.lidar_range_gate = float(self.get_parameter("lidar_range_gate").value)
        self.lidar_min_valid_range = float(self.get_parameter("lidar_min_valid_range").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.debug_sensor_fusion = bool(self.get_parameter("debug_sensor_fusion").value)
        self.fixed_num_landmarks = int(self.get_parameter("fixed_num_landmarks").value)
        self.fixed_horizon = int(self.get_parameter("fixed_horizon").value)
        self.use_random_episode_schedule = bool(self.get_parameter("use_random_episode_schedule").value)
        self.agent_odom_topic_template = str(self.get_parameter("agent_odom_topic_template").value)
        self.agent_scan_topic_template = str(self.get_parameter("agent_scan_topic_template").value)
        self.agent_camera_topic_template = str(self.get_parameter("agent_camera_topic_template").value)
        self.agent_camera_info_topic_template = str(
            self.get_parameter("agent_camera_info_topic_template").value
        )
        self.agent_cmd_topic_template = str(self.get_parameter("agent_cmd_topic_template").value)
        self.target_odom_topic_template = str(self.get_parameter("target_odom_topic_template").value)
        self.target_world_vel_topic_template = str(
            self.get_parameter("target_world_vel_topic_template").value
        )
        self.velocity_stamp_frame = str(self.get_parameter("velocity_stamp_frame").value)
        self.publish_target_velocities = bool(self.get_parameter("publish_target_velocities").value)
        self.exclude_agent_green_sticky = bool(self.get_parameter("exclude_agent_green_sticky").value)
        self.green_hsv_lower = list(self.get_parameter("green_hsv_lower").value)
        self.green_hsv_upper = list(self.get_parameter("green_hsv_upper").value)
        self.orange_hsv_lower = list(self.get_parameter("orange_hsv_lower").value)
        self.orange_hsv_upper = list(self.get_parameter("orange_hsv_upper").value)

        if not os.path.exists(params_file):
            raise FileNotFoundError(f"params file not found: {params_file}")
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"model checkpoint not found: {model_path}\n"
                "Copy best_model_seed42.pth from mbam_gazebo_tracking into "
                "share/mbam_central_controller/checkpoints/ after build, or set model_path."
            )

        torch.manual_seed(self.seed)
        np.random.seed(self.seed)

        with open(params_file, "r", encoding="utf-8") as f:
            params = yaml.load(f, Loader=yaml.FullLoader)

        self.params = params
        self.model_path = model_path

        self._init_core_components()

        self.agent_cmd_pubs = []
        for i in range(self.num_robots):
            topic = self.agent_cmd_topic_template.format(id=i)
            self.agent_cmd_pubs.append(self.create_publisher(Twist, topic, 10))

        self.target_vel_pubs = []
        for t in range(self.max_num_landmarks):
            topic = self.target_world_vel_topic_template.format(id=t)
            self.target_vel_pubs.append(self.create_publisher(Vector3Stamped, topic, 10))

        self.marker_pub = self.create_publisher(MarkerArray, "/mbam/markers", 10)

        self.odom_by_robot: List[Optional[AgentPose]] = [None] * self.num_robots
        self.scan_by_robot: List[Optional[ScanFrame]] = [None] * self.num_robots
        self.camera_cols_by_robot: List[Optional[np.ndarray]] = [None] * self.num_robots
        self.camera_width_by_robot: List[int] = [0] * self.num_robots
        self.camera_hfov_by_robot: List[float] = [1.39626] * self.num_robots

        for i in range(self.num_robots):
            odom_topic = self.agent_odom_topic_template.format(id=i)
            scan_topic = self.agent_scan_topic_template.format(id=i)
            cam_topic = self.agent_camera_topic_template.format(id=i)
            cam_info_topic = self.agent_camera_info_topic_template.format(id=i)
            self.create_subscription(Odometry, odom_topic, self._make_odom_cb(i), 20)
            self.create_subscription(LaserScan, scan_topic, self._make_scan_cb(i), 20)
            self.create_subscription(Image, cam_topic, self._make_image_cb(i), 10)
            self.create_subscription(CameraInfo, cam_info_topic, self._make_camera_info_cb(i), 10)

        self.target_odom_pose: List[Optional[AgentPose]] = [None] * self.max_num_landmarks
        for t in range(self.max_num_landmarks):
            todom = self.target_odom_topic_template.format(id=t)
            self.create_subscription(Odometry, todom, self._make_target_odom_cb(t), 20)

        self.target_names = [f"target_{i}" for i in range(self.max_num_landmarks)]

        self.active_episode = False
        self.ready = False
        self.completed = False

        self.episode_index = 0
        self.step_index = 0
        self.horizon = 0
        self.num_landmarks = 0
        self.mu_real: Optional[torch.Tensor] = None
        self.v: Optional[torch.Tensor] = None
        self.landmark_motion_bias: Optional[torch.Tensor] = None
        self.stats = None
        self.episode_targets_tracked: List[int] = []

        self.all_trail_summaries: List[Dict] = []
        self.overall_avg_targets: List[float] = []
        self.results_start_time = time.strftime("%Y%m%d_%H%M%S")

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info("MBAM central runner initialized (real-robot mode)")
        self.get_logger().info(f"params_file={params_file}")
        self.get_logger().info(f"model_path={model_path}")

        self.timer = self.create_timer(self.tau, self._control_loop)

    def _init_core_components(self):
        max_num_landmarks = (
            self.max_num_landmarks if self.max_num_landmarks > 0 else int(self.params["max_num_landmarks"])
        )
        tau = float(self.params["tau"])

        a = torch.zeros((2, 2), dtype=torch.float32)
        a[0, 0] = self.params["motion"]["A"]["_1"]
        a[1, 1] = self.params["motion"]["A"]["_2"]

        b = torch.zeros((2, 2), dtype=torch.float32)
        b[0, 0] = self.params["motion"]["B"]["_1"]
        b[1, 1] = self.params["motion"]["B"]["_2"]

        w = torch.zeros(2, dtype=torch.float32)
        w[0] = self.params["motion"]["W"]["_1"]
        w[1] = self.params["motion"]["W"]["_2"]

        landmark_motion_scale = float(self.params["motion"]["landmark_motion_scale"])

        init_info = float(self.params["init_info"])

        radius = float(self.params["FoV"]["radius"])
        psi = torch.tensor([float(self.params["FoV"]["psi"])], dtype=torch.float32)
        kappa = float(self.params["FoV"]["kappa"])

        v_noise = torch.zeros(2, dtype=torch.float32)
        v_noise[0] = self.params["FoV"]["V"]["_1"]
        v_noise[1] = self.params["FoV"]["V"]["_2"]

        lr = float(self.params["lr"])
        num_test_trials = int(self.params["num_test_trials"])

        if self.num_test_trials_override > 0:
            num_test_trials = self.num_test_trials_override

        if self.network_type != 1:
            self.get_logger().warn("Only attention policy is supported. Using network_type=1.")

        self.max_num_landmarks = max_num_landmarks
        self.num_test_trials = num_test_trials
        self.tau = tau
        self.a = a
        self.b = b
        self.w = w
        self.landmark_motion_scale = landmark_motion_scale
        self.radius = radius
        self.psi = psi
        self.kappa = kappa
        self.v_noise = v_noise

        self.agent = ModelBasedAgentAttRos(
            max_num_landmarks=max_num_landmarks,
            init_info=init_info,
            a=a,
            b=b,
            w=w,
            radius=radius,
            psi=psi,
            kappa=kappa,
            v_noise=v_noise,
            lr=lr,
            num_robots=self.num_robots,
        )
        self.agent.load_policy_state_dict(self.model_path)
        self.agent.eval_policy()

        self.sampler = EpisodeSampler(
            max_num_landmarks=max_num_landmarks,
            num_robots=self.num_robots,
            landmark_motion_scale=landmark_motion_scale,
            num_clusters=self.num_clusters,
        )

    def _make_odom_cb(self, idx: int):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
            self.odom_by_robot[idx] = AgentPose(x=float(p.x), y=float(p.y), yaw=float(yaw))

        return _cb

    def _make_target_odom_cb(self, idx: int):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
            self.target_odom_pose[idx] = AgentPose(x=float(p.x), y=float(p.y), yaw=float(yaw))

        return _cb

    def _make_scan_cb(self, idx: int):
        def _cb(msg: LaserScan):
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            self.scan_by_robot[idx] = ScanFrame(
                ranges=ranges,
                angle_min=float(msg.angle_min),
                angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min),
                range_max=float(msg.range_max),
            )

        return _cb

    def _make_camera_info_cb(self, idx: int):
        def _cb(msg: CameraInfo):
            width = int(msg.width)
            fx = float(msg.k[0]) if len(msg.k) > 0 and msg.k[0] != 0.0 else 0.0
            if width > 0 and fx > 1e-6:
                hfov = 2.0 * math.atan(width / (2.0 * fx))
                self.camera_hfov_by_robot[idx] = float(hfov)
                self.camera_width_by_robot[idx] = width

        return _cb

    def _make_image_cb(self, idx: int):
        def _cb(msg: Image):
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

            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

            lower_o = np.array(self.orange_hsv_lower, dtype=np.uint8)
            upper_o = np.array(self.orange_hsv_upper, dtype=np.uint8)
            mask_o = cv2.inRange(hsv, lower_o, upper_o)

            if self.exclude_agent_green_sticky:
                lower_g = np.array(self.green_hsv_lower, dtype=np.uint8)
                upper_g = np.array(self.green_hsv_upper, dtype=np.uint8)
                mask_g = cv2.inRange(hsv, lower_g, upper_g)
                mask = cv2.bitwise_and(mask_o, cv2.bitwise_not(mask_g))
            else:
                mask = mask_o

            col_scores = np.sum(mask > 0, axis=0)
            visible_cols = col_scores >= self.camera_min_pixels_per_col

            self.camera_cols_by_robot[idx] = visible_cols
            self.camera_width_by_robot[idx] = int(msg.width)

        return _cb

    def _sync_mu_real_from_targets(self):
        assert self.mu_real is not None
        for t in range(self.num_landmarks):
            pose = self.target_odom_pose[t]
            if pose is not None:
                self.mu_real[t, 0] = float(pose.x)
                self.mu_real[t, 1] = float(pose.y)

    def _control_loop(self):
        if self.completed:
            return

        if not self.ready:
            if not self._agents_ready():
                self.get_logger().warn("Waiting for all agent odometry streams...", throttle_duration_sec=5.0)
                return
            if not self._targets_ready():
                self.get_logger().warn("Waiting for all target odometry (SLAM) streams...", throttle_duration_sec=5.0)
                return

            self.ready = True
            self.get_logger().info("All agents and targets reporting pose. Starting episodes.")
            self._start_next_episode()
            return

        if not self.active_episode:
            return

        self._sync_mu_real_from_targets()

        x = self._current_robot_state_tensor()
        if x is None:
            self.get_logger().warn("Robot odometry not ready for all agents; skipping this tick.")
            return

        action = self.agent.plan(self.v, x)
        self._publish_actions(action)
        self._publish_target_world_velocities()

        z_world, visible = self._fuse_camera_lidar_observations(x)
        self.agent.update_info_from_observations(z_world, visible, x)

        self.v = self.sampler.rollout_landmark_velocity(self.num_landmarks, self.landmark_motion_bias)

        self.stats.update(visible)
        self.episode_targets_tracked.append(int(visible.any(dim=0).sum().item()))
        self._publish_markers(x, visible)

        self.step_index += 1
        if self.step_index >= self.horizon:
            self._finish_episode()

    def _agents_ready(self) -> bool:
        return all(self.odom_by_robot[i] is not None for i in range(self.num_robots))

    def _targets_ready(self) -> bool:
        n = self.fixed_num_landmarks if not self.use_random_episode_schedule else self.max_num_landmarks
        return all(self.target_odom_pose[t] is not None for t in range(n))

    def _start_next_episode(self):
        if self.episode_index >= self.num_test_trials:
            self._finalize_all_results()
            self.completed = True
            self._publish_zero_velocities()
            self._publish_zero_target_velocities()
            self.get_logger().info("All episodes complete.")
            return

        if self.use_random_episode_schedule:
            sampled = self.sampler.sample()
            self.num_landmarks = sampled.num_landmarks
            self.horizon = sampled.horizon
            self.mu_real = torch.zeros((self.num_landmarks, 2), dtype=torch.float32)
            self._sync_mu_real_from_targets()
            self.v = sampled.v.clone()
            self.landmark_motion_bias = sampled.landmark_motion_bias.clone()
        else:
            self.num_landmarks = min(self.fixed_num_landmarks, self.max_num_landmarks)
            self.horizon = self.fixed_horizon
            self.mu_real = torch.zeros((self.num_landmarks, 2), dtype=torch.float32)
            self._sync_mu_real_from_targets()
            self.landmark_motion_bias = (torch.rand(2) - 0.5) * 1.6
            self.v = (
                (torch.rand((self.num_landmarks, 2)) - 0.5) * self.landmark_motion_scale
                + self.landmark_motion_bias
            )

        self.agent.reset_estimate_mu(self.mu_real)
        self.agent.reset_agent_info()

        self.stats = TrackingStatistics(num_robots=self.num_robots, num_landmarks=self.num_landmarks)
        self.episode_targets_tracked = []
        self.step_index = 0
        self.episode_index += 1
        self.active_episode = True
        self._clear_markers()

        self.get_logger().info(
            f"Episode {self.episode_index}/{self.num_test_trials} started: "
            f"targets={self.num_landmarks}, horizon={self.horizon}"
        )

    def _finish_episode(self):
        self.active_episode = False

        reward = self.agent.update_policy_grad(False) / max(self.num_landmarks, 1)
        trail_summary = self.stats.get_trail_summary()
        trail_summary["episode"] = self.episode_index
        self.all_trail_summaries.append(trail_summary)

        avg_targets = np.mean(self.episode_targets_tracked) if self.episode_targets_tracked else 0.0
        self.overall_avg_targets.append(float(avg_targets))

        self.get_logger().info(
            f"Episode {self.episode_index} complete: objective_per_target={reward:.4f}, "
            f"avg_unique_targets_tracked={avg_targets:.2f}"
        )

        self._log_trail_summary(trail_summary)

        self._publish_zero_velocities()
        self._publish_zero_target_velocities()
        self._start_next_episode()

    def _finalize_all_results(self):
        aggregated = aggregate_by_target_count(self.all_trail_summaries)

        legacy_mean = float(np.mean(self.overall_avg_targets)) if self.overall_avg_targets else 0.0
        legacy_std = float(np.std(self.overall_avg_targets)) if self.overall_avg_targets else 0.0

        model_name = os.path.splitext(os.path.basename(self.model_path))[0]
        json_results = {
            "model_path": self.model_path,
            "model_name": model_name,
            "seed": self.seed,
            "network_type": 1,
            "num_robots": self.num_robots,
            "num_test_trials": self.num_test_trials,
            "trails": self.all_trail_summaries,
            "aggregated_by_targets": {str(k): v for k, v in aggregated.items()},
            "legacy_summary": {
                "mean_targets_tracked": round(legacy_mean, 4),
                "std_targets_tracked": round(legacy_std, 4),
            },
            "sensor_mode": "camera_presence + lidar_range_fusion + target_slam_poses",
            "timestamp": self.results_start_time,
        }

        os.makedirs(self.output_dir, exist_ok=True)
        out_path = os.path.join(
            self.output_dir,
            f"tracking_stats_{model_name}_seed{self.seed}_{self.results_start_time}.json",
        )
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(json_results, f, indent=2)

        self.get_logger().info(f"Saved results JSON: {out_path}")
        self._log_aggregated_summary(aggregated)

    def _current_robot_state_tensor(self) -> Optional[torch.Tensor]:
        poses = []
        for i in range(self.num_robots):
            pose = self.odom_by_robot[i]
            if pose is None:
                return None
            poses.append([pose.x, pose.y, pose.yaw])
        return torch.tensor(poses, dtype=torch.float32)

    def _fuse_camera_lidar_observations(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        z_world = torch.zeros((self.num_robots, self.num_landmarks, 2), dtype=torch.float32)
        visible = torch.zeros((self.num_robots, self.num_landmarks), dtype=torch.bool)
        dbg_counts = {
            "pairs_total": 0,
            "pairs_fov": 0,
            "pairs_camera": 0,
            "pairs_lidar": 0,
            "pairs_lidar_global_match": 0,
            "robots_missing_streams": 0,
            "camera_cols_nonzero_total": 0,
            "scan_finite_total": 0,
        }

        for r in range(self.num_robots):
            camera_cols = self.camera_cols_by_robot[r]
            width = self.camera_width_by_robot[r]
            hfov = self.camera_hfov_by_robot[r]
            scan = self.scan_by_robot[r]
            pose = self.odom_by_robot[r]

            if camera_cols is None or width <= 1 or scan is None or pose is None:
                dbg_counts["robots_missing_streams"] += 1
                continue
            dbg_counts["camera_cols_nonzero_total"] += int(np.count_nonzero(camera_cols))
            dbg_counts["scan_finite_total"] += int(np.isfinite(scan.ranges).sum())

            rx, ry, ryaw = pose.x, pose.y, pose.yaw

            for t in range(self.num_landmarks):
                dbg_counts["pairs_total"] += 1
                tx = float(self.mu_real[t, 0])
                ty = float(self.mu_real[t, 1])

                dx = tx - rx
                dy = ty - ry
                bearing = self._angle_wrap(math.atan2(dy, dx) - ryaw)
                gt_distance = math.hypot(dx, dy)

                if abs(bearing) > hfov * 0.5:
                    continue
                dbg_counts["pairs_fov"] += 1

                col = int(round((bearing + hfov * 0.5) / hfov * (width - 1)))
                c0 = max(0, col - self.camera_half_window_px)
                c1 = min(width, col + self.camera_half_window_px + 1)
                if c0 >= c1:
                    continue
                if not bool(np.any(camera_cols[c0:c1])):
                    continue
                dbg_counts["pairs_camera"] += 1

                lidar_r_a = self._lidar_range_at_bearing(scan, bearing, expected_distance=gt_distance)
                lidar_r_b = self._lidar_range_at_bearing(scan, -bearing, expected_distance=gt_distance)
                lidar_candidates = [v for v in (lidar_r_a, lidar_r_b) if v is not None]
                if not lidar_candidates:
                    continue
                lidar_r = min(lidar_candidates, key=lambda rr: abs(rr - gt_distance))
                if abs(lidar_r - gt_distance) > self.lidar_range_gate:
                    global_best = self._lidar_best_global_distance(scan, expected_distance=gt_distance)
                    if global_best is not None and abs(global_best - gt_distance) <= self.lidar_range_gate:
                        dbg_counts["pairs_lidar_global_match"] += 1
                    continue
                dbg_counts["pairs_lidar"] += 1

                rel_x = lidar_r * math.cos(bearing)
                rel_y = lidar_r * math.sin(bearing)

                world_x = rx + rel_x * math.cos(ryaw) - rel_y * math.sin(ryaw)
                world_y = ry + rel_x * math.sin(ryaw) + rel_y * math.cos(ryaw)

                z_world[r, t, 0] = world_x
                z_world[r, t, 1] = world_y
                visible[r, t] = True

        if self.debug_sensor_fusion and self.active_episode:
            tracked_now = int(visible.any(dim=0).sum().item())
            self.get_logger().info(
                "fusion_debug step=%d pairs(total=%d fov=%d cam=%d lidar=%d) robots_missing=%d tracked_now=%d"
                % (
                    self.step_index,
                    dbg_counts["pairs_total"],
                    dbg_counts["pairs_fov"],
                    dbg_counts["pairs_camera"],
                    dbg_counts["pairs_lidar"],
                    dbg_counts["robots_missing_streams"],
                    tracked_now,
                )
            )

        return z_world, visible

    def _lidar_range_at_bearing(
        self, scan: ScanFrame, bearing: float, expected_distance: Optional[float] = None
    ) -> Optional[float]:
        if scan.angle_increment == 0.0 or scan.ranges.size == 0:
            return None

        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        i0 = max(0, idx - self.lidar_index_half_window)
        i1 = min(scan.ranges.size, idx + self.lidar_index_half_window + 1)
        if i0 >= i1:
            return None

        window = scan.ranges[i0:i1]
        finite = np.isfinite(window)
        if not np.any(finite):
            return None

        valid = window[finite]
        min_valid = max(scan.range_min, self.lidar_min_valid_range)
        valid = valid[(valid >= min_valid) & (valid <= scan.range_max)]
        if valid.size == 0:
            return None

        if expected_distance is not None:
            idx_best = int(np.argmin(np.abs(valid - expected_distance)))
            return float(valid[idx_best])

        return float(np.min(valid))

    def _lidar_best_global_distance(self, scan: ScanFrame, expected_distance: float) -> Optional[float]:
        if scan.ranges.size == 0:
            return None
        finite = np.isfinite(scan.ranges)
        if not np.any(finite):
            return None
        valid = scan.ranges[finite]
        min_valid = max(scan.range_min, self.lidar_min_valid_range)
        valid = valid[(valid >= min_valid) & (valid <= scan.range_max)]
        if valid.size == 0:
            return None
        idx_best = int(np.argmin(np.abs(valid - expected_distance)))
        return float(valid[idx_best])

    def _publish_actions(self, action: torch.Tensor):
        action = action.detach()
        if action.dim() == 1:
            action = action[None, :]

        for i in range(self.num_robots):
            if i >= action.size(0):
                continue
            msg = Twist()
            msg.linear.x = float(action[i, 0])
            msg.angular.z = float(action[i, 1])
            self.agent_cmd_pubs[i].publish(msg)

    def _publish_target_world_velocities(self):
        if not self.publish_target_velocities or self.v is None:
            return
        stamp = self.get_clock().now().to_msg()
        for t in range(self.num_landmarks):
            msg = Vector3Stamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.velocity_stamp_frame
            msg.vector.x = float(self.v[t, 0])
            msg.vector.y = float(self.v[t, 1])
            msg.vector.z = 0.0
            self.target_vel_pubs[t].publish(msg)

    def _publish_zero_velocities(self):
        zero = Twist()
        for pub in self.agent_cmd_pubs:
            pub.publish(zero)

    def _publish_zero_target_velocities(self):
        stamp = self.get_clock().now().to_msg()
        for t in range(self.max_num_landmarks):
            msg = Vector3Stamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.velocity_stamp_frame
            msg.vector.x = 0.0
            msg.vector.y = 0.0
            msg.vector.z = 0.0
            self.target_vel_pubs[t].publish(msg)

    def _publish_markers(self, x: torch.Tensor, visible: torch.Tensor):
        msg = MarkerArray()
        marker_id = 0

        for i in range(self.num_robots):
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.ns = "agents"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = 0.8
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = 0.1
            m.color.g = 0.3
            m.color.b = 0.95
            m.color.a = 1.0
            m.pose.position.x = float(x[i, 0])
            m.pose.position.y = float(x[i, 1])
            m.pose.position.z = 0.3
            yaw = float(x[i, 2])
            m.pose.orientation.z = math.sin(yaw * 0.5)
            m.pose.orientation.w = math.cos(yaw * 0.5)
            msg.markers.append(m)

        for t in range(self.num_landmarks):
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.ns = "targets_slam"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            seen = bool(visible[:, t].any().item())
            if seen:
                m.color.r = 0.1
                m.color.g = 0.95
                m.color.b = 0.1
            else:
                m.color.r = 0.95
                m.color.g = 0.4
                m.color.b = 0.0
            m.color.a = 1.0
            m.pose.position.x = float(self.mu_real[t, 0])
            m.pose.position.y = float(self.mu_real[t, 1])
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            msg.markers.append(m)

        mu_est = self.agent.mu_update
        for t in range(self.num_landmarks):
            m = Marker()
            m.header.frame_id = self.marker_frame
            m.ns = "targets_est"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.9
            m.pose.position.x = float(mu_est[t, 0])
            m.pose.position.y = float(mu_est[t, 1])
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            msg.markers.append(m)

        scan_stride = 3
        min_valid = self.lidar_min_valid_range
        scan_colors = ((0.05, 0.45, 1.0), (1.0, 0.25, 0.2), (0.1, 0.9, 0.35), (1.0, 0.85, 0.15))
        for i in range(self.num_robots):
            pose = self.odom_by_robot[i]
            scan = self.scan_by_robot[i]
            if pose is None or scan is None or scan.ranges.size == 0:
                continue

            ranges = scan.ranges[::scan_stride]
            angles = scan.angle_min + np.arange(0, scan.ranges.size, scan_stride, dtype=np.float32) * scan.angle_increment
            valid = np.isfinite(ranges)
            valid &= ranges >= max(scan.range_min, min_valid)
            valid &= ranges <= (scan.range_max - 0.05)
            if not np.any(valid):
                continue

            sm = Marker()
            sm.header.frame_id = self.marker_frame
            sm.ns = "agent_scans"
            sm.id = marker_id
            marker_id += 1
            sm.type = Marker.POINTS
            sm.action = Marker.ADD
            sm.scale.x = 0.035
            sm.scale.y = 0.035
            c = scan_colors[i % len(scan_colors)]
            sm.color.r = c[0]
            sm.color.g = c[1]
            sm.color.b = c[2]
            sm.color.a = 0.65

            rvals = ranges[valid]
            avals = angles[valid]
            cy = math.cos(pose.yaw)
            sy = math.sin(pose.yaw)
            for rr, aa in zip(rvals, avals):
                lx = float(rr * math.cos(float(aa)))
                ly = float(rr * math.sin(float(aa)))
                wx = pose.x + lx * cy - ly * sy
                wy = pose.y + lx * sy + ly * cy
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.05
                sm.points.append(p)

            msg.markers.append(sm)

        self.marker_pub.publish(msg)

    def _clear_markers(self):
        msg = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self.marker_frame
        delete.action = Marker.DELETEALL
        msg.markers.append(delete)
        self.marker_pub.publish(msg)

    @staticmethod
    def _encoding_channels(encoding: str) -> int:
        if encoding in ("rgb8", "bgr8"):
            return 3
        if encoding in ("rgba8", "bgra8"):
            return 4
        if encoding in ("mono8",):
            return 1
        return 0

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_wrap(v: float) -> float:
        return (v + math.pi) % (2.0 * math.pi) - math.pi

    def _log_trail_summary(self, summary: Dict):
        lines = []
        lines.append("=" * 70)
        lines.append(
            f"Trail {summary['episode']} | targets={summary['num_targets']} | robots={summary['num_robots']}"
        )
        lines.append(
            "target | first | " + " | ".join([f"r{r} %" for r in range(summary["num_robots"])]) + " | cum % | overlap %"
        )
        for target_data in summary["per_target"]:
            per_robot_pct = " | ".join([f"{d['percentage']:5.1f}" for d in target_data["per_robot"]])
            lines.append(
                f"{target_data['target_idx']:>6d} | {target_data['first_tracked_step']:>5d} | "
                f"{per_robot_pct} | {target_data['cumulative_percentage']:5.1f} | {target_data['overlap_percentage']:5.1f}"
            )
        totals = summary["totals"]
        robot_totals = " | ".join([f"{d['percentage']:5.1f}" for d in totals["per_robot"]])
        lines.append(
            f"totals |   --  | {robot_totals} | {totals['cumulative_percentage']:5.1f} | {totals['overlap_percentage']:5.1f}"
        )
        lines.append("=" * 70)
        self.get_logger().info("\n".join(lines))

    def _log_aggregated_summary(self, aggregated: Dict[int, Dict]):
        lines = []
        lines.append("=" * 70)
        lines.append("Averaged Statistics by Number of Targets")
        lines.append("=" * 70)
        for num_targets, stats in sorted(aggregated.items()):
            row = (
                f"targets={num_targets:2d} trails={stats['num_trails']:2d} "
                f"cum={stats['cumulative_avg_pct']:5.1f}+-{stats['cumulative_std_pct']:4.1f} "
                f"overlap={stats['overlap_avg_pct']:5.1f}+-{stats['overlap_std_pct']:4.1f}"
            )
            lines.append(row)
        legacy_mean = float(np.mean(self.overall_avg_targets)) if self.overall_avg_targets else 0.0
        legacy_std = float(np.std(self.overall_avg_targets)) if self.overall_avg_targets else 0.0
        lines.append(f"legacy mean_targets_tracked={legacy_mean:.3f} std={legacy_std:.3f}")
        lines.append("=" * 70)
        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = MBAMCentralRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
