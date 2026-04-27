import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mbam_gazebo_tracking.core.model_based_agent_att_ros import ModelBasedAgentAttRos
from mbam_gazebo_tracking.core.real_world_types import RobotReport
from mbam_gazebo_tracking.core.track_manager import ReportSnapshot, TrackManager


class LimoCentralCoordinator(Node):
    def __init__(self):
        super().__init__("mbam_limo_central_coordinator")

        pkg_share = get_package_share_directory("mbam_gazebo_tracking")
        default_params = os.path.join(pkg_share, "config", "params_compare.yaml")
        default_ckpt = os.path.join(pkg_share, "checkpoints", "best_model_seed42.pth")

        self.declare_parameter("params_file", default_params)
        self.declare_parameter("model_path", default_ckpt)
        self.declare_parameter("robot_names", "limo0,limo1")
        self.declare_parameter("report_topic", "/mbam/robot_reports")
        self.declare_parameter("cmd_topic_template", "/{robot_name}/mbam_cmd_vel")
        self.declare_parameter("marker_topic", "/mbam/real_world_markers")
        self.declare_parameter("marker_frame", "map")
        self.declare_parameter("max_num_landmarks", 7)
        self.declare_parameter("control_period_sec", -1.0)
        self.declare_parameter("max_report_age_sec", 1.0)
        self.declare_parameter("track_association_distance_m", 1.5)
        self.declare_parameter("track_timeout_sec", 2.0)
        self.declare_parameter("search_linear_velocity", 0.0)
        self.declare_parameter("search_angular_velocity", 0.4)
        self.declare_parameter("enable_collision_pause", True)
        self.declare_parameter("collision_lookahead_sec", 1.0)
        self.declare_parameter("collision_robot_radius", 0.28)
        self.declare_parameter("collision_target_radius", 0.35)
        self.declare_parameter("collision_safety_margin", 0.10)

        self.params_file = str(self.get_parameter("params_file").value)
        self.model_path = str(self.get_parameter("model_path").value)
        if not os.path.exists(self.params_file):
            raise FileNotFoundError(f"params file not found: {self.params_file}")
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"model checkpoint not found: {self.model_path}")

        robot_names_csv = str(self.get_parameter("robot_names").value)
        self.robot_names = [item.strip() for item in robot_names_csv.split(",") if item.strip()]
        if not self.robot_names:
            raise ValueError("robot_names must contain at least one robot name")
        self.num_robots = len(self.robot_names)

        with open(self.params_file, "r", encoding="utf-8") as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)

        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.report_topic = str(self.get_parameter("report_topic").value)
        self.cmd_topic_template = str(self.get_parameter("cmd_topic_template").value)
        self.max_num_landmarks = int(self.get_parameter("max_num_landmarks").value)
        self.max_report_age_sec = float(self.get_parameter("max_report_age_sec").value)
        self.enable_collision_pause = bool(self.get_parameter("enable_collision_pause").value)
        self.collision_lookahead_sec = float(self.get_parameter("collision_lookahead_sec").value)
        self.collision_robot_radius = float(self.get_parameter("collision_robot_radius").value)
        self.collision_target_radius = float(self.get_parameter("collision_target_radius").value)
        self.collision_safety_margin = float(self.get_parameter("collision_safety_margin").value)
        self.search_linear_velocity = float(self.get_parameter("search_linear_velocity").value)
        self.search_angular_velocity = float(self.get_parameter("search_angular_velocity").value)

        self._init_core_components()

        control_period_sec = float(self.get_parameter("control_period_sec").value)
        if control_period_sec <= 0.0:
            control_period_sec = self.tau
        self.control_period_sec = control_period_sec

        self.track_manager = TrackManager(
            max_tracks=self.max_num_landmarks,
            association_distance_m=float(self.get_parameter("track_association_distance_m").value),
            track_timeout_sec=float(self.get_parameter("track_timeout_sec").value),
        )

        self.latest_reports: Dict[str, ReportSnapshot] = {}
        self.warn_times: Dict[str, float] = {}
        self.active_track_ids: List[int] = []

        self.cmd_pubs = {
            robot_name: self.create_publisher(Twist, self.cmd_topic_template.format(robot_name=robot_name), 10)
            for robot_name in self.robot_names
        }
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(String, self.report_topic, self._report_cb, 50)
        self.timer = self.create_timer(self.control_period_sec, self._control_loop)

        self.get_logger().info(
            f"Central MBAM coordinator ready for robots={self.robot_names} report_topic='{self.report_topic}' "
            f"cmd_topics={[self.cmd_topic_template.format(robot_name=name) for name in self.robot_names]}"
        )

    def _init_core_components(self):
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

        init_info = float(self.params["init_info"])
        radius = float(self.params["FoV"]["radius"])
        psi = torch.tensor([float(self.params["FoV"]["psi"])], dtype=torch.float32)
        kappa = float(self.params["FoV"]["kappa"])

        v_noise = torch.zeros(2, dtype=torch.float32)
        v_noise[0] = self.params["FoV"]["V"]["_1"]
        v_noise[1] = self.params["FoV"]["V"]["_2"]

        lr = float(self.params["lr"])

        self.tau = tau
        self.a = a
        self.b = b
        self.w = w

        self.agent = ModelBasedAgentAttRos(
            max_num_landmarks=self.max_num_landmarks,
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

    def _report_cb(self, msg: String):
        try:
            report = RobotReport.from_json(msg.data)
        except Exception as exc:
            self._warn_throttle("bad_report", f"Failed to parse robot report: {exc}")
            return

        if report.robot_name not in self.cmd_pubs:
            self._warn_throttle(
                f"unknown_robot_{report.robot_name}",
                f"Ignoring report from unknown robot '{report.robot_name}'.",
            )
            return

        received_sec = self.get_clock().now().nanoseconds * 1e-9
        self.latest_reports[report.robot_name] = ReportSnapshot(report=report, received_sec=received_sec)

    def _control_loop(self):
        snapshots = self._get_fresh_snapshots()
        if snapshots is None:
            self._publish_zero_velocities()
            return

        if not self._reports_share_frame(snapshots):
            self._publish_zero_velocities()
            return

        x = self._robot_state_tensor(snapshots)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.track_manager.update(snapshots, now_sec)
        track_ids, labels, mu_tracks, v_tracks, z_world, visible = self.track_manager.export_tensors(self.robot_names)

        if mu_tracks.size(0) == 0:
            self.active_track_ids = []
            search_action = self._search_action_tensor()
            safe_action = self._apply_collision_pause(search_action, x, mu_tracks)
            self._publish_actions(safe_action)
            self._publish_markers(x, labels, mu_tracks, visible)
            return

        if track_ids != self.active_track_ids:
            self.agent.set_estimate_mu(mu_tracks, add_noise=False)
            self.agent.reset_agent_info()
            self.active_track_ids = list(track_ids)

        action = self.agent.plan(v_tracks, x)
        safe_action = self._apply_collision_pause(action, x, mu_tracks)
        self._publish_actions(safe_action)
        self.agent.update_info_from_observations(z_world, visible, x)

        self._publish_markers(x, labels, self.agent.mu_update, visible)

    def _get_fresh_snapshots(self) -> Optional[List[ReportSnapshot]]:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        snapshots: List[ReportSnapshot] = []
        missing = []
        stale = []
        for robot_name in self.robot_names:
            snapshot = self.latest_reports.get(robot_name)
            if snapshot is None:
                missing.append(robot_name)
                continue
            if now_sec - snapshot.received_sec > self.max_report_age_sec:
                stale.append(robot_name)
                continue
            snapshots.append(snapshot)

        if missing:
            self._warn_throttle("missing_reports", f"Waiting for robot reports from: {missing}")
            return None
        if stale:
            self._warn_throttle("stale_reports", f"Robot reports are stale for: {stale}")
            return None
        return snapshots

    def _reports_share_frame(self, snapshots: Sequence[ReportSnapshot]) -> bool:
        frames = {snap.report.world_frame for snap in snapshots if snap.report.world_frame}
        if len(frames) <= 1:
            return True
        self._warn_throttle(
            "frame_mismatch",
            f"Robot reports use different world frames: {sorted(frames)}. Central tracking requires a shared frame.",
        )
        return False

    def _robot_state_tensor(self, snapshots: Sequence[ReportSnapshot]) -> torch.Tensor:
        poses = []
        report_by_name = {snap.report.robot_name: snap.report for snap in snapshots}
        for robot_name in self.robot_names:
            report = report_by_name[robot_name]
            poses.append([report.pose_x, report.pose_y, report.pose_yaw])
        return torch.tensor(poses, dtype=torch.float32)

    def _search_action_tensor(self) -> torch.Tensor:
        action = torch.zeros((self.num_robots, 2), dtype=torch.float32)
        for idx in range(self.num_robots):
            action[idx, 0] = self.search_linear_velocity
            action[idx, 1] = self.search_angular_velocity if idx % 2 == 0 else -self.search_angular_velocity
        return action

    def _publish_actions(self, action: torch.Tensor):
        action = action.detach()
        if action.dim() == 1:
            action = action[None, :]

        for idx, robot_name in enumerate(self.robot_names):
            if idx >= action.size(0):
                continue
            msg = Twist()
            msg.linear.x = float(action[idx, 0])
            msg.angular.z = float(action[idx, 1])
            self.cmd_pubs[robot_name].publish(msg)

    def _publish_zero_velocities(self):
        zero = Twist()
        for pub in self.cmd_pubs.values():
            pub.publish(zero)

    def _apply_collision_pause(self, action: torch.Tensor, x: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        if not self.enable_collision_pause:
            return action

        if action.dim() == 1:
            action = action[None, :]
        safe_action = action.detach().clone()

        lookahead = max(self.tau, self.collision_lookahead_sec)
        robot_clearance = max(0.0, 2.0 * self.collision_robot_radius + self.collision_safety_margin)
        target_clearance = max(
            0.0,
            self.collision_robot_radius + self.collision_target_radius + self.collision_safety_margin,
        )

        current_xy: List[Tuple[float, float]] = []
        predicted_xy: List[Tuple[float, float]] = []
        velocity_xy: List[Tuple[float, float]] = []
        for i in range(self.num_robots):
            px = float(x[i, 0])
            py = float(x[i, 1])
            yaw = float(x[i, 2])
            vlin = float(safe_action[i, 0]) if i < safe_action.size(0) else 0.0
            vx = vlin * math.cos(yaw)
            vy = vlin * math.sin(yaw)
            nx = px + vlin * lookahead * math.cos(yaw)
            ny = py + vlin * lookahead * math.sin(yaw)
            current_xy.append((px, py))
            predicted_xy.append((nx, ny))
            velocity_xy.append((vx, vy))

        should_pause = [False] * self.num_robots

        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                d_min = self._min_pair_distance_over_horizon(
                    p1=current_xy[i],
                    v1=velocity_xy[i],
                    p2=current_xy[j],
                    v2=velocity_xy[j],
                    horizon=lookahead,
                )
                if d_min <= robot_clearance:
                    should_pause[i] = True
                    should_pause[j] = True

        for i in range(self.num_robots):
            if should_pause[i]:
                continue
            start = current_xy[i]
            end = predicted_xy[i]
            for t in range(targets.size(0)):
                tx = float(targets[t, 0])
                ty = float(targets[t, 1])
                d_now = math.hypot(start[0] - tx, start[1] - ty)
                d_pred = math.hypot(end[0] - tx, end[1] - ty)
                d_seg = self._point_segment_distance(tx, ty, start[0], start[1], end[0], end[1])
                if min(d_now, d_pred, d_seg) <= target_clearance:
                    should_pause[i] = True
                    break

        for i in range(self.num_robots):
            if should_pause[i] and i < safe_action.size(0):
                safe_action[i, 0] = 0.0
                safe_action[i, 1] = 0.0

        return safe_action

    def _publish_markers(
        self,
        x: torch.Tensor,
        labels: Sequence[str],
        mu_est: torch.Tensor,
        visible: torch.Tensor,
    ):
        msg = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self.marker_frame
        delete.action = Marker.DELETEALL
        msg.markers.append(delete)
        marker_id = 0

        for i, robot_name in enumerate(self.robot_names):
            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.ns = "robots"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.65
            marker.scale.y = 0.16
            marker.scale.z = 0.16
            marker.color.r = 0.1
            marker.color.g = 0.3
            marker.color.b = 0.95
            marker.color.a = 1.0
            marker.pose.position.x = float(x[i, 0])
            marker.pose.position.y = float(x[i, 1])
            marker.pose.position.z = 0.15
            yaw = float(x[i, 2])
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            msg.markers.append(marker)

            text = Marker()
            text.header.frame_id = self.marker_frame
            text.ns = "robot_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.20
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.pose.position.x = float(x[i, 0])
            text.pose.position.y = float(x[i, 1])
            text.pose.position.z = 0.45
            text.pose.orientation.w = 1.0
            text.text = robot_name
            msg.markers.append(text)

        for t in range(mu_est.size(0)):
            seen = bool(visible[:, t].any().item()) if visible.size(1) > t else False
            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.ns = "target_tracks"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.28
            marker.scale.y = 0.28
            marker.scale.z = 0.28
            if seen:
                marker.color.r = 0.0
                marker.color.g = 0.95
                marker.color.b = 0.25
            else:
                marker.color.r = 0.95
                marker.color.g = 0.55
                marker.color.b = 0.0
            marker.color.a = 0.9
            marker.pose.position.x = float(mu_est[t, 0])
            marker.pose.position.y = float(mu_est[t, 1])
            marker.pose.position.z = 0.20
            marker.pose.orientation.w = 1.0
            msg.markers.append(marker)

            text = Marker()
            text.header.frame_id = self.marker_frame
            text.ns = "target_track_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.18
            text.color.r = 0.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.pose.position.x = float(mu_est[t, 0])
            text.pose.position.y = float(mu_est[t, 1])
            text.pose.position.z = 0.45
            text.pose.orientation.w = 1.0
            label = labels[t] if t < len(labels) else "target"
            text.text = f"{label}:{t}"
            msg.markers.append(text)

        self.marker_pub.publish(msg)

    def _warn_throttle(self, key: str, message: str, period_sec: float = 2.0):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        last_time = self.warn_times.get(key, 0.0)
        if now_sec - last_time >= period_sec:
            self.get_logger().warn(message)
            self.warn_times[key] = now_sec

    @staticmethod
    def _point_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x2 - x1
        dy = y2 - y1
        denom = dx * dx + dy * dy
        if denom <= 1e-9:
            return math.hypot(px - x1, py - y1)
        u = ((px - x1) * dx + (py - y1) * dy) / denom
        u = max(0.0, min(1.0, u))
        cx = x1 + u * dx
        cy = y1 + u * dy
        return math.hypot(px - cx, py - cy)

    @staticmethod
    def _min_pair_distance_over_horizon(
        p1: Tuple[float, float],
        v1: Tuple[float, float],
        p2: Tuple[float, float],
        v2: Tuple[float, float],
        horizon: float,
    ) -> float:
        px = p1[0] - p2[0]
        py = p1[1] - p2[1]
        vx = v1[0] - v2[0]
        vy = v1[1] - v2[1]
        vv = vx * vx + vy * vy
        if vv <= 1e-9 or horizon <= 1e-9:
            return math.hypot(px, py)
        t_star = -(px * vx + py * vy) / vv
        t_star = max(0.0, min(horizon, t_star))
        dx = px + vx * t_star
        dy = py + vy * t_star
        return math.hypot(dx, dy)


def main(args=None):
    rclpy.init(args=args)
    node = LimoCentralCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
