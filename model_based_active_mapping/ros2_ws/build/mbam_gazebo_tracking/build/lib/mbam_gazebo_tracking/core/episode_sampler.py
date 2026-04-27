from dataclasses import dataclass
from typing import Dict

import torch
from torch import tensor


@dataclass
class EpisodeState:
    num_landmarks: int
    env_size: tensor
    horizon: int
    mu_real: tensor
    v: tensor
    landmark_motion_bias: tensor
    x: tensor


class EpisodeSampler:
    def __init__(self, max_num_landmarks: int, num_robots: int, landmark_motion_scale: float, num_clusters: int = 2):
        self.max_num_landmarks = max_num_landmarks
        self.num_robots = num_robots
        self.landmark_motion_scale = landmark_motion_scale
        self.num_clusters = max(1, num_clusters)

    def sample(self) -> EpisodeState:
        num_landmarks = torch.randint(3, self.max_num_landmarks + 1, (1,)).item()
        env_size = tensor([num_landmarks * 4.0, num_landmarks * 4.0], dtype=torch.float32)
        horizon = int(num_landmarks * 5)

        x = torch.empty((self.num_robots, 3), dtype=torch.float32)
        for i in range(self.num_robots):
            x[i, :2] = (torch.rand(2) - 0.5) * env_size * 0.6
            x[i, 2] = (torch.rand(1) * 2 - 1) * torch.pi

        num_clusters = min(self.num_clusters, num_landmarks)
        min_robot_dist = 0.3 * torch.min(env_size)
        mu = self._generate_clusters(num_landmarks, num_clusters, env_size, x, min_robot_dist)

        landmark_motion_bias = (torch.rand(2) - 0.5) * 1.6
        v = (torch.rand((num_landmarks, 2)) - 0.5) * self.landmark_motion_scale + landmark_motion_bias

        return EpisodeState(
            num_landmarks=num_landmarks,
            env_size=env_size,
            horizon=horizon,
            mu_real=mu,
            v=v,
            landmark_motion_bias=landmark_motion_bias,
            x=x,
        )

    def rollout_landmark_velocity(self, num_landmarks: int, landmark_motion_bias: tensor) -> tensor:
        return (torch.rand((num_landmarks, 2)) - 0.5) * self.landmark_motion_scale + landmark_motion_bias

    @staticmethod
    def _generate_clusters(num_landmarks: int, num_clusters: int, env_size: tensor, robot_poses: tensor, min_robot_dist: tensor) -> tensor:
        centers = torch.zeros((num_clusters, 2), dtype=torch.float32)
        env_half = env_size * 0.5
        robot_xy = robot_poses[:, :2]
        min_center_sep = min_robot_dist * 0.5

        for i in range(num_clusters):
            best_center = None
            best_min_dist = -1.0
            for _ in range(50):
                candidate = (torch.rand(2) - 0.5) * env_size
                d_to_robots = torch.linalg.norm(robot_xy - candidate, dim=1)
                min_dist = d_to_robots.min().item()

                if i > 0:
                    d_to_centers = torch.linalg.norm(centers[:i] - candidate, dim=1)
                    if d_to_centers.min().item() < min_center_sep:
                        continue

                if min_dist >= float(min_robot_dist):
                    best_center = candidate
                    break

                if min_dist > best_min_dist:
                    best_min_dist = min_dist
                    best_center = candidate

            centers[i] = best_center if best_center is not None else torch.clamp(candidate, -env_half, env_half)

        mu = torch.zeros((num_landmarks, 2), dtype=torch.float32)
        points_per_cluster = num_landmarks // num_clusters
        remainder = num_landmarks % num_clusters

        start_idx = 0
        for i in range(num_clusters):
            count = points_per_cluster + (1 if i < remainder else 0)
            cluster_points = centers[i] + torch.randn((count, 2)) * (env_size[0] / 15.0)
            mu[start_idx : start_idx + count] = cluster_points
            start_idx += count

        return torch.clamp(mu, -env_half, env_half)
