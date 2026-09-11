"""Episode sampling, bit-identical to MRMT/multi_robot_env.py's reset().

The RNG call order is preserved exactly (number of landmarks, robot poses,
cluster/uniform branch, motion bias, velocities). Given the same torch seed this
draws the same 30 episodes as the synthetic evaluation, which is what makes the
Isaac runs directly comparable to the thesis tables.
"""

from __future__ import annotations

from dataclasses import dataclass

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
    """Mirrors MultiRobotEnv.reset() / .step() landmark sampling."""

    def __init__(self, max_num_landmarks: int, num_robots: int, landmark_motion_scale: float,
                 num_clusters: int = 2, clustering_prob: float = 0.65):
        self._max_num_landmarks = max_num_landmarks
        self._num_robots = num_robots
        self._landmark_motion_scale = landmark_motion_scale
        self._num_clusters = num_clusters
        self._clustering_prob = clustering_prob

    def sample(self) -> EpisodeState:
        num_landmarks = torch.randint(4, min(10, self._max_num_landmarks + 1), (1,)).item()
        num_landmarks = max(num_landmarks, self._num_clusters)
        num_landmarks = min(num_landmarks, self._max_num_landmarks)

        env_size = tensor([num_landmarks * 3.2, num_landmarks * 3.2])
        horizon = num_landmarks * 5

        x = torch.empty(self._num_robots, 3)
        for i in range(self._num_robots):
            x[i, :2] = (torch.rand(2) - 0.5) * env_size * 0.5
            x[i, 2] = (torch.rand(1) * 2 - 1) * torch.pi

        if torch.rand(1).item() < self._clustering_prob:
            min_robot_dist = 0.3 * torch.min(env_size)
            mu = self._generate_clusters(num_landmarks, self._num_clusters, env_size, x, min_robot_dist)
        else:
            mu = (torch.rand((num_landmarks, 2)) - 0.5) * env_size

        landmark_motion_bias = (torch.rand(2) - 0.5) * 1.6
        v = (torch.rand((num_landmarks, 2)) - 0.5) * self._landmark_motion_scale + landmark_motion_bias

        return EpisodeState(num_landmarks=num_landmarks, env_size=env_size, horizon=horizon,
                            mu_real=mu, v=v, landmark_motion_bias=landmark_motion_bias, x=x)

    def rollout_landmark_velocity(self, num_landmarks: int, landmark_motion_bias: tensor) -> tensor:
        return (torch.rand((num_landmarks, 2)) - 0.5) * self._landmark_motion_scale + landmark_motion_bias

    @staticmethod
    def _generate_clusters(num_landmarks, num_clusters, env_size, robot_poses, min_robot_dist):
        centers = torch.zeros((num_clusters, 2))
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

                if min_dist >= min_robot_dist:
                    best_center = candidate
                    break

                if min_dist > best_min_dist:
                    best_min_dist = min_dist
                    best_center = candidate

            centers[i] = best_center if best_center is not None else torch.clamp(candidate, -env_half, env_half)

        mu = torch.zeros((num_landmarks, 2))
        points_per_cluster = num_landmarks // num_clusters
        remainder = num_landmarks % num_clusters

        start_idx = 0
        for i in range(num_clusters):
            count = points_per_cluster + (1 if i < remainder else 0)
            cluster_points = centers[i] + torch.randn((count, 2)) * (env_size[0] / 15.0)
            mu[start_idx:start_idx + count] = cluster_points
            start_idx += count

        return mu
