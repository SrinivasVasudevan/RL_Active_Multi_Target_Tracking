"""The MBAM policy / estimator / statistics loop, with no Isaac dependencies.

This is deliberately the only place episode logic lives. `mbam_env.py` just asks
the coordinator what the robots and targets should do next, applies that to the
simulator, and reports back the poses the simulator actually achieved.

Because nothing here imports Isaac, `tests/test_core_offline.py` can drive the
whole loop on a plain workstation and check it against the thesis numbers.
"""

from __future__ import annotations

import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
from torch import tensor

from .config import MBAMConfig
from .episode_sampler import EpisodeSampler, EpisodeState


def _import_mbam(repo: str):
    """Import the thesis agent/policy/utilities straight from the repo."""
    if repo not in sys.path:
        sys.path.insert(0, repo)
    from agents.model_based_agent import ModelBasedAgentAtt  # noqa: E402
    from utilities.utils import SE2_kinematics, landmark_motion_real, triangle_SDF  # noqa: E402
    return ModelBasedAgentAtt, SE2_kinematics, landmark_motion_real, triangle_SDF


def compute_fov_mask(mu_real: tensor, x: tensor, psi: tensor, radius: float, triangle_sdf) -> tensor:
    """Boolean [num_robots x num_landmarks] visibility mask.

    Identical to compute_fov_mask() in scripts/run_model_based_testing.py.
    """
    if len(x.size()) == 1:
        x = x[None, :]
    masks = []
    for r in range(x.size(0)):
        q = torch.vstack((
            (mu_real[:, 0] - x[r, 0]) * torch.cos(x[r, 2]) + (mu_real[:, 1] - x[r, 1]) * torch.sin(x[r, 2]),
            (x[r, 0] - mu_real[:, 0]) * torch.sin(x[r, 2]) + (mu_real[:, 1] - x[r, 1]) * torch.cos(x[r, 2])
        )).T
        masks.append(triangle_sdf(q, psi, radius) <= 0)
    return torch.stack(masks)


class MBAMCoordinator:
    """Drives one MBAM evaluation: sampling, planning, estimation, statistics."""

    def __init__(self, cfg: MBAMConfig):
        self.cfg = cfg
        (self._AgentCls, self._se2, self._landmark_motion_real, self._triangle_sdf) = _import_mbam(cfg.mbam_repo)

        # TrackingStatistics is reused verbatim from the ROS package so the
        # reported columns match the synthetic and Gazebo runs exactly.
        ros_core = os.path.join(cfg.mbam_repo, "ros2_ws", "src", "mbam_gazebo_tracking")
        if ros_core not in sys.path:
            sys.path.insert(0, ros_core)
        from mbam_gazebo_tracking.core.tracking_stats import (  # noqa: E402
            TrackingStatistics, aggregate_by_target_count,
        )
        self._StatsCls = TrackingStatistics
        self._aggregate = aggregate_by_target_count

        self._psi = tensor([cfg.psi])
        self.sampler = EpisodeSampler(
            max_num_landmarks=cfg.max_num_landmarks, num_robots=cfg.num_robots,
            landmark_motion_scale=cfg.landmark_motion_scale,
            num_clusters=cfg.num_clusters, clustering_prob=cfg.clustering_prob,
        )

        if not cfg.model_path or not os.path.exists(cfg.model_path):
            raise FileNotFoundError(f"MBAM checkpoint not found: {cfg.model_path!r}")

        self._check_robot_count(cfg)

        self.agent = self._AgentCls(
            max_num_landmarks=cfg.max_num_landmarks, init_info=cfg.init_info,
            A=cfg.a, B=cfg.b, W=cfg.w, radius=cfg.radius, psi=self._psi,
            kappa=cfg.kappa, V=cfg.v_noise, lr=cfg.lr, num_robots=cfg.num_robots,
        )
        self.agent.load_policy_state_dict(cfg.model_path)
        self.agent.eval_policy()

        self.episode_index = 0
        self.step_index = 0
        self.horizon = 0
        self.num_landmarks = 0
        self.mu_real: Optional[tensor] = None
        self.v: Optional[tensor] = None
        self._bias: Optional[tensor] = None
        self._stats = None
        self._per_step_tracked: List[int] = []

        self.all_summaries: List[Dict] = []
        self.all_avg_targets: List[float] = []


    @staticmethod
    def _check_robot_count(cfg: MBAMConfig) -> None:
        """Fail early and clearly if the checkpoint was trained for a different team size.

        The policy encodes each robot's own pose plus every teammate's relative
        pose, so the first layer is sized 3 + 3*(num_robots-1). A checkpoint is
        therefore tied to the robot count it was trained with; only the number of
        *targets* is free to vary (the landmark encoder is shared per target).
        """
        state = torch.load(cfg.model_path, map_location="cpu")
        w = state.get("agent_pos_fc1_pi.weight")
        if w is None:
            return
        ckpt_robots = (int(w.shape[1]) - 3) // 3 + 1
        if ckpt_robots != cfg.num_robots:
            raise ValueError(
                f"checkpoint {os.path.basename(cfg.model_path)} was trained for "
                f"{ckpt_robots} robots but num_robots={cfg.num_robots} was requested. "
                f"Target count may vary freely; robot count may not - retrain, or run with "
                f"--num-robots {ckpt_robots}."
            )

    # -- episode lifecycle -------------------------------------------------

    def start_episode(self) -> EpisodeState:
        """Sample a fresh episode. Returns initial robot poses and target positions."""
        s = self.sampler.sample()
        self.num_landmarks = s.num_landmarks
        self.horizon = s.horizon
        self.mu_real = s.mu_real.clone()
        self.v = s.v.clone()
        self._bias = s.landmark_motion_bias.clone()

        self.agent.reset_estimate_mu(self.mu_real)
        self.agent.reset_agent_info()

        self._stats = self._StatsCls(num_robots=self.cfg.num_robots, num_landmarks=self.num_landmarks)
        self._per_step_tracked = []
        self.step_index = 0
        self.episode_index += 1
        return s

    def policy_step(self, x_current: tensor) -> Tuple[tensor, tensor]:
        """One policy decision.

        Args:
            x_current: [num_robots x 3] SE(2) poses the simulator is actually at.

        Returns:
            (desired_x, mu_next) - where the robots should be at the end of this
            control interval, and where the targets have moved to.
        """
        action = self.agent.plan(self.v, x_current)

        desired_x = torch.stack([
            self._se2(x_current[i], action[i], self.cfg.tau) for i in range(x_current.size(0))
        ])

        self.mu_real = self._landmark_motion_real(self.mu_real, self.v, self.cfg.a, self.cfg.b, self.cfg.w)
        self.v = self.sampler.rollout_landmark_velocity(self.num_landmarks, self._bias)
        return desired_x, self.mu_real

    def observe(self, x_actual: tensor) -> tensor:
        """Fold the achieved poses into the estimator and the statistics.

        Returns the visibility mask so the caller can colour the scene.
        """
        self.agent.update_info_mu(self.mu_real, x_actual)
        fov = compute_fov_mask(self.mu_real, x_actual, self._psi, self.cfg.radius, self._triangle_sdf)
        self._stats.update(fov)
        self._per_step_tracked.append(int(fov.any(dim=0).sum().item()))
        self.step_index += 1
        return fov

    @property
    def episode_done(self) -> bool:
        return self.step_index >= self.horizon

    def finish_episode(self) -> Dict:
        objective = self.agent.update_policy_grad(False) / max(self.num_landmarks, 1)
        summary = self._stats.get_trail_summary()
        summary["episode"] = self.episode_index
        summary["objective_per_target"] = float(objective)
        self.all_summaries.append(summary)
        avg = float(np.mean(self._per_step_tracked)) if self._per_step_tracked else 0.0
        self.all_avg_targets.append(avg)
        return summary

    # -- reporting ---------------------------------------------------------

    def results(self) -> Dict:
        cum = np.array([s["totals"]["cumulative_percentage"] for s in self.all_summaries])
        ovl = np.array([s["totals"]["overlap_percentage"] for s in self.all_summaries])
        excl = cum - ovl
        return {
            "model_path": self.cfg.model_path,
            "seed": self.cfg.seed,
            "num_robots": self.cfg.num_robots,
            "control_mode": self.cfg.control_mode,
            "num_test_trials": len(self.all_summaries),
            "trails": self.all_summaries,
            "aggregated_by_targets": {str(k): v for k, v in self._aggregate(self.all_summaries).items()},
            "thesis_metrics": {
                "cumulative_tracking_pct": [round(float(cum.mean()), 4), round(float(cum.std()), 4)],
                "overlap_pct": [round(float(ovl.mean()), 4), round(float(ovl.std()), 4)],
                "exclusive_tracking_pct": [round(float(excl.mean()), 4), round(float(excl.std()), 4)],
                "mean_unique_targets": [round(float(np.mean(self.all_avg_targets)), 4),
                                        round(float(np.std(self.all_avg_targets)), 4)],
            },
        }
