"""Scenario configuration for the MBAM-on-Isaac experiments."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional

import torch
import yaml
from torch import tensor

# Root of the thesis repository. The MBAM policy/agent code is imported from here
# rather than vendored, so the Isaac runs use bit-identical code to the synthetic
# evaluation in scripts/run_model_based_testing.py.
DEFAULT_MBAM_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


@dataclass
class MBAMConfig:
    """Everything needed to reproduce the thesis scenario inside Isaac Sim."""

    # --- fleet -------------------------------------------------------------
    num_robots: int = 2
    # Targets are sampled per episode in [4, max_num_landmarks], matching the thesis.
    max_num_landmarks: int = 7
    num_clusters: int = 2
    clustering_prob: float = 0.65

    # --- evaluation --------------------------------------------------------
    num_test_trials: int = 30
    seed: int = 42

    # --- policy ------------------------------------------------------------
    model_path: str = ""
    mbam_repo: str = DEFAULT_MBAM_REPO
    params_file: str = ""

    # --- filled in from params_compare.yaml --------------------------------
    tau: float = 1.0
    init_info: float = 0.5
    radius: float = 6.0
    psi: float = 0.785
    kappa: float = 0.4
    lr: float = 3e-4
    landmark_motion_scale: float = 1.0
    a: tensor = field(default_factory=lambda: torch.eye(2))
    b: tensor = field(default_factory=lambda: torch.eye(2))
    w: tensor = field(default_factory=lambda: torch.full((2,), 0.0025))
    v_noise: tensor = field(default_factory=lambda: torch.full((2,), 0.04))

    # --- Isaac-specific ----------------------------------------------------
    # Drones fly a fixed-altitude SE(2) slice; the policy is unchanged.
    flight_altitude: float = 2.5
    target_altitude: float = 0.45   # sphere radius, so markers sit on the ground
    # "kinematic": teleport drones to the policy pose each decision step. Highest
    #   fidelity to the synthetic env; use this for numbers comparable to the thesis.
    # "offboard": stream PX4 position setpoints and let the flight stack track them.
    #   Realistic, but tracking lag changes the achieved poses.
    control_mode: str = "kinematic"
    # Simulation steps per policy decision. IsaacEnv runs at 1/250 s, tau = 1 s.
    sim_dt: float = 1.0 / 250.0
    # In kinematic mode the robots are teleported, so there is no dynamics to
    # integrate - we only need enough ticks to render the new state. Stepping the
    # full decimation there would cost 250x for an identical result.
    kinematic_ticks: int = 2
    layout: str = "grid"
    target_asset: str = "sphere"  # "sphere" | "crab"
    robot_scale: float = 6.0       # visual scale of the drone mesh in the footage
    show_fov: bool = True          # draw each robot's sensor wedge

    # --- recording ---------------------------------------------------------
    record: bool = True            # capture the viewport to per-episode videos
    record_fps: int = 5            # 1 policy step = 1 frame, so 5 fps ~ 5x real time
    record_episodes: int = 0       # 0 = record every episode, N = only the first N
    keep_frames: bool = False      # keep the raw PNGs alongside the video
    camera_margin: float = 1.4     # how much slack around the tracked entities

    @property
    def decimation(self) -> int:
        """Number of sim ticks per policy step (tau / sim_dt)."""
        return max(1, int(round(self.tau / self.sim_dt)))

    @classmethod
    def from_params_yaml(cls, params_file: str = "", **overrides) -> "MBAMConfig":
        """Build a config from the thesis params_compare.yaml, then apply overrides."""
        repo = overrides.get("mbam_repo", DEFAULT_MBAM_REPO)
        if not params_file:
            params_file = os.path.join(repo, "params", "params_compare.yaml")
        with open(params_file, "r", encoding="utf-8") as f:
            p = yaml.load(f, Loader=yaml.FullLoader)

        a = torch.zeros((2, 2)); a[0, 0] = p["motion"]["A"]["_1"]; a[1, 1] = p["motion"]["A"]["_2"]
        b = torch.zeros((2, 2)); b[0, 0] = p["motion"]["B"]["_1"]; b[1, 1] = p["motion"]["B"]["_2"]
        w = torch.zeros(2);      w[0] = p["motion"]["W"]["_1"];    w[1] = p["motion"]["W"]["_2"]
        v = torch.zeros(2);      v[0] = p["FoV"]["V"]["_1"];       v[1] = p["FoV"]["V"]["_2"]

        cfg = cls(
            max_num_landmarks=int(p["max_num_landmarks"]),
            num_test_trials=int(p["num_test_trials"]),
            tau=float(p["tau"]),
            init_info=float(p["init_info"]),
            radius=float(p["FoV"]["radius"]),
            psi=float(p["FoV"]["psi"]),
            kappa=float(p["FoV"]["kappa"]),
            lr=float(p["lr"]),
            landmark_motion_scale=float(p["motion"]["landmark_motion_scale"]),
            a=a, b=b, w=w, v_noise=v,
            params_file=params_file,
        )
        for k, val in overrides.items():
            if val is not None and hasattr(cfg, k):
                setattr(cfg, k, val)
        return cfg
