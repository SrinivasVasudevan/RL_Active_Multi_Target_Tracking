"""MBAMDroneEnv - the MDC IsaacEnv tweaked to the thesis setup.

Differences from the stock MDC DroneEnv:

  * 2 robots instead of the demo's 6, driven by one shared MBAM attention policy
    rather than per-drone teleop/PID controllers.
  * N moving targets (4-7, resampled per episode) as kinematic markers, replacing
    the pretrained-RL crabs. Target motion uses the thesis motion model so the
    numbers stay comparable.
  * An episode loop: 30 trials, each of horizon 5*num_targets policy steps, with
    tracking statistics written out as JSON.
  * Policy runs at tau = 1 s while the sim ticks at 1/250 s, so each decision is
    held for `cfg.decimation` sim steps.
"""

from __future__ import annotations

import json
import os
import time
from typing import List

import numpy as np
import torch

from sim.isaac_env import IsaacEnv

from .config import MBAMConfig
from .coordinator import MBAMCoordinator
from .recorder import OverheadCamera, ViewportRecorder
from .robot_bridge import make_bridges
from .targets import TargetMarkers
from .viz import FovWedge


class MBAMDroneEnv(IsaacEnv):
    def __init__(self, cfg: MBAMConfig, output_dir: str = ""):
        super().__init__(layout=cfg.layout)
        self.cfg = cfg
        self.output_dir = output_dir or os.path.join(os.getcwd(), "mbam_isaac_results")
        os.makedirs(self.output_dir, exist_ok=True)

        torch.manual_seed(cfg.seed)
        np.random.seed(cfg.seed)

        self.coord = MBAMCoordinator(cfg)
        self.bridges: List = []
        self.targets = None
        self.camera = None
        self.recorder = None
        self.fov_wedges: List = []
        self._initial_poses = np.zeros((cfg.num_robots, 3))

    # -- Isaac lifecycle ---------------------------------------------------

    def post_init(self):
        self.targets = TargetMarkers(num_markers=self.cfg.max_num_landmarks,
                                     altitude=self.cfg.target_altitude)
        self.bridges = make_bridges(self.cfg, self, self._initial_poses)
        for b in self.bridges:
            if hasattr(b, "post_init"):
                b.post_init()

        if self.cfg.show_fov:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            palette = [(0.20, 0.65, 1.00), (1.00, 0.55, 0.15), (0.60, 1.00, 0.40), (0.90, 0.40, 0.90)]
            self.fov_wedges = [
                FovWedge(stage, f"/World/mbam_fov/robot_{i}", radius=self.cfg.radius,
                         psi=self.cfg.psi, color=palette[i % len(palette)])
                for i in range(self.cfg.num_robots)
            ]

        self.camera = OverheadCamera()
        self.camera.activate()
        self.recorder = ViewportRecorder(
            output_dir=self.output_dir, fps=self.cfg.record_fps,
            enabled=self.cfg.record, keep_frames=self.cfg.keep_frames,
        )
        self.recorder.attach(self.camera.path)

    @property
    def _ticks_per_step(self) -> int:
        """Sim ticks to hold each policy decision for.

        Kinematic mode teleports, so it only needs a render; offboard mode must
        give PX4 the full tau to actually fly to the setpoint.
        """
        return self.cfg.decimation if self.cfg.control_mode == "offboard" else self.cfg.kinematic_ticks

    def _frame_camera(self, x) -> None:
        """Keep robots and targets in shot."""
        pts = np.vstack([np.asarray(x)[:, :2], self.coord.mu_real.detach().numpy()])
        self.camera.frame_points(pts, margin=self.cfg.camera_margin)

    def _update_wedges(self, x) -> None:
        for i, wedge in enumerate(self.fov_wedges):
            wedge.set_pose(float(x[i][0]), float(x[i][1]), float(x[i][2]))

    def _tick(self, n: int = 1):
        """Advance the simulator, servicing any per-step controller work."""
        for _ in range(n):
            self.world.step(render=True)
            for b in self.bridges:
                if hasattr(b, "post_step"):
                    b.post_step()

    # -- evaluation --------------------------------------------------------

    def run(self) -> dict:
        """Run all episodes and write the results JSON. Returns the results dict."""
        if not self.init_reset:
            self.reset()

        for _ in range(self.cfg.num_test_trials):
            self._run_episode()

        results = self.coord.results()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        name = os.path.splitext(os.path.basename(self.cfg.model_path))[0]
        path = os.path.join(self.output_dir,
                            f"tracking_stats_{name}_seed{self.cfg.seed}_{self.cfg.control_mode}_{stamp}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)

        m = results["thesis_metrics"]
        print("\n" + "=" * 72)
        print(f"MBAM on Isaac ({self.cfg.control_mode} mode) - "
              f"{results['num_test_trials']} episodes, {self.cfg.num_robots} robots")
        print("=" * 72)
        for k in ("cumulative_tracking_pct", "overlap_pct", "exclusive_tracking_pct", "mean_unique_targets"):
            print(f"  {k:26s} {m[k][0]:9.4f} +/- {m[k][1]:8.4f}")
        print(f"\n> results written to {path}")
        return results

    def _run_episode(self):
        state = self.coord.start_episode()

        x = state.x.clone()
        self._initial_poses = x.numpy().copy()
        for i, b in enumerate(self.bridges):
            b.reset(x[i].tolist())
        self._update_wedges(x)
        self.targets.update(self.coord.mu_real)

        self.camera.frame_env(float(state.env_size[0]), margin=self.cfg.camera_margin)
        self._frame_camera(x)
        record_this = self.cfg.record and (
            self.cfg.record_episodes <= 0 or self.coord.episode_index <= self.cfg.record_episodes
        )
        self.recorder.enabled = self.cfg.record and record_this
        self.recorder.start_episode(self.coord.episode_index)

        # Let the flight stack settle on the start pose before the episode counts.
        self._tick(self._ticks_per_step)

        print(f"[MBAM] episode {self.coord.episode_index}/{self.cfg.num_test_trials}: "
              f"{self.coord.num_landmarks} targets, horizon {self.coord.horizon}")

        while not self.coord.episode_done:
            desired_x, mu_next = self.coord.policy_step(x)

            for i, b in enumerate(self.bridges):
                b.command_pose(desired_x[i].tolist())
            self._update_wedges(desired_x)
            self.targets.update(mu_next)

            self._tick(self._ticks_per_step)

            # Fold back the poses the simulator actually achieved.
            x = torch.tensor(np.stack([b.read_pose() for b in self.bridges]), dtype=torch.float32)
            fov = self.coord.observe(x)

            # Visibility is only known after observe(), so the scene has to be
            # re-rendered before capturing - otherwise every frame shows the
            # previous step's tracked/untracked colouring.
            self._update_wedges(x)
            self.targets.update(self.coord.mu_real, visible_any=fov.any(dim=0).tolist())
            self._frame_camera(x)
            self._tick(1)

            self.recorder.capture()

        video = self.recorder.finish_episode()
        summary = self.coord.finish_episode()
        if video:
            summary["video"] = video
        t = summary["totals"]
        print(f"[MBAM]   cumulative {t['cumulative_percentage']:.1f}%  overlap {t['overlap_percentage']:.1f}%")

    def close(self):
        for b in self.bridges:
            b.close()
        super().close()
