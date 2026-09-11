"""Verify the Isaac-free coordinator against the thesis numbers.

Runs the full MBAM episode loop with a perfect-tracking stand-in for the
simulator (control_mode="kinematic": the robots reach exactly the pose the
policy asked for). That makes the loop mathematically identical to the
synthetic evaluation, so it must reproduce Table 4.1 of the thesis exactly.

If this passes, any discrepancy in an Isaac run comes from the Isaac binding
(pose tracking, frames, timing) and not from the policy or bookkeeping.

Usage:  python3 tests/test_core_offline.py [--model-path ...] [--trials 30]
"""

import argparse
import os
import sys

import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mbam_isaac.config import MBAMConfig  # noqa: E402
from mbam_isaac.coordinator import MBAMCoordinator  # noqa: E402

# Thesis Table 4.1, DRA column (mean, std).
THESIS_DRA = {
    "cumulative_tracking_pct": (57.71, 20.89),
    "overlap_pct": (21.98, 28.64),
    "exclusive_tracking_pct": (35.73, 15.87),
    "mean_unique_targets": (2.1821, 0.7454),
}


def run(cfg: MBAMConfig) -> dict:
    torch.manual_seed(cfg.seed)
    coord = MBAMCoordinator(cfg)
    for _ in range(cfg.num_test_trials):
        state = coord.start_episode()
        x = state.x.clone()
        while not coord.episode_done:
            desired_x, _mu = coord.policy_step(x)
            x = desired_x           # perfect tracking stands in for the simulator
            coord.observe(x)
        coord.finish_episode()
    return coord.results()


def main() -> int:
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-path", default=os.path.join(repo, "scripts", "checkpoints",
                                                         "best_model_seed42_resume1.pth"))
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--tol", type=float, default=0.01)
    args = ap.parse_args()

    cfg = MBAMConfig.from_params_yaml(mbam_repo=repo, model_path=args.model_path,
                                      seed=args.seed, num_test_trials=args.trials)
    res = run(cfg)

    print(f"checkpoint : {os.path.basename(cfg.model_path)}")
    print(f"seed       : {cfg.seed}   episodes: {res['num_test_trials']}   robots: {cfg.num_robots}\n")
    print(f"{'metric':26s} {'thesis (DRA)':>20s} {'coordinator':>20s}   result")
    print("-" * 78)

    ok = True
    for key, (t_mean, t_std) in THESIS_DRA.items():
        m, s = res["thesis_metrics"][key]
        hit = abs(m - t_mean) <= args.tol and abs(s - t_std) <= args.tol
        ok &= hit
        print(f"{key:26s} {t_mean:9.4f}+/-{t_std:8.4f} {m:9.4f}+/-{s:8.4f}   {'MATCH' if hit else 'DIFFERS'}")

    print("\n" + ("PASS - coordinator reproduces the thesis DRA column."
                  if ok else "FAIL - coordinator does not match the thesis."))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
