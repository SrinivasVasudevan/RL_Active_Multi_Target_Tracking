"""Run the MBAM policy inside the Multi-Drone-Control Isaac Sim template.

Drop this file (and the mbam_isaac/ package) into the root of a
Multi-Drone-Control checkout, then from the IsaacLab environment:

    python3 run_mbam_isaac.py --model-path /path/to/best_model_seed42_resume1.pth

Useful flags:
    --control-mode kinematic   exact pose tracking, numbers comparable to the thesis
    --control-mode offboard    full PX4 flight stack (needs PX4 + Pegasus configured)
    --trials 30 --seed 42      matches the thesis evaluation protocol
"""

import argparse
import os
import sys


def main() -> int:
    # isaac_mdc/ lives inside model_based_active_mapping/, so the checkout is one
    # level up from this file. MBAM_REPO overrides it, which is what you want when
    # this file is copied into a Multi-Drone-Control checkout elsewhere.
    default_repo = os.environ.get(
        "MBAM_REPO",
        os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")),
    )

    ap = argparse.ArgumentParser(description="MBAM multi-target tracking on Isaac Sim / MDC")
    ap.add_argument("--mbam-repo", default=default_repo,
                    help="path to model_based_active_mapping (source of the agent + params)")
    ap.add_argument("--model-path", default=None, help="policy checkpoint (.pth)")
    ap.add_argument("--params-file", default=None, help="defaults to <repo>/params/params_compare.yaml")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--num-robots", type=int, default=2)
    ap.add_argument("--max-targets", type=int, default=7)
    ap.add_argument("--num-clusters", type=int, default=2)
    ap.add_argument("--clustering-prob", type=float, default=0.65)
    ap.add_argument("--control-mode", choices=["kinematic", "offboard"], default="kinematic")
    ap.add_argument("--altitude", type=float, default=2.5)
    ap.add_argument("--layout", default="grid", choices=["air", "grid"])
    ap.add_argument("--output-dir", default="")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--no-record", action="store_true", help="disable viewport recording")
    ap.add_argument("--record-fps", type=int, default=5)
    ap.add_argument("--record-episodes", type=int, default=0,
                    help="record only the first N episodes (0 = all)")
    ap.add_argument("--keep-frames", action="store_true", help="keep raw PNG frames")
    args = ap.parse_args()

    # Fail before launching Isaac: a wrong --mbam-repo otherwise surfaces as an
    # opaque ImportError deep inside the coordinator.
    sentinel = os.path.join(args.mbam_repo, "params", "params_compare.yaml")
    if not os.path.exists(sentinel):
        ap.error(
            f"--mbam-repo {args.mbam_repo!r} is not a model_based_active_mapping "
            f"checkout (missing {sentinel}). Pass --mbam-repo or set MBAM_REPO."
        )

    # The Isaac app must be launched before any isaaclab/isaacsim import.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    # MDC's sim.app.init_app() asserts headless is unsupported. That assert is
    # stale for this stack - headless launches fine and is what you want for an
    # unattended run, since recording goes through a Replicator render product
    # rather than the GUI viewport. Launch directly and publish the app into
    # MDC's module global so its get_app() keeps working.
    # Use isaacsim's SimulationApp rather than isaaclab's AppLauncher: the latter
    # brings up a minimal Kit experience without omni.replicator / omni.kit.viewport,
    # which the recorder needs.
    from isaacsim import SimulationApp
    import sim.app as mdc_app
    mdc_app._app = SimulationApp({"headless": args.headless})

    from mbam_isaac.config import MBAMConfig
    from mbam_isaac.mbam_env import MBAMDroneEnv

    model_path = args.model_path or os.path.join(
        args.mbam_repo, "scripts", "checkpoints", f"best_model_seed{args.seed}.pth")

    cfg = MBAMConfig.from_params_yaml(
        params_file=args.params_file or "",
        mbam_repo=args.mbam_repo,
        model_path=model_path,
        seed=args.seed,
        num_test_trials=args.trials,
        num_robots=args.num_robots,
        max_num_landmarks=args.max_targets,
        num_clusters=args.num_clusters,
        clustering_prob=args.clustering_prob,
        control_mode=args.control_mode,
        flight_altitude=args.altitude,
        layout=args.layout,
        record=not args.no_record,
        record_fps=args.record_fps,
        record_episodes=args.record_episodes,
        keep_frames=args.keep_frames,
    )

    env = MBAMDroneEnv(cfg, output_dir=args.output_dir)
    try:
        env.run()
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
