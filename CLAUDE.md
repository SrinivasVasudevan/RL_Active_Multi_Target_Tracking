# CLAUDE.md — orientation for a coding agent

Thesis code: **multi-robot active target tracking**. A 2-robot team with limited
field-of-view sensors tracks 4–7 moving targets; the policy is trained by a
model-based gradient through a differentiable Kalman filter.

Thesis PDF: `model_based_active_mapping/svasude7_ErrorFreeDraft.pdf`
(the results to reproduce are Tables 4.1 / 4.2 / 4.3 in Chapter 4).

Read this file first. The "Traps" section lists things that have already cost
real debugging time — check it before re-deriving them.

---

## The single most important fact

**`scripts/checkpoints/best_model_seed42_resume1.pth` is the thesis DRA
checkpoint.** Evaluated at `--seed 42 --trials 30`, it reproduces the thesis to
the decimal — Table 4.1, *and* the per-target-count breakdowns in 4.2 and 4.3:

| Metric (DRA) | Thesis 4.1 | Reproduced |
|---|---|---|
| Cumulative tracking (%) | 57.71 ± 20.89 | 57.7127 ± 20.8893 |
| Overlap (%) | 21.98 ± 28.64 | 21.9847 ± 28.6424 |
| Exclusive tracking (%) | 35.73 ± 15.87 | 35.7280 ± 15.8713 |
| Mean unique targets | 2.1821 ± 0.7454 | 2.1821 ± 0.7454 |

Table 4.2/4.3 by target count (cumulative %, overlap %): 4 → 66.79±22.86 /
40.55±33.41 · 5 → 58.71±11.72 / 11.29±7.54 · 6 → 66.84±22.39 / 35.00±35.96 ·
7 → 38.23±11.09 / 4.15±7.36. All exact.

Do **not** confuse it with `best_model_seed42.pth`, which is close but *not* the
thesis checkpoint (mean unique targets 2.1073, cumulative differs per target
count). If a number fails to match the thesis, check which checkpoint is loaded
before suspecting the code.

### Verify parity in ~1 minute, no Isaac needed

```bash
cd model_based_active_mapping/isaac_mdc
python3 tests/test_core_offline.py     # prints a MATCH table, exits PASS/FAIL
```

Run this before and after touching the agent, policy net, or estimator. It runs
the full episode loop with perfect pose tracking, so it is mathematically the
same experiment as the synthetic evaluation. If it passes and an Isaac run
disagrees, the bug is in the Isaac binding, not the policy.

---

## Three evaluation environments, one policy

| Environment | Where | Purpose |
|---|---|---|
| Synthetic (matplotlib) | `scripts/run_model_based_testing.py` | the thesis numbers |
| Gazebo + ROS 2 | `ros2_ws/src/mbam_gazebo_tracking/` | physics-sim qualitative check (thesis Fig. 4.2) |
| Isaac Sim / Multi-Drone-Control | `isaac_mdc/` | third renderer + publishable video |

All three share the same checkpoint and the same `TrackingStatistics` bookkeeping,
so their JSON output schemas match and are directly comparable.

**Isaac Sim: see `isaac_mdc/README.md`** — it is complete and current. It covers
the venv install (Isaac Sim 4.5.0 + IsaacLab v2.1.0), kinematic vs offboard
control modes, built-in Replicator recording, every CLI flag, and the
already-run 30-episode result. Key points worth knowing up front:

* `--control-mode kinematic` (default) teleports to the commanded pose, so
  numbers match the thesis exactly. `--control-mode offboard` runs the real PX4
  flight stack and will score *worse* — that gap is the interesting experiment,
  not a bug.
* Kinematic mode needs **neither PX4 nor Pegasus nor the crab-rl fork**.
* Use `--layout grid` (default); `--layout air` fetches from a Nucleus server and
  fails offline.
* Isaac segfaults during its own shutdown *after* results and videos are written.
  Known Kit teardown issue — harmless, do not chase it.
* The policy is 2-robot only (layer 1 is sized `3 + 3*(N-1)`); target count is free.

---

## Branches

Both feature branches fork from `best_simple_reward` (`37b0023`), the
linear-reward baseline that carries the thesis results.

| Branch | What it is |
|---|---|
| `best_simple_reward` | baseline; thesis state |
| `best_simple_reward_HRA` | pure Decomposed Reward Architecture (thesis §3.8) |
| `hybrid` | **current tip** — analytic exploration gradient + learned coord/pers critics; contains everything below |

`hybrid` descends from `best_simple_reward_HRA`, so it has all the DRA code plus
the checkpoints, the Isaac port, and the packaging fixes.

### What was learned (read before redoing this work)

1. In the baseline, `_episode_reward` is computed entirely from **detached**
   tensors. The persistence / coverage / continuity / loss / overlap terms
   contribute **zero gradient** — they only affect logging and best-checkpoint
   selection. The policy is trained purely by the analytic information gain
   backpropagated through the Kalman filter.
2. A **pure DRA** replaces that exact analytic gradient with a learned `Q^exp`,
   and this **measurably degrades tracking** (branch `best_simple_reward_HRA`).
3. Hence `hybrid`: keep the analytic gradient for exploration, where an exact
   gradient exists, and use Q-heads only for coordination and persistence, where
   none does. `--lam 0` recovers the baseline exactly, so it can only help or be
   tuned back to neutral.

---

## Checkpoint naming

All in `scripts/checkpoints/` (tracked; ~54 KB policies, ~114 KB critics):

| Pattern | Meaning |
|---|---|
| `best_model_seed42.pth` | baseline run |
| `best_model_seed42_resume1.pth` | **the thesis DRA checkpoint** (see above) |
| `best_model_seed42_hra_head` / `_hra_slow` | pure-DRA runs; `critic_seed42_hra_*` are their Q-heads |
| `best_model_seed42_hyb{0.05,0.2,1.0}` | hybrid, by `--lam` |
| `best_model_seed{0..4}_ms_s{N}_l{0.0,0.2}` | multi-seed sweep: seed N, lambda 0.0 / 0.2 |

Naming comes from `--tag`. Critics are saved beside policies as `critic_<tag>`.

---

## Running things

```bash
# synthetic evaluation (the thesis protocol)
cd model_based_active_mapping/scripts
python3 run_model_based_testing.py --model-path ./checkpoints/best_model_seed42_resume1.pth \
                                   --seed 42 --num-robots 2

# pure DRA: staged warm start (phase 1 fits critics, phase 2 alternates)
python3 run_hra_training.py --resume ./checkpoints/best_model_seed42_resume1.pth

# hybrid: --lam weights the critic surrogate; 0 == baseline
python3 run_hybrid_training.py --resume ./checkpoints/best_model_seed42_resume1.pth --lam 1.0
```

Both trainers warm-start from a baseline policy (`--resume` is required) and
default to `--out-dir ./checkpoints`, `--eval-trials 30`, `--eval-seed 42`.

---

## Traps

* **`MRMT/multi_robot_env.py` must stay tracked.** Every training and testing
  script does `sys.path.append(repo_root + "/MRMT")` and imports `MultiRobotEnv`
  from it. It was gitignored at one point, which made a fresh clone die at import.
  The ignore rule is gone; do not re-add it.
* **`scripts/checkpoints/` is only partly ignored on purpose.** `.gitignore` has
  `checkpoints/*` followed by `!checkpoints/*.pth`, so weights are versioned and
  future junk is not. Do not "simplify" this back to ignoring the folder.
* **Don't hardcode absolute paths.** `isaac_mdc/run_mbam_isaac.py` previously
  defaulted `--mbam-repo` to a `/home/svasude7/...` path. It now derives the
  checkout from `__file__` (matching `config.py`'s `DEFAULT_MBAM_REPO`), honours
  `$MBAM_REPO`, and fails fast with a clear message *before* launching Isaac.
* **ROS 2 `install/` still holds an absolute-path symlink** for
  `best_model_seed42.pth`, so it dangles on a fresh clone. `build/` was converted
  to a real file. `colcon build` regenerates `install/` anyway, so this only bites
  if you run the Gazebo stack without rebuilding.
* **`run_model_based_testing.py` must be run from `scripts/`.** It resolves
  params as `<cwd>/../params/params_compare.yaml`, i.e. relative to the *current
  working directory*, not to the script. Run it from anywhere else and it fails
  to find the params file. Episode count comes from that YAML
  (`num_test_trials: 30`), not from a flag, and `--seed` defaults to 0 — pass
  `--seed 42` for the thesis protocol.
* **`torch` is not in `requirements.txt`** — it arrives transitively via
  `stable-baselines3==1.6.0`. Fine today, fragile if sb3 is ever dropped.
* **`scripts/test_results/` and `scripts/logs/` are gitignored**, so evaluation
  JSONs are local-only. Copy anything you need to keep.

---

## Open threads

* **The hybrid sweep has no recorded results.** The `_hyb*` and `_ms_*`
  checkpoints exist, but no evaluation JSON was saved for them, so whether
  hybrid beats the baseline is currently **unknown**. Re-evaluate them with
  `run_model_based_testing.py` and keep the JSON. This is the obvious next step.
* Thesis §4.5 names the main open failure mode: **cluster convergence** — robots
  collapse onto the densest target cluster, driving overlap to 40.55% at 4
  targets while outlier targets go unobserved, and coverage collapses at 7
  targets (38.23%). Suggested fixes there: a global coverage term in the
  coordination penalty, or counterfactual credit assignment.
* `--control-mode offboard` in Isaac has not been run end-to-end; it measures
  what the PX4 flight stack costs in tracking performance.
