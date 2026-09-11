# MBAM on the Multi-Drone-Control Isaac Sim template

Runs the thesis tracking policy (2 robots, 4–7 moving targets) inside
[lil-cosine/Multi-Drone-Control](https://github.com/lil-cosine/Multi-Drone-Control),
so the same checkpoint can be evaluated in a third environment alongside the
matplotlib simulator and the Gazebo/ROS 2 port.

## Verdict: yes, it fits — with one real caveat

| MDC provides | MBAM needs | Fit |
|---|---|---|
| `DRONE_CONFIGS` list, arbitrary fleet size | exactly 2 robots | direct — set the list to 2 |
| `OffboardController.set_target_position/​set_target_yaw_deg` | SE(2) pose setpoint per control step | direct — this is the natural seam |
| animals as independently-moving entities | 4–7 moving targets | replaced (see below) |
| `IsaacEnv` `post_init`/`step`/`reset` hooks | episode loop | direct |
| `sim_dt = 1/250 s` | `tau = 1 s` policy step | decimation of 250 sim ticks |

**The caveat:** the MDC crab/ant agents move under their *own* pretrained RL
policies. The MBAM policy was trained against targets following a biased random
walk (`landmark_motion_real`). Using the crabs as targets would change the target
dynamics and make the numbers incomparable to the thesis. So targets here are
kinematic markers driven by the thesis motion model — the same choice the
existing Gazebo port makes. The crabs stay available as scenery.

Two further mismatches are handled rather than ignored:

* **SE(2) vs SE(3).** Drones fly a fixed-altitude slice (`--altitude`, default
  2.5 m) with yaw as heading. The policy is untouched.
* **Robot count is fixed by the checkpoint.** The policy encodes each robot's own
  pose plus every teammate's relative pose, so layer 1 is sized `3 + 3*(N-1)`.
  `best_model_seed42_resume1.pth` is a 2-robot checkpoint and cannot be run with
  1 or 3 robots without retraining. Target count *is* free to vary — the landmark
  encoder is shared per target. The loader raises a clear error on mismatch.

## Layout

```
mbam_isaac/
  config.py           MBAMConfig - the whole scenario in one dataclass
  episode_sampler.py  episode sampling, bit-identical to MRMT/multi_robot_env.py
  coordinator.py      policy + estimator + statistics loop   (no Isaac imports)
  targets.py          kinematic target markers               (Isaac)
  robot_bridge.py     SE(2) pose -> drone setpoint           (Isaac)
  mbam_env.py         MBAMDroneEnv                           (Isaac)
run_mbam_isaac.py     standalone entry point
tests/test_core_offline.py   thesis-parity check, runs without Isaac
```

Everything policy-related lives in `coordinator.py` and imports nothing from
Isaac. `mbam_env.py` only asks the coordinator what should happen next, applies
it, and reports back the poses actually achieved.

The MBAM agent, policy network and math utilities are imported from the thesis
repo via `--mbam-repo` rather than vendored, so the Isaac runs use bit-identical
code to `scripts/run_model_based_testing.py`.

## Install (done on this machine)

Isaac Sim is installed in a dedicated venv at `~/isaac_venv` (Python 3.10,
Isaac Sim 4.5.0, IsaacLab v2.1.0). Kinematic mode needs **neither PX4, Pegasus,
nor the crab-rl IsaacLab fork** - that whole branch of the MDC setup guide is
only required for `--control-mode offboard`.

```bash
python3.10 -m venv ~/isaac_venv
~/isaac_venv/bin/pip install --upgrade pip "setuptools<81" wheel
~/isaac_venv/bin/pip install "isaacsim[all,extscache]==4.5.0.0" --extra-index-url https://pypi.nvidia.com

git clone --depth 1 -b v2.1.0 https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab
~/isaac_venv/bin/pip install --no-build-isolation "flatdict==4.0.1"   # needs pkg_resources
~/isaac_venv/bin/pip install -e ~/IsaacLab/source/isaaclab
~/isaac_venv/bin/pip install h5py                                     # silences isaaclab_tasks

# MDC's USD assets are Git LFS pointers - without LFS the drone mesh will not load
git clone https://github.com/lil-cosine/Multi-Drone-Control.git ~/Multi-Drone-Control
cd ~/Multi-Drone-Control && git lfs install --local && git lfs pull
```

Isaac Sim requires accepting the NVIDIA Omniverse EULA; scripted runs set
`OMNI_KIT_ACCEPT_EULA=YES`.

Three things worth knowing, all handled in the code:

* `sim/app.py` asserts headless is unsupported. That is stale for this stack -
  headless works. The runner launches `SimulationApp` directly and publishes it
  into `sim.app._app` so MDC's `get_app()` still works.
* Launch via isaacsim's `SimulationApp`, **not** isaaclab's `AppLauncher` - the
  latter brings up a minimal Kit experience with no `omni.replicator`, so
  recording silently turns off.
* Use `--layout grid` (the default). `--layout air` uses `GroundPlaneCfg`, which
  fetches its asset from a Nucleus server and fails on an offline machine.

## Two control modes

* `--control-mode kinematic` (default) — teleports a visual iris prim to the
  commanded pose each step. Pose tracking is exact, so results match the
  synthetic evaluation. This is the "same policy, different renderer" run, and
  the one to use for numbers comparable to the thesis.
* `--control-mode offboard` — the full MDC stack (Pegasus `Multirotor` + PX4
  backend + `OffboardController`), streaming `SET_POSITION_TARGET_LOCAL_NED`.
  Realistic flight dynamics; the drone lags the 1 s setpoints, so tracking
  numbers will be *worse* than the thesis. This is the interesting experiment:
  it measures what the flight stack costs you.

## Verified: runs in Isaac Sim, reproduces the thesis exactly

The full 30-episode protocol was run inside Isaac Sim (kinematic mode, seed 42)
and reproduces every thesis number to the decimal:

| Metric (DRA) | Thesis Table 4.1 | Isaac Sim run |
|---|---|---|
| Cumulative tracking (%) | 57.71 ± 20.89 | **57.7127 ± 20.8893** |
| Overlap (%) | 21.98 ± 28.64 | **21.9847 ± 28.6424** |
| Exclusive tracking (%) | 35.73 ± 15.87 | **35.7280 ± 15.8713** |
| Mean unique targets | 2.1821 ± 0.7454 | **2.1821 ± 0.7454** |

Tables 4.2 and 4.3 match per target count as well (4/5/6/7 targets, cumulative
and overlap all identical). This is expected rather than lucky: kinematic mode
tracks the commanded pose exactly, so the episode loop is mathematically the
same as the synthetic evaluation - it is the *same experiment in a different
renderer*, which is exactly what makes the footage publishable alongside the
thesis numbers.

`tests/test_core_offline.py` checks the same four numbers without Isaac
installed, so the policy path can be re-verified on any machine.

## Recording

Recording is built in - no external screen recorder needed. A top-down camera
follows the action (centred on robots + targets, zoomed to contain them, with
smoothing), and one frame is captured per policy step through a Replicator
render product, then assembled into one MP4 per episode:

```
<output-dir>/videos/mbam_episode_001.mp4 ... _030.mp4
<output-dir>/tracking_stats_<model>_seed<N>_<mode>_<stamp>.json
```

The 30-episode run produced 30 videos, 960x720, ~47 MB total.

What you see in a frame: the grid world, two drones, each drone's sensor wedge
(blue / orange - the exact `triangle_SDF` footprint, `radius` long with
half-angle `psi`), and the targets coloured **green when tracked** by at least
one robot and **orange when not**.

Recording works headless because it goes through Replicator rather than the GUI
viewport. Drop `--headless` to watch it live in the Isaac window as well.

## Usage

```bash
# copy this package into the MDC checkout (already done at ~/Multi-Drone-Control)
cp -r isaac_mdc/mbam_isaac isaac_mdc/run_mbam_isaac.py ~/Multi-Drone-Control/

cd ~/Multi-Drone-Control
OMNI_KIT_ACCEPT_EULA=YES ~/isaac_venv/bin/python run_mbam_isaac.py \
    --model-path ~/HRA/RL_Active_Multi_Target_Tracking/model_based_active_mapping/scripts/checkpoints/best_model_seed42_resume1.pth \
    --seed 42 --trials 30 --headless \
    --output-dir ~/mbam_isaac_results
```

Startup takes ~3 minutes (Kit extension load); the 30 episodes then run in a few
minutes. Isaac Sim segfaults during its own shutdown *after* results and videos
are written - a known Kit teardown issue, harmless here.

Results land in `mbam_isaac_results/tracking_stats_<model>_seed<N>_<mode>_<stamp>.json`,
in the same schema as the synthetic and Gazebo runs, with a `thesis_metrics`
block holding the four Table 4.1 numbers.

Run the parity check without Isaac at any time:

```bash
python3 tests/test_core_offline.py
```

## Knobs

| flag | default | note |
|---|---|---|
| `--num-robots` | 2 | must match the checkpoint |
| `--max-targets` | 7 | targets sampled in `[4, max]`; free to change |
| `--num-clusters` | 2 | target cluster count |
| `--clustering-prob` | 0.65 | probability an episode is clustered vs uniform |
| `--control-mode` | kinematic | `kinematic` \| `offboard` |
| `--altitude` | 2.5 | flight altitude of the SE(2) slice |
| `--layout` | grid | `grid` \| `air` (air needs Nucleus) |
| `--no-record` | off | disable video capture |
| `--record-fps` | 5 | 1 policy step = 1 frame |
| `--record-episodes` | 0 | record only the first N (0 = all) |
| `--headless` | off | omit to watch live in the Isaac window |
| `--seed` / `--trials` | 42 / 30 | thesis protocol |
