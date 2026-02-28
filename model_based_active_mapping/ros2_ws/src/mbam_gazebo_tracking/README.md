# mbam_gazebo_tracking

ROS 2 package for running `run_model_based_testing`-equivalent evaluation in Gazebo + RViz, using the attention policy checkpoint and live robot entities.

## What this package does

- Spawns **agent robots** and **target robots** in Gazebo.
- Loads `PolicyNetAtt` checkpoint and runs the policy loop for agent control (`cmd_vel`).
- Moves targets across episodes with the same landmark-motion style dynamics as the original testing setup.
- Uses **camera + lidar fusion** to simulate restricted FOV observation:
  - Camera detects whether target-colored pixels are present in-view.
  - Lidar provides range at corresponding camera-bearing angles.
  - Fused estimate gives relative/absolute target position used for state update.
- Publishes markers to RViz for true targets, estimated targets, and agent poses.
- Logs per-episode and aggregated tracking statistics to JSON.

## Package layout

- `mbam_gazebo_tracking/core/`
  - `policy_net_att.py`: copied policy-attention architecture.
  - `model_based_agent_att_ros.py`: ROS-side policy/filter agent logic.
  - `math_utils.py`: SE(2), FOV SDF, landmark motion helper functions.
  - `tracking_stats.py`: same tracking metrics and aggregation strategy.
  - `episode_sampler.py`: episode sampling and clustered target initialization.
- `mbam_gazebo_tracking/nodes/episode_runner_node.py`
  - Main ROS node that runs all test episodes and visualization streams.
- `launch/run_mbam_gazebo.launch.py`
  - Starts Gazebo, spawns models, starts RViz and runner node.
- `models/agent_bot/`
  - Blue agent model with camera + lidar + planar move plugin.
- `models/target_bot/`
  - Orange target model.
- `worlds/mbam_tracking.world`
  - World used for testing.
- `rviz/mbam_tracking.rviz`
  - Default RViz config with only two large camera panels:
    - `/agent_0/camera/image_raw`
    - `/agent_1/camera/image_raw`
- `config/params_compare.yaml`
  - Test configuration copy.
- `checkpoints/best_model_seed42.pth`
  - Checkpoint copy used by default.
- `scripts/run_model_based_testing_gazebo.sh`
  - One-command build + launch script.

## Prerequisites

- ROS 2 (tested design against Gazebo Classic flow used by `gazebo_ros`).
- Packages:
  - `gazebo_ros`
  - `rviz2`
  - `gazebo_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`
- Python deps:
  - `torch`
  - `numpy`
  - `opencv-python`
  - `pyyaml`

## Run

From the repo root:

```bash
bash ros2_ws/src/mbam_gazebo_tracking/scripts/run_model_based_testing_gazebo.sh
```

With overrides:

```bash
bash ros2_ws/src/mbam_gazebo_tracking/scripts/run_model_based_testing_gazebo.sh \
  num_robots:=2 \
  max_num_landmarks:=7 \
  num_test_trials:=10 \
  seed:=42
```

Override MBAM config/checkpoint explicitly:

```bash
bash ros2_ws/src/mbam_gazebo_tracking/scripts/run_model_based_testing_gazebo.sh \
  mbam_params_file:=/absolute/path/to/params_compare.yaml \
  model_path:=/absolute/path/to/best_model_seed42.pth
```

## Output

- JSON results are saved under:
  - `mbam_ros2_test_results/tracking_stats_<model>_seed<seed>_<timestamp>.json`

## Notes

- This package is standalone and does not import runtime code from the original package modules.
- Existing files in the original project were not edited.
- The ROS package uses camera color segmentation + lidar range gating for observation fusion, while target dynamics are controlled by the episode runner node.
- Launch also publishes a static `world -> map` transform so RViz fixed-frame rendering is immediately valid.
- Gazebo world is intentionally obstacle-free and uses a custom green ground plane to avoid robot falls/collisions.
