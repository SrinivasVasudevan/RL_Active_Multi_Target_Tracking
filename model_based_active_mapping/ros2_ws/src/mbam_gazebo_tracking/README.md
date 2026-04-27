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
  - Lidar point-marker visualization is off by default to avoid clutter.
  - Gazebo lidar ray rendering is also disabled in the agent model.
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
  - Turtlebot-like agent model with camera + lidar + planar move plugin.
- `models/target_bot/`
  - Red quadruped-style target model.
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

Use Gazebo model-database assets when available (for example, external ANYmal or burger-like models):

```bash
bash ros2_ws/src/mbam_gazebo_tracking/scripts/run_model_based_testing_gazebo.sh \
  agent_model_database:=turtlebot3_burger \
  target_model_database:=CERBERUS_ANYMAL_C_SENSOR_CONFIG_1
```

If database models are not installed locally, the package automatically falls back to the bundled local SDF models.
Set `strict_database_models:=true` to disable this fallback behavior.

`run_model_based_testing_gazebo.sh` also auto-assigns a `ROS_DOMAIN_ID` when unset, so one run is isolated from stale Gazebo/ROS processes from older sessions.

Lidar marker restriction controls:

```bash
publish_lidar_markers:=false
lidar_marker_stride:=6
restrict_lidar_visualization:=true
lidar_visualization_margin_rad:=0.05
```

Collision safety controls:

```bash
enable_collision_pause:=true
collision_lookahead_sec:=1.0
collision_robot_radius:=0.28
collision_target_radius:=0.30
collision_safety_margin:=0.10
```

When enabled, each robot command is paused (zero linear/angular velocity) if the predicted near-future motion would collide with another robot or a target.

## Output

- JSON results are saved under:
  - `mbam_ros2_test_results/tracking_stats_<model>_seed<seed>_<timestamp>.json`

## Notes

- This package is standalone and does not import runtime code from the original package modules.
- Existing files in the original project were not edited.
- The ROS package uses camera color segmentation + lidar range gating for observation fusion, while target dynamics are controlled by the episode runner node.
- Launch also publishes a static `world -> map` transform so RViz fixed-frame rendering is immediately valid.
- Gazebo world uses a custom green ground plane and decorative visual-only trees (non-collidable) to keep motion safe while improving scene realism.
- The run script uses workspace-local `GAZEBO_HOME` and defaults DDS to UDP transport to avoid common permission issues in restricted/containerized environments.
