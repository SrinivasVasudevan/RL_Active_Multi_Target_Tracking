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
- Adds a **real-robot deployment path** for Agilex LIMO:
  - one **central coordinator** node that consumes robot reports, maintains target tracks, runs the MBAM policy, and publishes planner commands
  - one **LIMO-side observer** node per robot that detects humans / robot targets from the onboard camera and sends back pose + target belief reports
  - one **LIMO-side safety controller** per robot that filters planner commands with local lidar before they reach the real `/cmd_vel`
  - one **cross-distro UDP transport path** so a ROS 2 Humble controller can communicate with ROS 2 Foxy LIMOs without direct inter-distro ROS topic exchange

## Package layout

- `mbam_gazebo_tracking/core/`
  - `policy_net_att.py`: copied policy-attention architecture.
  - `model_based_agent_att_ros.py`: ROS-side policy/filter agent logic.
  - `math_utils.py`: SE(2), FOV SDF, landmark motion helper functions.
  - `tracking_stats.py`: same tracking metrics and aggregation strategy.
  - `episode_sampler.py`: episode sampling and clustered target initialization.
  - `perception.py`: real-world 2D detection helpers, camera-ground projection, optional scan fusion.
  - `real_world_types.py`: JSON-serializable robot-report and detection payload types.
  - `track_manager.py`: central multi-robot target candidate tracker and velocity estimator.
- `mbam_gazebo_tracking/nodes/episode_runner_node.py`
  - Main ROS node that runs all test episodes and visualization streams.
- `mbam_gazebo_tracking/nodes/limo_observer_node.py`
  - Runs on each LIMO.
  - Uses camera detections to estimate target ground positions in the shared world frame.
  - Publishes robot pose plus detected target beliefs to the central processor.
- `mbam_gazebo_tracking/nodes/limo_central_coordinator_node.py`
  - Runs on the single processing unit.
  - Collects reports from all robots, maintains target tracks, executes the MBAM policy, and publishes `/ROBOT_NAME/mbam_cmd_vel`.
- `mbam_gazebo_tracking/nodes/limo_safety_controller_node.py`
  - Runs on each LIMO.
  - Subscribes to planner commands plus local lidar.
  - Slows, stops, or turns away from obstacles and walls before publishing the final `/cmd_vel`.
- `launch/run_mbam_gazebo.launch.py`
  - Starts Gazebo, spawns models, starts RViz and runner node.
- `launch/run_mbam_limo_edge.launch.py`
  - Starts one LIMO-side observer node and one LIMO-side safety controller node.
- `launch/run_mbam_limo_central.launch.py`
  - Starts the central coordinator node.
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
  - Optional for stronger human detection:
    - `ultralytics` with a local YOLO weights file

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

## Real LIMO deployment

### Architecture

- Each LIMO runs `limo_observer`.
- Each LIMO also runs `limo_safety_controller`.
- The observer subscribes to:
  - a **shared-frame pose** topic in `nav_msgs/Odometry`
  - the RGB camera image
  - camera calibration
  - optional `/scan` for range refinement
- The safety controller subscribes to:
  - planner commands from the central coordinator
  - local `/scan`
  - and publishes the filtered result to the real `/cmd_vel`
- The central processor runs `limo_central_coordinator`.
- The coordinator receives robot reports and publishes commands using either:
  - direct ROS topics for same-distro setups
  - UDP JSON for cross-distro setups such as **Humble controller + Foxy LIMOs**
- In the Humble/Foxy case, the coordinator listens for UDP reports and sends UDP commands.
- Each LIMO keeps ROS local for sensors and `/cmd_vel`.
- The coordinator still publishes:
  - `/mbam/real_world_markers` for RViz

### Humble/Foxy compatibility

- A ROS 2 Humble machine and a ROS 2 Foxy machine should not be treated as a reliable direct ROS 2 communication pair for this application.
- This package now avoids that dependency by moving the **machine-to-machine seam** to UDP JSON.
- Local machine behavior stays ROS-native:
  - the Foxy LIMO still reads `Odometry`, `Image`, `CameraInfo`, `LaserScan`, and writes `/cmd_vel`
  - the Humble controller still runs the MBAM node and RViz normally

### Important assumptions

- All robot poses must already be expressed in the **same global frame** such as `map`.
  The central tracker cannot merge detections correctly if each robot only publishes its own local `odom` frame.
- For **human targets**, the observer detects people on the full frame and also on a lower-body crop, then projects the detection footpoint to the ground plane.
  This is intended for cases where mostly the lower torso / legs are visible.
- For **robot targets**, the observer supports:
  - ArUco markers on the target robots
  - red-color segmentation fallback
- Camera detections are converted to ground positions using camera height and pitch.
  If `/scan` is available, the observer can refine the detection range with lidar while still using the camera as the detector.
- The last command sent to the robot is always passed through the local safety controller.
  This is the layer that prevents wall / object collisions even if the central policy asks for an unsafe motion.

### LIMO safety behavior

- Forward motion is slowed as the robot approaches an obstacle and stopped completely inside the emergency distance.
- Reverse motion is filtered separately with its own rear stop distances.
- Turning into a nearby wall or object is suppressed using left / right lidar sectors.
- If the robot is blocked in front and no safe forward motion exists, the controller can keep a local escape turn instead of pushing into the obstacle.
- If planner commands or lidar data time out, the safety controller publishes zero velocity.

### Recommended target setup

- For robot-vs-robot tracking, mount a visible ArUco tag or red fiducial panel on each target robot.
- For human tracking, keep `enable_people_detection:=true` and provide a YOLO weights path if available for better robustness than the OpenCV HOG fallback.

### Example commands

On each LIMO, launch the edge stack with that robot's topics and calibration:

```bash
source ros2_ws/setup.bash

ros2 launch mbam_gazebo_tracking run_mbam_limo_edge.launch.py \
  robot_name:=limo0 \
  report_transport_mode:=udp \
  controller_host:=192.168.50.1 \
  controller_report_port:=15000 \
  odom_topic:=/global_ekf/odom \
  base_frame:=base_link \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  scan_topic:=/scan \
  command_transport_mode:=udp \
  command_port:=15001 \
  output_cmd_topic:=/cmd_vel \
  world_frame:=map \
  camera_height_m:=0.32 \
  camera_pitch_rad:=0.30 \
  yolo_model_path:=/absolute/path/to/yolo_weights.pt
```

Launch the second robot the same way with `robot_name:=limo1`.

On the central processor:

```bash
source ros2_ws/setup.bash

ros2 launch mbam_gazebo_tracking run_mbam_limo_central.launch.py \
  robot_names:=limo0,limo1 \
  transport_mode:=udp \
  report_bind_host:=0.0.0.0 \
  report_port:=15000 \
  robot_command_targets_csv:="limo0=192.168.50.101:15001,limo1=192.168.50.102:15001" \
  marker_frame:=map \
  max_num_landmarks:=7 \
  track_association_distance_m:=1.5
```

If you prefer parameter files, use:

```bash
ros2 run mbam_gazebo_tracking limo_observer --ros-args --params-file \
  ros2_ws/src/mbam_gazebo_tracking/config/limo_edge.params.yaml

ros2 run mbam_gazebo_tracking limo_safety_controller --ros-args --params-file \
  ros2_ws/src/mbam_gazebo_tracking/config/limo_safety.params.yaml

ros2 run mbam_gazebo_tracking limo_central_coordinator --ros-args --params-file \
  ros2_ws/src/mbam_gazebo_tracking/config/limo_central.params.yaml
```

### Deployment tuning notes

- `camera_height_m` and `camera_pitch_rad` matter directly for ground-plane projection accuracy.
- `track_association_distance_m` should be increased if targets move fast or your localization is noisy.
- `control_period_sec` defaults to the MBAM training timestep (`tau` from `params_compare.yaml`); override it only if your hardware loop needs a different cadence.
- `search_angular_velocity` controls how the robots scan when no target tracks are currently active.
- `ros2_ws/setup.bash` auto-detects the local ROS 2 distro from `/opt/ros`, and you can override that with `MBAM_ROS_DISTRO=foxy` or `MBAM_ROS_DISTRO=humble`.
- Use `MBAM_EXTRA_UNDERLAYS=/path/to/limo/install:/path/to/other/install` when a machine needs extra overlays with machine-specific paths.
- The observer now publishes pose-only heartbeat reports until camera image and calibration are ready, and the central coordinator will issue degraded search commands to robots that are already reporting instead of freezing the whole team.
- If the configured odometry topic is missing, the observer falls back to TF using `world_frame -> base_frame`. Set `world_frame:=odom` if the LIMO only has local odometry, or keep `world_frame:=map` only if both robots truly share the same global frame.
- `controller_host`, `report_port`, `command_port`, and `robot_command_targets_csv` are the transport settings that matter for Humble/Foxy interoperation.
- `forward_emergency_stop_distance_m` and `forward_slowdown_distance_m` are the main wall / obstacle safety knobs on the LIMO side.
- `turn_clearance_distance_m` and `side_clearance_distance_m` control how aggressively the robot rejects turns into nearby walls or objects.

## Output

- JSON results are saved under:
  - `mbam_ros2_test_results/tracking_stats_<model>_seed<seed>_<timestamp>.json`

## Notes

- This package is standalone and does not import runtime code from the original package modules.
- Existing files in the original project were not edited.
- The ROS package uses camera color segmentation + lidar range gating for observation fusion, while target dynamics are controlled by the episode runner node.
- The real-robot path uses a distributed architecture:
  - edge observers on the LIMOs
  - edge safety controllers on the LIMOs
  - one central policy coordinator
- ROS 1 target-robot bringup lives separately in [ros1_targets_ws/README.md](/home/svasude7/HRA/RL_Active_Multi_Target_Tracking/model_based_active_mapping/ros1_targets_ws/README.md).
- Launch also publishes a static `world -> map` transform so RViz fixed-frame rendering is immediately valid.
- Gazebo world uses a custom green ground plane and decorative visual-only trees (non-collidable) to keep motion safe while improving scene realism.
- The run script uses workspace-local `GAZEBO_HOME` and defaults DDS to UDP transport to avoid common permission issues in restricted/containerized environments.
