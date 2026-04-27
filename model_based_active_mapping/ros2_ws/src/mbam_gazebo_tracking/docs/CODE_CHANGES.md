# Code Changes and Documentation

This document provides a full list of new files added for ROS 2 Gazebo/RViz testing and the purpose of each file.

## New package root

- `ros2_ws/src/mbam_gazebo_tracking/package.xml`
  - ROS 2 package metadata and runtime dependencies.
- `ros2_ws/src/mbam_gazebo_tracking/setup.py`
  - Python package build/install entry, data-file installation, console script registration.
- `ros2_ws/src/mbam_gazebo_tracking/setup.cfg`
  - Script install path settings.
- `ros2_ws/src/mbam_gazebo_tracking/resource/mbam_gazebo_tracking`
  - Ament resource index file.
- `ros2_ws/src/mbam_gazebo_tracking/README.md`
  - End-user documentation and run instructions.

## Core algorithm and helpers

- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/math_utils.py`
  - Ported kinematics and FOV math (`se2_kinematics`, `triangle_sdf`, `phi`, landmark motion).
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/policy_net_att.py`
  - Policy attention network architecture used for inference.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/model_based_agent_att_ros.py`
  - Attention-agent policy wrapper for ROS runtime.
  - Includes planning, reward bookkeeping, and belief update from sensor-fused observations.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/tracking_stats.py`
  - Per-target, per-robot tracking summary and aggregated metrics.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/episode_sampler.py`
  - Episode initialization sampler with clustered target generation.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/network_utils.py`
  - UDP transport helpers for Humble/Foxy communication without direct ROS 2 inter-distro topic exchange.

## ROS node

- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/nodes/episode_runner_node.py`
  - Main runner node equivalent to testing loop.
  - Key responsibilities:
    - Load YAML params and checkpoint.
    - Reset and run multiple episodes.
    - Control agent robots with `PolicyNetAtt` output over `/agent_i/cmd_vel`.
    - Move target robots each step with landmark-like dynamics via `/gazebo/set_entity_state`.
    - Fuse camera detections + lidar ranges to construct restricted-FOV target observations.
    - Update internal belief/filter and tracking stats.
    - Publish RViz `MarkerArray` and write JSON summaries.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/nodes/limo_observer_node.py`
  - Real-world edge node for each Agilex LIMO.
  - Subscribes to robot pose, RGB camera, camera intrinsics, and optional lidar.
  - Detects human targets and robot targets, projects them to ground/world coordinates, and publishes JSON reports.
  - Supports ROS-topic transport and UDP JSON transport.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/nodes/limo_central_coordinator_node.py`
  - Real-world central node.
  - Subscribes to per-robot reports, builds target tracks, estimates target velocity, runs MBAM policy inference, and publishes planner velocity commands.
  - Supports ROS-topic transport and UDP JSON transport.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/nodes/limo_safety_controller_node.py`
  - Real-world edge safety node for each Agilex LIMO.
  - Subscribes to planner commands plus local lidar and publishes the final safe `/cmd_vel`.
  - Adds obstacle and wall avoidance using sector-based lidar braking and turn suppression.
  - Supports receiving planner commands over ROS topics or UDP JSON.

## Real-world core helpers

- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/perception.py`
  - 2D detection backends and geometric projection helpers.
  - Supports:
    - person detection via YOLO (if available) or OpenCV HOG fallback
    - robot detection via ArUco markers and red-color segmentation fallback
    - optional camera+lidar range refinement
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/real_world_types.py`
  - Shared report / detection payload dataclasses used by the LIMO-side and central nodes.
  - Also includes a serialized velocity-command payload for UDP command transport.
- `ros2_ws/src/mbam_gazebo_tracking/mbam_gazebo_tracking/core/track_manager.py`
  - Maintains persistent target candidates across multiple robot reports using nearest-neighbor association and simple velocity estimation.

## Launch and simulation assets

- `ros2_ws/src/mbam_gazebo_tracking/launch/run_mbam_gazebo.launch.py`
  - Launches Gazebo world, spawns all entities, starts RViz and runner node.
- `ros2_ws/src/mbam_gazebo_tracking/launch/run_mbam_limo_edge.launch.py`
  - Launches one LIMO-side observer node and one LIMO-side safety controller node.
- `ros2_ws/src/mbam_gazebo_tracking/launch/run_mbam_limo_central.launch.py`
  - Launches the central policy coordinator.
- `ros2_ws/src/mbam_gazebo_tracking/worlds/mbam_tracking.world`
  - Gazebo world definition.
- `ros2_ws/src/mbam_gazebo_tracking/models/agent_bot/model.config`
- `ros2_ws/src/mbam_gazebo_tracking/models/agent_bot/model.sdf`
  - Turtlebot-like agent model with camera, lidar, and planar-move plugin.
- `ros2_ws/src/mbam_gazebo_tracking/models/target_bot/model.config`
- `ros2_ws/src/mbam_gazebo_tracking/models/target_bot/model.sdf`
  - Red quadruped-style target model for visually distinct tracked entities.
- `ros2_ws/src/mbam_gazebo_tracking/rviz/mbam_tracking.rviz`
  - Updated default RViz layout to camera-only:
    - Two large image panels for per-agent POV (`/agent_0/camera/image_raw`, `/agent_1/camera/image_raw`)
    - Other visual layers removed from the default file to keep focus on robot cameras
- `ros2_ws/src/mbam_gazebo_tracking/worlds/mbam_tracking.world`
  - Simplified to an obstacle-free environment with:
    - `sun`
    - custom `green_ground` plane (collision + visual)
  - Added decorative tree visuals that intentionally have no collision geometry.

## Runtime assets

- `ros2_ws/src/mbam_gazebo_tracking/config/params_compare.yaml`
  - Copied test configuration for ROS runner.
- `ros2_ws/src/mbam_gazebo_tracking/config/limo_edge.params.yaml`
  - Default parameter template for a LIMO-side observer node.
- `ros2_ws/src/mbam_gazebo_tracking/config/limo_safety.params.yaml`
  - Default parameter template for a LIMO-side safety controller node.
- `ros2_ws/src/mbam_gazebo_tracking/config/limo_central.params.yaml`
  - Default parameter template for the central coordinator node.
  - These real-world configs now default to UDP for Humble/Foxy compatibility.
- `ros2_ws/src/mbam_gazebo_tracking/checkpoints/best_model_seed42.pth`
  - Copied checkpoint used by default launch.

## Run script

- `ros2_ws/src/mbam_gazebo_tracking/scripts/run_model_based_testing_gazebo.sh`
  - One-command script:
    - Source ROS.
    - Build package with `colcon`.
    - Source workspace install.
    - Launch Gazebo + RViz + runner.

## Scope guard

- No existing source file under the original non-ROS package directories was modified.
- All new implementation is contained in the standalone ROS 2 package tree under `ros2_ws/src/mbam_gazebo_tracking`, except for a small initialization helper added to `model_based_agent_att_ros.py` inside that same package.

## Post-test fixes

- `launch/run_mbam_gazebo.launch.py`
  - Renamed `params_file` launch argument to `mbam_params_file` to avoid collision with Gazebo launch internals.
  - This prevents passing non-ROS YAML into `gzserver` as a ROS params file.
  - Added optional model-database launch args:
    - `agent_model_database`
    - `target_model_database`
    - `strict_database_models`
  - Launch now falls back to local SDF model files when those args are empty.
  - Launch now verifies database-model availability locally and falls back to bundled SDF models when unavailable (unless strict mode is enabled).
  - Entity spawns are chained sequentially (on process exit) with extended spawn timeout to avoid factory-service contention and insertion queue timeouts.
- `scripts/run_model_based_testing_gazebo.sh`
  - Wrapped ROS setup sourcing with temporary `set +u` to avoid unbound-variable failures.
  - Added local log directory setup (`ROS_LOG_DIR`) when unset.
  - Added automatic `GAZEBO_MASTER_URI` selection fallback to reduce default-port conflicts.
  - Added busy-port recovery for pre-set `GAZEBO_MASTER_URI`: if the configured port is already in use, the script auto-selects a free port.
  - Added automatic `ROS_DOMAIN_ID` assignment fallback when unset, isolating runs from stale ROS graphs that can interfere with `/spawn_entity` and `/gazebo/model_states`.
  - Added workspace-local `GAZEBO_HOME` fallback so Gazebo runtime directories are writable even when `~/.gazebo` is restricted.
  - Added default `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` to reduce shared-memory transport permission failures in restricted/containerized setups.
- `worlds/mbam_tracking.world`
  - Added `libgazebo_ros_state.so` plugin so state services (notably `/gazebo/set_entity_state`) are available to the runner.
  - Added multiple visual-only tree models (no collision geometry) for scene realism without introducing physical obstacles.
- `models/agent_bot/model.sdf`
  - Replaced heading `cone` visual with a `box` visual to avoid Gazebo factory geometry-load errors.
- `mbam_gazebo_tracking/nodes/episode_runner_node.py`
  - Detached policy action tensors before converting to scalar values for `Twist` messages, removing runtime PyTorch warnings.
  - Added sensor-fusion debug counters/logs (`pairs_*`, camera column hits, finite lidar counts) to isolate visibility drop points.
  - Updated camera segmentation from orange HSV range to red HSV ranges (with hue wraparound handling).
  - Readiness gate now requires:
    - target models visible in `/gazebo/model_states`
    - per-agent odometry streams ready
    - instead of requiring every agent name in `/gazebo/model_states`.
  - Added restricted lidar marker visualization controls:
    - `publish_lidar_markers` (default `false`)
    - `lidar_marker_stride` (default `6`)
    - `restrict_lidar_visualization` (default `true`)
    - `lidar_visualization_margin_rad` (default `0.05`)
  - When lidar markers are enabled, `/mbam/markers` scan points can be clipped to each robot camera HFOV (plus margin).
  - Added pre-collision command safety:
    - `enable_collision_pause` (default `true`)
    - `collision_lookahead_sec` (default `1.0`)
    - `collision_robot_radius` (default `0.28`)
    - `collision_target_radius` (default `0.30`)
    - `collision_safety_margin` (default `0.10`)
  - Robot commands are set to zero for that tick if predicted motion would collide with another robot or an active target.
  - Robot-robot collision detection upgraded from endpoint-only checks to continuous-time minimum-distance prediction over the lookahead horizon.
  - Updated marker publishing:
    - Marker frame is configurable (`marker_frame`, default `map`).
    - Marker clear now uses explicit episode-start `DELETEALL` instead of per-tick `DELETEALL`.
    - Publishes per-agent lidar point markers (world/map coordinates) into `/mbam/markers` for RViz visibility even without sensor TF frames.
- `models/agent_bot/model.sdf`
  - Tuned lidar placement for stable intersections with target geometry while reducing self-hits from the robot body.
  - Reworked visuals to a more realistic turtlebot-like mobile base while preserving policy-driven planar control and sensor topics.
  - Disabled Gazebo lidar ray visualization (`<visualize>false</visualize>`) to avoid overwhelming on-screen lidar clutter.
- `models/target_bot/model.sdf`
  - Reworked visuals to a red quadruped-style target robot with lidar-visible collision body.
- `launch/run_mbam_gazebo.launch.py`
  - Enabled `use_sim_time` for runner and RViz.
  - Added static transform publisher `world -> map` so RViz fixed-frame rendering is immediately available.
  - Added launch arguments and node parameter wiring for:
    - lidar marker publishing/restriction controls
    - collision pause controls
