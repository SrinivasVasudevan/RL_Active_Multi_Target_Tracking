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

## Launch and simulation assets

- `ros2_ws/src/mbam_gazebo_tracking/launch/run_mbam_gazebo.launch.py`
  - Launches Gazebo world, spawns all entities, starts RViz and runner node.
- `ros2_ws/src/mbam_gazebo_tracking/worlds/mbam_tracking.world`
  - Gazebo world definition.
- `ros2_ws/src/mbam_gazebo_tracking/models/agent_bot/model.config`
- `ros2_ws/src/mbam_gazebo_tracking/models/agent_bot/model.sdf`
  - Agent model with visual identity (blue), camera, lidar, planar-move plugin.
- `ros2_ws/src/mbam_gazebo_tracking/models/target_bot/model.config`
- `ros2_ws/src/mbam_gazebo_tracking/models/target_bot/model.sdf`
  - Target model with visually distinct identity (orange sphere).
- `ros2_ws/src/mbam_gazebo_tracking/rviz/mbam_tracking.rviz`
  - Updated default RViz layout to camera-only:
    - Two large image panels for per-agent POV (`/agent_0/camera/image_raw`, `/agent_1/camera/image_raw`)
    - Other visual layers removed from the default file to keep focus on robot cameras
- `ros2_ws/src/mbam_gazebo_tracking/worlds/mbam_tracking.world`
  - Simplified to an obstacle-free environment with:
    - `sun`
    - custom `green_ground` plane (collision + visual)
  - Removed custom walls/trees/path/floor geometry that could interfere with robot motion.

## Runtime assets

- `ros2_ws/src/mbam_gazebo_tracking/config/params_compare.yaml`
  - Copied test configuration for ROS runner.
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
- All new implementation is contained in the standalone ROS 2 package tree under `ros2_ws/src/mbam_gazebo_tracking`.

## Post-test fixes

- `launch/run_mbam_gazebo.launch.py`
  - Renamed `params_file` launch argument to `mbam_params_file` to avoid collision with Gazebo launch internals.
  - This prevents passing non-ROS YAML into `gzserver` as a ROS params file.
- `scripts/run_model_based_testing_gazebo.sh`
  - Wrapped ROS setup sourcing with temporary `set +u` to avoid unbound-variable failures.
  - Added local log directory setup (`ROS_LOG_DIR`) when unset.
  - Added automatic `GAZEBO_MASTER_URI` selection fallback to reduce default-port conflicts.
- `worlds/mbam_tracking.world`
  - Added `libgazebo_ros_state.so` plugin so state services (notably `/gazebo/set_entity_state`) are available to the runner.
- `models/agent_bot/model.sdf`
  - Replaced heading `cone` visual with a `box` visual to avoid Gazebo factory geometry-load errors.
- `mbam_gazebo_tracking/nodes/episode_runner_node.py`
  - Detached policy action tensors before converting to scalar values for `Twist` messages, removing runtime PyTorch warnings.
  - Added sensor-fusion debug counters/logs (`pairs_*`, camera column hits, finite lidar counts) to isolate visibility drop points.
  - Updated marker publishing:
    - Marker frame is configurable (`marker_frame`, default `map`).
    - Marker clear now uses explicit episode-start `DELETEALL` instead of per-tick `DELETEALL`.
    - Publishes per-agent lidar point markers (world/map coordinates) into `/mbam/markers` for RViz visibility even without sensor TF frames.
- `models/agent_bot/model.sdf`
  - Adjusted lidar sensor pose to `0.20 0 0.10` so the scan plane intersects target geometry without self-hitting the robot base.
  - Root cause fixed: previous lidar placement produced either no target intersections (beam above target sphere) or constant self-return ranges.
- `launch/run_mbam_gazebo.launch.py`
  - Enabled `use_sim_time` for runner and RViz.
  - Added static transform publisher `world -> map` so RViz fixed-frame rendering is immediately available.
