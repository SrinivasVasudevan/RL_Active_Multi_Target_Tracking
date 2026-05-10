# Central controller (ROS 2 Humble)

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mbam_central_controller central_controller.launch.py
```

Override checkpoint or params, for example:

```bash
ros2 launch mbam_central_controller central_controller.launch.py \
  model_path:=/path/to/best_model_seed42.pth \
  params_file:=$(ros2 pkg prefix mbam_central_controller)/share/mbam_central_controller/config/params_compare.yaml
```

## Key parameters (`central_runner`)

- **SLAM / poses:** `agent_odom_topic_template`, `target_odom_topic_template` — use your LIMO SLAM pose topics (must be in the same frame as `marker_frame` used for fusion, typically `odom` or `map`).
- **Targets:** `fixed_num_landmarks`, `fixed_horizon`, `use_random_episode_schedule` — for hardware, keep `use_random_episode_schedule:=false` so the number of physical targets matches `fixed_num_landmarks`.
- **Velocity broadcast:** `publish_target_velocities`, `velocity_stamp_frame` — set `velocity_stamp_frame` to the frame in which `vector.x` / `vector.y` are expressed (default `odom` to match `target_velocity_follower`).
- **Agents / sensors:** `agent_scan_topic_template`, `agent_camera_topic_template`, etc., if your drivers use different names than `agent_{id}/scan`.
- **Green sticky filtering:** `exclude_agent_green_sticky`, `green_hsv_*`, `orange_hsv_*`.

## Dependencies

Python: `torch`, `numpy`, `opencv-python`, `pyyaml` (install via pip if not pulled in by your environment).
