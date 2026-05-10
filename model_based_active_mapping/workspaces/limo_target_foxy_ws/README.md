# LIMO target stack (ROS 2 Foxy)

Targets follow planar velocities commanded by the central controller and run the same style of safety gating as agents.

## Run follower + safety

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch mbam_limo_target limo_target_stack.launch.py \
  desired_vel_topic:=mbam/target_0/desired_world_velocity \
  odom_topic:=odom \
  scan_topic:=scan
```

Use a **unique** `desired_vel_topic` per target (`target_1`, `target_2`, …). If two nodes run on one machine, give them distinct ROS node namespaces or run on separate machines.

## Frames

`Vector3Stamped.vector.x` and `.y` are interpreted as velocity in the **odom** frame (2D). Match the controller parameter `velocity_stamp_frame:=odom` and ensure SLAM odometry is published on `odom_topic`.

## Tuning

Reduce `max_linear_x` / `max_angular_z` in both `target_velocity_follower` and `cmd_vel_safety` for slow, predictable target motion.
