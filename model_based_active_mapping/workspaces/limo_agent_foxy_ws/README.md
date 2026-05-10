# LIMO agent stack (ROS 2 Foxy)

Run **after** pointing the central controller at this robot’s command topic.

## Typical wiring

1. Central controller publishes to `agent_0/cmd_vel` (over DDS) or your relay topic.
2. On the LIMO, remap that subscription to `cmd_vel_raw` (or set `cmd_vel_in` below).
3. Safety node outputs to the driver’s real `cmd_vel`.

## Run safety node

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch mbam_limo_agent limo_agent_safety.launch.py \
  cmd_vel_in:=agent_0/cmd_vel \
  cmd_vel_out:=cmd_vel \
  scan_topic:=scan
```

Tune `max_linear_x`, `max_angular_z`, `min_range_m`, and `front_angle_deg` for your lab. Keep speeds low during bring-up.

## Direct run

```bash
ros2 run mbam_limo_agent cmd_vel_safety --ros-args \
  -p cmd_vel_in:=agent_0/cmd_vel \
  -p cmd_vel_out:=cmd_vel \
  -p max_linear_x:=0.15
```
