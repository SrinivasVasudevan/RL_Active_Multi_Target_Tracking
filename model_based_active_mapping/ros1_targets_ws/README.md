# ros1_targets_ws

Standalone ROS 1 workspace for running physical target robots that move autonomously while enforcing local lidar-based safety.

## What this workspace contains

- `mbam_target_robot`
  - `target_motion_controller.py`
    - Generates target motion commands.
    - Supports `constant` and `wander` motion modes.
  - `target_safety_controller.py`
    - Filters motion commands with local `/scan`.
    - Slows, stops, or turns away from nearby walls and obstacles before publishing the final `/cmd_vel`.

## Workspace layout

- `src/mbam_target_robot/`
  - ROS 1 catkin package for target robot autonomy and safety.

## Requirements

- ROS 1 Noetic or another Python 3 ROS 1 environment.
- Packages:
  - `rospy`
  - `geometry_msgs`
  - `sensor_msgs`
- Python:
  - `numpy`

## Build

From the repo root:

```bash
cd ros1_targets_ws
catkin_make
source devel/setup.bash
```

## Single target robot

Run one target robot with constant velocity:

```bash
roslaunch mbam_target_robot run_target_robot.launch \
  robot_name:=target0 \
  scan_topic:=/scan \
  output_cmd_topic:=/cmd_vel \
  motion_mode:=constant \
  constant_linear_velocity:=0.20 \
  constant_angular_velocity:=0.05
```

Run one target robot with wandering motion:

```bash
roslaunch mbam_target_robot run_target_robot.launch \
  robot_name:=target1 \
  scan_topic:=/scan \
  output_cmd_topic:=/cmd_vel \
  motion_mode:=wander \
  wander_forward_velocity_min_mps:=0.12 \
  wander_forward_velocity_max_mps:=0.28 \
  reverse_probability:=0.05
```

## Two target robots

Run the same launch once per target robot with different velocity settings, or use separate machines / namespaces as appropriate for your ROS 1 setup.

Example:

```bash
roslaunch mbam_target_robot run_target_robot.launch \
  robot_name:=target0 \
  scan_topic:=/target0/scan \
  output_cmd_topic:=/target0/cmd_vel \
  motion_mode:=constant \
  constant_linear_velocity:=0.18 \
  constant_angular_velocity:=0.03
```

```bash
roslaunch mbam_target_robot run_target_robot.launch \
  robot_name:=target1 \
  scan_topic:=/target1/scan \
  output_cmd_topic:=/target1/cmd_vel \
  motion_mode:=constant \
  constant_linear_velocity:=0.24 \
  constant_angular_velocity:=-0.04
```

## Safety behavior

- Forward motion is slowed inside `forward_slowdown_distance_m` and stopped inside `forward_emergency_stop_distance_m`.
- Reverse motion has separate rear safety distances.
- Turning into a nearby wall or object is suppressed using left and right lidar sectors.
- If the robot is blocked in front, the safety controller can inject a local escape turn.
- If planner commands or scans time out, the safety controller publishes zero velocity.

## Key parameters to tune on hardware

- `forward_emergency_stop_distance_m`
- `forward_slowdown_distance_m`
- `turn_clearance_distance_m`
- `side_clearance_distance_m`
- `constant_linear_velocity`
- `constant_angular_velocity`
- `wander_forward_velocity_min_mps`
- `wander_forward_velocity_max_mps`

## Notes

- The safety controller is local to the target robot and does not depend on the ROS 2 coordinator.
- The motion controller publishes raw target motion to an internal topic, and the safety controller publishes the final safe `/cmd_vel`.
