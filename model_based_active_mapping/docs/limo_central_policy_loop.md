# LIMO Central Policy Loop

This is the real-robot version of the network smoke test.

## Architecture

- Each LIMO runs `limo_observation_client`.
- The controller runs `central_controller`.
- LIMOs publish JSON observations:
  - `/limo_1/local_observation`
  - `/limo_2/local_observation`
- The controller publishes desired velocity commands:
  - `/limo_1/controller_cmd_vel`
  - `/limo_2/controller_cmd_vel`
- Each LIMO safety node publishes the final robot command to:
  - `/cmd_vel`

The central controller is allowed to plan, but it does not directly drive `/cmd_vel`.
The robot-local LIMO node is the final safety gate.

## LIMO Safety

The LIMO node enforces:

- command timeout stop;
- maximum linear speed;
- maximum angular speed;
- global test speed scaling;
- front obstacle stop;
- front obstacle slow-down zone;
- rear obstacle stop for reverse commands;
- side obstacle turn limiting;
- optional scan-required-before-motion behavior.

Default test limits are intentionally slow:

```text
test_speed_scale: 0.35
max_linear_mps: 0.18
max_angular_radps: 0.55
front_stop_distance_m: 0.55
slow_distance_m: 1.10
```

## Launch

Controller:

```bash
cd controller_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_super_client_10_103_218_1.xml
ros2 launch central_controller central_controller.launch.py robot_ids:=limo_1,limo_2
```

LIMO:

```bash
cd ~/limo_ros2_ws_network_test
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=10.103.218.1:11811
ros2 launch limo_observation_client limo_observation.launch.py robot_id:=limo_1
```

Use `robot_id:=limo_2` on the second LIMO.

## Target Detection Interface

The LIMO node accepts target detections as JSON on:

```text
/<robot_id>/target_detections
```

Schema:

```json
{
  "detections": [
    {
      "label": "unitree_go2",
      "score": 0.87,
      "bearing_rad": 0.12,
      "range_m": 2.4,
      "bbox_xyxy": [220, 140, 360, 310]
    }
  ]
}
```

`bearing_rad` is enough for the current controller to turn toward the target.
`range_m` lets the controller stop before getting too close. Range can come from lidar association, depth camera, or a monocular range estimate once calibrated.

## Object Detector Choice

For Unitree Go robot dogs, do not rely on the generic COCO `dog` class. A quadruped robot can be missed or confused with an animal, especially from side/rear views.

Recommended path:

- Use a small custom YOLO detector for real-time LIMO-side inference.
- Train/fine-tune it with labels like `unitree_go1`, `unitree_go2`, or a single `robot_dog` class.
- Use Grounding DINO or a similar open-vocabulary detector offline to bootstrap labels, then curate them before training YOLO.

A detector node should publish the JSON schema above; the central controller does not need to know which model produced it.
