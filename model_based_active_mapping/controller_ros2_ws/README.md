# Controller ROS 2 Workspace

This workspace contains the central controller node for the first multi-LIMO network test.

## Build

```bash
cd controller_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Run on the central controller computer

Use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on the Humble controller and both Foxy LIMO robots.

```bash
cd controller_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=$PWD/log/network_test
ros2 launch central_controller central_controller.launch.py robot_ids:=limo_1,limo_2
```

The node subscribes to:

- `/limo_1/local_observation`
- `/limo_2/local_observation`

It publishes targeted acknowledgments to:

- `/limo_1/observation_ack`
- `/limo_2/observation_ack`
