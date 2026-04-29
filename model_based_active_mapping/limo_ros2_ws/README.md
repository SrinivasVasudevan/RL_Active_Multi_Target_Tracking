# LIMO ROS 2 Workspace

This workspace contains the LIMO-side test node. Run one copy on each ROS 2 Foxy LIMO robot with a unique `robot_id`.

## Build

```bash
cd limo_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Run on LIMO 1

```bash
cd limo_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=$PWD/log/network_test
ros2 launch limo_observation_client limo_observation.launch.py robot_id:=limo_1
```

## Run on LIMO 2

```bash
cd limo_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=$PWD/log/network_test
ros2 launch limo_observation_client limo_observation.launch.py robot_id:=limo_2
```

Each LIMO publishes to `/<robot_id>/local_observation` and listens only on `/<robot_id>/observation_ack`.
