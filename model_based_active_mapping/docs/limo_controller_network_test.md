# LIMO Controller Network Test

This repository now has two separate ROS 2 workspaces for the first local-network communication test:

- `controller_ros2_ws`: central controller computer.
- `limo_ros2_ws`: LIMO-side node to copy or build on each LIMO robot.

The controller machine uses ROS 2 Humble. The LIMO robots use ROS 2 Foxy.
The test uses `std_msgs/String` JSON payloads so it does not require shared custom messages during the first connectivity check. This keeps the first mixed-distro test focused on DDS discovery, topic transport, robot IDs, and targeted acknowledgments.

## Network Assumptions

All three machines must be on the same local network:

- Central controller computer.
- LIMO robot 1.
- LIMO robot 2.

Use the same ROS domain on all machines:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`rmw_fastrtps_cpp` is recommended for the first test because Foxy and Humble both commonly have it installed. If your machines already use Cyclone DDS, use `rmw_cyclonedds_cpp` on all three machines instead. Do not mix RMW implementations for this first test.

If discovery does not work, check that the machines can ping each other, multicast is enabled on the Wi-Fi/router, and firewall rules allow DDS/RTPS UDP traffic.

## Build

On the central controller computer:

```bash
cd controller_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

On each LIMO:

```bash
cd limo_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Preflight Checks

Run these on each machine to confirm the intended distro and RMW:

Controller:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 --version
printenv ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
```

Each LIMO:

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 --version
printenv ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
```

Check basic network reachability:

```bash
ping <controller_ip>
ping <limo_1_ip>
ping <limo_2_ip>
```

Check ROS 2 multicast discovery before running the nodes:

Controller terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast receive
```

LIMO terminal:

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast send
```

## Run

Terminal on central controller:

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

Terminal on LIMO 1:

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

Terminal on LIMO 2:

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

## Expected Behavior

Each LIMO publishes JSON observations to its own topic:

- `/limo_1/local_observation`
- `/limo_2/local_observation`

The central controller subscribes to those topics and replies on the matching robot-specific ack topic:

- `/limo_1/observation_ack`
- `/limo_2/observation_ack`

Each LIMO logs acknowledgments only from its own ack topic.

## Quick Verification Commands

On any sourced ROS 2 terminal with the same `ROS_DOMAIN_ID`:

```bash
ros2 topic list
ros2 topic echo /limo_1/local_observation
ros2 topic echo /limo_1/observation_ack
ros2 topic echo /limo_2/local_observation
ros2 topic echo /limo_2/observation_ack
```

## Optional Firewall Commands

Only use these if `ping` works but ROS 2 multicast/topic discovery does not.
Replace `<lan_subnet>` with your LAN subnet, for example `192.168.1.0/24`.

Ubuntu `ufw`:

```bash
sudo ufw allow from <lan_subnet> to any proto udp port 7400:25000
sudo ufw reload
sudo ufw status numbered
```

If you want a temporary firewall check instead:

```bash
sudo ufw status
sudo ufw disable
```

Re-enable it after testing:

```bash
sudo ufw enable
```

## Next Integration Point

The placeholder observation lives in:

```text
limo_ros2_ws/src/limo_observation_client/limo_observation_client/nodes/limo_observation_node.py
```

Replace `_read_local_observation()` with real LIMO local state, odometry, lidar, camera, or fused observation data when the communication smoke test is passing.
