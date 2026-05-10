# LIMO Controller Network Test

This repository now has two separate ROS 2 workspaces for the first local-network communication test:

- `controller_ros2_ws`: central controller computer.
- `limo_ros2_ws`: LIMO-side node to copy or build on each LIMO robot.

For the controller-to-LIMO velocity command loop, see `docs/limo_central_policy_loop.md`.

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

If the controller stays on `Waiting for UDP multicast datagram...`, multicast is not reaching the controller. Common causes are that the send command was not run from the LIMO, the machines are on different subnets, the Wi-Fi access point blocks multicast/client-to-client traffic, or the firewall is blocking UDP multicast.

First confirm both directions:

Controller sends, LIMO receives:

```bash
# Controller
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast send
```

```bash
# LIMO
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast receive
```

LIMO sends, controller receives:

```bash
# Controller
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast receive
```

```bash
# LIMO
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast send
```

## Fallback: Fast DDS Discovery Server

Use this when ping works but multicast discovery is blocked. The central controller runs a unicast discovery server, and both Foxy LIMOs point discovery at it.

Only the controller machine needs the `fastdds` command-line tool. The LIMO robots do not need `fastdds-tools`; they only need Foxy with `rmw_fastrtps_cpp`, which is normally present in standard ROS 2 Foxy installs.

Pick the controller LAN IP:

```bash
hostname -I
```

Start the discovery server on the controller:

```bash
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 -l <controller_ip> -p 11811
```

Equivalent long option form:

```bash
fastdds discovery --server-id 0 --udp-address <controller_ip> --udp-port 11811
```

You can also bind the discovery server to all controller interfaces:

```bash
fastdds discovery -i 0
```

In that case, the LIMOs still use `ROS_DISCOVERY_SERVER=<controller_ip>:11811`, where `<controller_ip>` is the controller IP address reachable from the LIMO robots.

If `fastdds` is not installed:

```bash
sudo apt update
sudo apt install ros-humble-rmw-fastrtps-cpp fastdds-tools
```

If `fastdds discovery` prints `fast-discovery-server tool not found`, the `fastdds` wrapper exists but the discovery server executable is missing. Install `fastdds-tools` on the controller:

```bash
sudo apt update
sudo apt install fastdds-tools
```

Do not run this Humble install command on the Foxy LIMO robots.

In a second controller terminal, launch the controller node:

```bash
cd controller_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<controller_ip>:11811
export ROS_LOG_DIR=$PWD/log/network_test
ros2 daemon stop
ros2 launch central_controller central_controller.launch.py robot_ids:=limo_1,limo_2
```

For a central controller that should inspect and subscribe to all robot topics, it can be useful to run the controller and ROS CLI as a Fast DDS super client. Create a super-client XML profile using the server GUID prefix printed by `fastdds discovery`. For server ID `0`, the prefix is usually:

```text
44.53.00.5f.45.50.52.4f.53.49.4d.41
```

Then launch with:

```bash
cd controller_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_super_client_10_103_218_1.xml
export ROS_LOG_DIR=$PWD/log/network_test
ros2 daemon stop
ros2 launch central_controller central_controller.launch.py robot_ids:=limo_1,limo_2
```

On each LIMO, launch its node with the same discovery server:

```bash
cd limo_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<controller_ip>:11811
export ROS_LOG_DIR=$PWD/log/network_test
ros2 daemon stop
ros2 launch limo_observation_client limo_observation.launch.py robot_id:=limo_1
```

Use `robot_id:=limo_2` on the second LIMO.

If a LIMO does not have the Fast DDS RMW package installed, install the Foxy package on that LIMO:

```bash
sudo apt update
sudo apt install ros-foxy-rmw-fastrtps-cpp
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

On any sourced ROS 2 terminal with the same `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `ROS_DISCOVERY_SERVER` settings:

```bash
ros2 daemon stop
ros2 topic list
ros2 node list
ros2 topic echo /limo_1/local_observation
ros2 topic echo /limo_1/observation_ack
ros2 topic echo /limo_2/local_observation
ros2 topic echo /limo_2/observation_ack
```

If `ros2 topic list` only shows `/parameter_events` and `/rosout`, the terminal is not discovering the test nodes. Check these in order:

1. The controller launch terminal prints `Central controller ready for robots: limo_1, limo_2`.
2. Each LIMO launch terminal prints `Published local observation seq=...`.
3. The terminal running `ros2 topic list` has the same discovery environment as the launched nodes:

```bash
printenv ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER
```

4. Stop the ROS 2 daemon after changing discovery environment:

```bash
ros2 daemon stop
ros2 node list
ros2 topic list
```

5. If using the Fast DDS Discovery Server fallback, make sure every controller/LIMO terminal has:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<controller_ip>:11811
```

6. If using multicast discovery instead, make sure `ROS_DISCOVERY_SERVER` is not set:

```bash
unset ROS_DISCOVERY_SERVER
```

When using a Fast DDS super-client XML profile for ROS CLI:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_super_client_10_103_218_1.xml
ros2 daemon stop
ros2 topic list --no-daemon
ros2 topic info --no-daemon -v /limo_1/local_observation
```

If the topic exists but `Publisher count` is `0`, the controller is seeing its own subscription but not the LIMO publisher.

## Optional Firewall Commands

Only use these if `ping` works but ROS 2 multicast/topic discovery does not.
Replace `<lan_subnet>` with your LAN subnet, for example `192.168.1.0/24`.

Fast DDS discovery and ROS 2 data transport require UDP from the LIMO robots to the controller. A quick raw UDP check is:

Controller terminal:

```bash
nc -u -l 12001
```

LIMO terminal:

```bash
printf LIMO_UDP_TEST | nc -u -w1 <controller_ip> 12001
```

If the controller does not print `LIMO_UDP_TEST`, fix the firewall/router before continuing ROS tests.

Ubuntu `ufw`:

```bash
sudo ufw allow from <lan_subnet> to any proto udp port 7400:25000
sudo ufw allow from <lan_subnet> to any proto udp port 11811
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
