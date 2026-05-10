# Distributed MBAM workspaces

Three separate colcon workspaces support the Alienware **ROS 2 Humble** central controller and **ROS 2 Foxy** LIMO computers.

| Workspace | ROS distro | Machine | Package(s) |
|-----------|------------|---------|------------|
| `central_controller_humble_ws` | Humble | Alienware laptop | `mbam_central_controller` — policy, fusion, `cmd_vel` to agents, desired world velocity to targets |
| `limo_agent_foxy_ws` | Foxy | Agent LIMOs | `mbam_limo_agent` — `cmd_vel_safety` (limits + lidar stop) |
| `limo_target_foxy_ws` | Foxy | Target LIMOs | `mbam_limo_target` — `target_velocity_follower` + `cmd_vel_safety` |

Reference implementation for training/simulation remains under `ros2_ws/src/mbam_gazebo_tracking`.

---

## Copy-paste: ROS 2 environment (run in every terminal)

Pick a domain ID (here `42`) and use the **same** number on **all** machines.

**Alienware (ROS 2 Humble):**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
# Optional: persist for the session
# echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
```

**Each LIMO (ROS 2 Foxy):**

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
```

Install Cyclone RMW on **both** distros if missing:

```bash
# On Humble machine:
sudo apt update && sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# On each Foxy machine:
sudo apt update && sudo apt install -y ros-foxy-rmw-cyclonedds-cpp
```

---

## Networking (explicit setup — do this first)

ROS 2 discovery runs over the **LAN**; the controller and LIMOs must share one DDS domain.

### Physical and IP layer

1. Same L2 segment (same VLAN / Wi‑Fi without client isolation). Prefer **Ethernet** for first tests.
2. Static IPs or DHCP reservations (example):

   | Host | Example IP |
   |------|------------|
   | Alienware | `192.168.1.10` |
   | Agent LIMO 0 | `192.168.1.11` |
   | Agent LIMO 1 | `192.168.1.12` |
   | Target LIMO(s) | `192.168.1.20`, … |

3. From the Alienware, test reachability (replace IPs):

   ```bash
   ping -c 5 192.168.1.11
   ping -c 5 192.168.1.12
   ping -c 5 192.168.1.20
   ```

4. Optional time sync (on Ubuntu, chrony is typical):

   ```bash
   sudo apt install -y chrony
   chronyc tracking   # verify synchronized
   ```

### Cyclone DDS: force interface or unicast peers

List interfaces:

```bash
ip -br addr
```

If multicast is blocked (common on some Wi‑Fi), set **unicast peers** on **every** machine. Replace `enp3s0` with your interface and list **all** other lab hosts (include the Alienware IP on robots, and all robot IPs on the Alienware).

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enp3s0</NetworkInterfaceAddress></General><Discovery><Peers><Peer address="192.168.1.10"/><Peer address="192.168.1.11"/><Peer address="192.168.1.12"/><Peer address="192.168.1.20"/></Peers></Discovery></Domain></CycloneDDS>'
```

To avoid retyping, save once per machine:

```bash
echo 'export CYCLONEDDS_URI='"'"'<CycloneDDS><Domain><General><NetworkInterfaceAddress>enp3s0</NetworkInterfaceAddress></General><Discovery><Peers><Peer address="192.168.8.227"/><Peer address="192.168.8.126"/><Peer address="192.168.8.211"/></Peers></Discovery></Domain></CycloneDDS>'"'"'' >> ~/.bashrc_mbam_lab
# Then: source ~/.bashrc_mbam_lab
```

After changing `CYCLONEDDS_URI`, restart all ROS nodes.

### Firewall (Ubuntu `ufw` example)

For initial bring-up only (tighten later):

```bash
sudo ufw status
sudo ufw disable   # temporary; re-enable with proper rules after DDS works
```

### Verify `ROS_LOCALHOST_ONLY` is off

```bash
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
# Must be empty or 0. If set to 1:
unset ROS_LOCALHOST_ONLY
```

### Message compatibility (Humble ↔ Foxy)

Standard messages (`geometry_msgs/Twist`, `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `sensor_msgs/Image`, `geometry_msgs/Vector3Stamped`) are compatible. Fix remaps before assuming a bridge is needed.

---

## Phase 0 — Build workspaces and checkpoint (one time per machine)

### 0.1 Controller (Alienware, Humble)

```bash
cd /path/to/model_based_active_mapping/workspaces/central_controller_humble_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Confirm the checkpoint exists after build (install share):

```bash
ros2 pkg prefix mbam_central_controller
ls "$(ros2 pkg prefix mbam_central_controller)/share/mbam_central_controller/checkpoints/"
```

If `best_model_seed42.pth` is missing, copy it from the Gazebo package source:

```bash
cp /path/to/model_based_active_mapping/ros2_ws/src/mbam_gazebo_tracking/checkpoints/best_model_seed42.pth \
   /path/to/model_based_active_mapping/workspaces/central_controller_humble_ws/src/mbam_central_controller/checkpoints/
cd /path/to/model_based_active_mapping/workspaces/central_controller_humble_ws
colcon build --symlink-install --packages-select mbam_central_controller
source install/setup.bash
```

### 0.2 Agent LIMOs (Foxy)

On **each** agent robot:

```bash
cd /path/to/model_based_active_mapping/workspaces/limo_agent_foxy_ws
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

### 0.3 Target LIMOs (Foxy)

On **each** target robot:

```bash
cd /path/to/model_based_active_mapping/workspaces/limo_target_foxy_ws
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

---

## What must be running before you test from the controller

Before `ros2 topic list` on the Alienware can show robot topics, **each LIMO** must publish its drivers (laser, odom, camera, base). The exact launch file names depend on your **AgileX / LIMO** software stack; replace the placeholder below with the commands from the vendor manual.

### On each agent LIMO (Foxy) — start in order

1. **LIMO base / motor driver / CAN** (vendor launch). Example placeholder only:

   ```bash
   source /opt/ros/foxy/setup.bash
   source /path/to/limo_agent_foxy_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   unset ROS_LOCALHOST_ONLY
   # ros2 launch <your_limo_stack> <base_driver>.launch.py
   ```

2. **SLAM or localization** so the controller receives pose on the topic you will configure (e.g. `agent_0/odom`). Example placeholder:

   ```bash
   # ros2 launch <your_slam_package> <slam>.launch.py
   ```

3. **Topic names:** either:
   - **Preferred:** configure the central controller’s parameters (`agent_odom_topic_template`, `agent_scan_topic_template`, …) to match the vendor topics, **or**
   - add a small relay/remap launch that republishes vendor topics to `agent_{id}/odom`, `agent_{id}/scan`, etc.

4. **MBAM safety** (subscribes network `cmd_vel`, outputs to real base `cmd_vel`):

   ```bash
   source /opt/ros/foxy/setup.bash
   source /path/to/limo_agent_foxy_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   unset ROS_LOCALHOST_ONLY
   ros2 launch mbam_limo_agent limo_agent_safety.launch.py \
     cmd_vel_in:=/agent_0/cmd_vel \
     cmd_vel_out:=/cmd_vel \
     scan_topic:=/scan \
     max_linear_x:=0.15 \
     max_angular_z:=0.5 \
     min_range_m:=0.5
   ```

   On the **second** agent, use `cmd_vel_in:=/agent_1/cmd_vel` (and ensure SLAM/camera/scan topics match the controller templates for robot `1`).

### On each target LIMO (Foxy)

1. **LIMO base** (vendor launch) — same idea as agents.
2. **SLAM** publishing a world-fixed or `odom` pose. Republish or remap to `target_<id>/odom` (must match `target_odom_topic_template` on the controller, default `target_{id}/odom`).
3. **MBAM target stack** (velocity follower + safety):

   ```bash
   source /opt/ros/foxy/setup.bash
   source /path/to/limo_target_foxy_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   unset ROS_LOCALHOST_ONLY
   ros2 launch mbam_limo_target limo_target_stack.launch.py \
     odom_topic:=/odom \
     scan_topic:=/scan \
     desired_vel_topic:=/mbam/target_0/desired_world_velocity
   ```

   For the second physical target, use `desired_vel_topic:=/mbam/target_1/desired_world_velocity`, and ensure its SLAM is remapped to `/target_1/odom`, etc.

### On the Alienware (Humble) — before integration test only

Do **not** start `central_runner` until Steps 1–4 pass. For discovery checks you only need the environment:

```bash
source /opt/ros/humble/setup.bash
source /path/to/central_controller_humble_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
```

---

## Step 1 — Networking of controller and robots

**Goal:** DDS sees remote topics; ping is not enough.

| Step | Command(s) | Pass criterion |
|------|------------|----------------|
| 1.1 | From Alienware: `ping -c 5 <robot_ip>` for each robot | 0% packet loss |
| 1.2 | On **every** machine, in the same terminal you use for ROS: `echo $ROS_DOMAIN_ID` | Same value (e.g. `42`) on all |
| 1.3 | On every machine: `echo ${RMW_IMPLEMENTATION}` | `rmw_cyclonedds_cpp` |
| 1.4 | On every machine: `echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"` | Empty or `0` |
| 1.5 | On **one** LIMO (with vendor drivers running): `source /opt/ros/foxy/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && unset ROS_LOCALHOST_ONLY && ros2 topic list` | You see lidar/odom/camera topics |
| 1.6 | On **Alienware** (robots still running, **no** extra nodes needed on laptop): `source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && unset ROS_LOCALHOST_ONLY && ros2 topic list` | Same topic **names** appear as on the robot (or your documented global names) |
| 1.7 | If 1.6 fails but 1.1 passes: set `CYCLONEDDS_URI` (see above) on **all** hosts, restart robots’ launches, repeat 1.6 | Topics visible remotely |
| 1.8 | Optional refresh ROS 2 daemon on laptop: `ros2 daemon stop && ros2 daemon start` | Stale graph cleared |

**Failure modes:** only localhost topics on laptop → `ROS_LOCALHOST_ONLY=1`, wrong interface in `CYCLONEDDS_URI`, or firewall; topic list empty everywhere on robot → drivers not launched.

---

## Step 2 — Topics present and observations visible

**Prerequisite:** complete **“What must be running”** for all agents and targets so scans, cameras, and odometry exist.

**Goal:** Contract topics exist and publish at reasonable rates. Replace topic names if you use vendor defaults; examples assume `agent_0/...` and `target_0/odom`.

| Step | Command(s) | Pass criterion |
|------|------------|----------------|
| 2.1 | `ros2 topic list \| grep -E 'agent_|target_|mbam|scan|odom|camera'` | Expected names present |
| 2.2 | `ros2 topic list --types \| grep agent_0` | Types include `nav_msgs/msg/Odometry`, `sensor_msgs/msg/LaserScan`, etc. |
| 2.3 | `ros2 topic hz /agent_0/odom` | Stable rate (typ. ≥ 10 Hz for odom) |
| 2.4 | `ros2 topic hz /agent_0/scan` | Stable rate |
| 2.5 | `ros2 topic hz /agent_0/camera/image_raw` | Stable rate if fusion uses camera |
| 2.6 | `ros2 topic echo /agent_0/camera/image_raw --once` | `encoding` field set (e.g. `bgr8`) |
| 2.7 | `ros2 topic hz /target_0/odom` (repeat per target) | Stable rate |
| 2.8 | `ros2 topic echo /agent_0/scan --once` | `ranges` has finite values in front sector |
| 2.9 | `ros2 topic echo /agent_0/odom --once` | Position/orientation finite |

If topics use different names, either remaps on the robots or pass overrides to `central_runner` (see Phase 5).

---

## Step 3 — Velocity commands over the network

**Safety:** clear space, low speeds, estop ready, use **lifted wheels** for first try if possible.

**Prerequisite:** `mbam_limo_agent` safety running on each agent with `cmd_vel_in` matching the topic you publish from the laptop (see **What must be running**).

| Step | Command(s) | Pass criterion |
|------|------------|----------------|
| 3.1 | On Alienware: `ros2 topic pub /agent_0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10` | Robot 0 moves slowly forward ~1 s then stop with Ctrl+C |
| 3.2 | `ros2 topic pub /agent_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10` | Robot 1 responds |
| 3.3 | Optional rotation test: `ros2 topic pub /agent_0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" --rate 10` | Yaw rate bounded by safety limits |
| 3.4 | **Target:** with `limo_target_stack` running, on Alienware: `ros2 topic pub /mbam/target_0/desired_world_velocity geometry_msgs/msg/Vector3Stamped "{header: {frame_id: 'odom'}, vector: {x: 0.03, y: 0.0, z: 0.0}}" --rate 10` | Target creeps; stops when pub stops (and follower timeout applies) |
| 3.5 | Stress (still small): alternate `linear.x` between `0.03` and `0.0` every 2 s | Motion bounded; lidar stop still works when obstacle placed ahead |

**Failure modes:** no motion → wrong `cmd_vel_in`/`cmd_vel_out` or safety sees obstacle; spin only → differential-drive sign convention.

---

## Step 4 — Desirable ROS state before `central_runner`

Run these on the **Alienware** with all robots up and MBAM safety/target stacks running.

| Check | Command(s) | Desirable outcome |
|-------|------------|-------------------|
| Graph | `ros2 topic list` | All configured agent + target + `mbam` topics visible |
| Rates | `ros2 topic hz /agent_0/odom`, `ros2 topic hz /agent_0/scan`, … | No large dropouts |
| No duplicate cmd | `ros2 topic info /cmd_vel --verbose` | Typically **one** publisher (safety → base); controller publishes to `/agent_i/cmd_vel`, not `/cmd_vel` |
| Controller params | (no single command) | `fixed_num_landmarks` = number of targets; `velocity_stamp_frame` matches velocity interpretation (often `odom`) |
| TF noise | `ros2 topic echo /tf --once` (optional) | Understand parent/child frames before tuning fusion |

---

## Phase 5 — Integrated MBAM test (`central_runner`)

**Prerequisite:** Steps 1–4 pass.

1. Keep **all** LIMO vendor + SLAM + MBAM nodes running as in **What must be running**.
2. On **Alienware**:

   ```bash
   source /opt/ros/humble/setup.bash
   source /path/to/central_controller_humble_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   unset ROS_LOCALHOST_ONLY
   ros2 launch mbam_central_controller central_controller.launch.py \
     model_path:=$(ros2 pkg prefix mbam_central_controller)/share/mbam_central_controller/checkpoints/best_model_seed42.pth
   ```

3. If your topics differ from defaults, pass parameters instead of editing code, for example:

   ```bash
   ros2 run mbam_central_controller central_runner --ros-args \
     -p agent_odom_topic_template:="agent_{id}/odom" \
     -p agent_scan_topic_template:="agent_{id}/scan" \
     -p target_odom_topic_template:="target_{id}/odom" \
     -p fixed_num_landmarks:=5 \
     -p velocity_stamp_frame:=odom \
     -p marker_frame:=odom
   ```

4. Watch logs for “All agents and targets reporting pose” and episode progress.

---

## Build (quick reference)

**Controller (Humble):**

```bash
cd central_controller_humble_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y
colcon build --symlink-install
source install/setup.bash
```

**Agents / targets (Foxy):**

```bash
cd limo_agent_foxy_ws   # or limo_target_foxy_ws
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src -y
colcon build --symlink-install
source install/setup.bash
```

---

## Topic contract (summary)

- **Agents:** controller subscribes to `agent_{id}/odom`, `agent_{id}/scan`, `agent_{id}/camera/...` (templates are parameters) and publishes `agent_{id}/cmd_vel`. On each LIMO, `mbam_limo_agent` should subscribe to that network topic and publish to the real base `cmd_vel`.
- **Targets:** controller publishes `mbam/target_{id}/desired_world_velocity` (`Vector3Stamped`). Each target runs `mbam_limo_target` follower + safety. Targets must publish SLAM odometry on `target_{id}/odom` (template).

See per-workspace README files under `central_controller_humble_ws/`, `limo_agent_foxy_ws/`, and `limo_target_foxy_ws/` for extra tuning notes.
