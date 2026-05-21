# limo_ws — Distributed Real-Robot Deployment

Multi-machine ROS2 Humble workspace for running the MBAM tracking policy on physical Limo robots.

```
Laptop                          Agent Limo (×N)             Target Limo (×M)
---------                       ---------------             ----------------
central_controller              limo_agent                  limo_target
  DRA policy + KF   ──────────▶  camera + LiDAR              OU random walk
                  cmd_vel_desired  safety filter               LiDAR safety stop
                  target_beliefs ◀─ observation
```

**Data flow:** each agent publishes `/agent_<id>/observation` → central_controller runs KF + policy → publishes `/agent_<id>/cmd_vel_desired` + `/target_beliefs` → agent safety-filters and drives hardware.

---

## Prerequisites

All machines must be on the **same LAN** and running **ROS2 Humble**.

```bash
# Same on every machine — pick any unused domain (e.g. 42)
export ROS_DOMAIN_ID=42
```

Add this to `~/.bashrc` on each machine so it persists across terminals.

The `central_controller` launch file also expects the `mbam_gazebo_tracking` package (for default params/checkpoint paths). Either install that package in the same workspace or pass `params_file` and `model_path` explicitly at launch (see below).

---

## Build

Run this **on each machine** (laptop and every Limo) from the workspace root:

```bash
cd ~/limo_ws          # or wherever you cloned/copied limo_ws
colcon build --symlink-install
source install/setup.bash
```

Only the packages each machine needs to run have to be built (e.g. a target Limo only needs `limo_target` and `mbam_interfaces`), but building everything is harmless.

---

## Starting the system

### 1. Laptop — Central Controller

```bash
source install/setup.bash

ros2 launch central_controller central_controller.launch.py \
    num_robots:=2 \
    num_targets:=3 \
    params_file:=/path/to/params_compare.yaml \
    model_path:=/path/to/best_model_seed42.pth
```

Key arguments (all have defaults matching `config/params.yaml`):

| Argument | Default | Notes |
|---|---|---|
| `num_robots` | 2 | Number of agent Limos |
| `num_targets` | 3 | Number of target Limos |
| `max_num_targets` | 7 | Must match across all nodes |
| `tau` | 1.0 | Control period [s]; must match agent `tau` |
| `max_linear_vel` | 0.15 | [m/s] policy output clip |
| `max_angular_vel` | 0.40 | [rad/s] policy output clip |
| `arena_half_size` | 5.0 | [m] initial belief spread |
| `params_file` | (from mbam_gazebo_tracking) | Path to policy YAML |
| `model_path` | (from mbam_gazebo_tracking) | Path to `.pth` checkpoint |

---

### 2. Agent Limo — one terminal per robot

SSH into each agent Limo and run:

```bash
source ~/limo_ws/install/setup.bash

# Agent 0
ros2 launch limo_agent agent.launch.py robot_id:=0

# Agent 1 (separate SSH session / separate Limo)
ros2 launch limo_agent agent.launch.py robot_id:=1
```

`robot_id` must be unique (0-indexed) and consistent with what the central controller expects.

Key parameters (edit `src/limo_agent/config/agent_params.yaml` before building):

| Parameter | Default | Notes |
|---|---|---|
| `tau` | 1.0 | Must match central_controller `tau` |
| `max_num_targets` | 7 | Must match central_controller |
| `fov_half_angle` | 1.3963 rad | ~80°; must cover policy FoV |
| `safety_stop_dist` | 0.30 m | Full stop threshold |
| `safety_slow_dist` | 0.55 m | Slowdown begins here |

---

### 3. Target Limo — one terminal per robot

SSH into each target Limo and run:

```bash
source ~/limo_ws/install/setup.bash

# Target 0
ros2 launch limo_target target.launch.py robot_id:=0

# Target 1 (separate SSH session / separate Limo)
ros2 launch limo_target target.launch.py robot_id:=1

# Target 2
ros2 launch limo_target target.launch.py robot_id:=2
```

Target Limos are **not** commanded by the central controller — they run an autonomous OU-process random walk with LiDAR safety stops.

Key parameters (edit `src/limo_target/config/target_params.yaml` before building):

| Parameter | Default | Notes |
|---|---|---|
| `tau` | 2.0 s | Target control period |
| `max_linear_vel` | 0.08 m/s | Max forward speed |
| `vel_decay` | 0.75 | Velocity persistence |
| `safety_stop_dist` | 0.30 m | Full stop threshold |
| `seed` | 42 | Set -1 for unseeded random motion |

---

## Topic map

```
/agent_0/observation        (mbam_interfaces/AgentObservation)   agent_0 → central
/agent_1/observation        (mbam_interfaces/AgentObservation)   agent_1 → central

/target_beliefs             (mbam_interfaces/TargetBeliefs)      central → all agents
/agent_0/cmd_vel_desired    (geometry_msgs/Twist)                 central → agent_0
/agent_1/cmd_vel_desired    (geometry_msgs/Twist)                 central → agent_1

/agent_0/cmd_vel            (geometry_msgs/Twist)                 agent_0 → hardware
/agent_1/cmd_vel            (geometry_msgs/Twist)                 agent_1 → hardware

/target_0/cmd_vel           (geometry_msgs/Twist)                 target_0 → hardware
/target_1/cmd_vel           (geometry_msgs/Twist)                 target_1 → hardware
/target_2/cmd_vel           (geometry_msgs/Twist)                 target_2 → hardware
```

---

## Startup order

1. Set `ROS_DOMAIN_ID` on all machines.
2. Start **central_controller** on laptop first (it must be ready to receive observations).
3. Start **target** nodes (they run independently; order doesn't matter relative to agents).
4. Start **agent** nodes last (they begin publishing observations immediately).

---

## Troubleshooting

**Nodes can't see each other:**
```bash
ros2 topic list   # should show topics from all machines if domain ID matches
ping <other-machine-ip>
```

**Agent sees no `target_beliefs`:** check `max_num_targets` matches across all nodes.

**Robot moves too fast / safety stops triggering constantly:** lower `max_linear_vel` in the central_controller launch args and verify `safety_slow_dist` in agent/target params.
