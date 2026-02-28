#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -d /opt/ros/humble ]]; then
    export ROS_DISTRO=humble
  elif [[ -d /opt/ros/iron ]]; then
    export ROS_DISTRO=iron
  elif [[ -d /opt/ros/jazzy ]]; then
    export ROS_DISTRO=jazzy
  else
    echo "Unable to detect ROS 2 distro under /opt/ros. Set ROS_DISTRO manually."
    exit 1
  fi
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 setup file not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "${WS_DIR}"
colcon build --symlink-install --packages-select mbam_gazebo_tracking
set +u
source "${WS_DIR}/install/setup.bash"
set -u

if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${WS_DIR}/.ros_log"
fi
mkdir -p "${ROS_LOG_DIR}"

# Avoid "Address already in use" from stale/default Gazebo master port.
# Keep user override if GAZEBO_MASTER_URI is already provided.
if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then
  base_port=12000
  offset=$(( (BASHPID % 2000) + (RANDOM % 100) ))
  export GAZEBO_MASTER_URI="http://127.0.0.1:$((base_port + offset))"
fi

ros2 launch mbam_gazebo_tracking run_mbam_gazebo.launch.py "$@"
