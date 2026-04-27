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
source "${WS_DIR}/setup.bash"
set -u

if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${WS_DIR}/.ros_log"
fi
mkdir -p "${ROS_LOG_DIR}"

# Some environments expose a partially restricted $HOME. Gazebo then fails when
# creating ~/.gazebo/server-<port>. Use a writable local HOME fallback only if needed.
ORIG_HOME="${HOME:-}"
home_is_writable=true
if [[ -z "${ORIG_HOME}" ]]; then
  home_is_writable=false
else
  if ! mkdir -p "${ORIG_HOME}/.gazebo" >/dev/null 2>&1; then
    home_is_writable=false
  else
    probe_file="${ORIG_HOME}/.gazebo/.mbam_write_probe.$$"
    if ! touch "${probe_file}" >/dev/null 2>&1; then
      home_is_writable=false
    else
      rm -f "${probe_file}" >/dev/null 2>&1 || true
    fi
  fi
fi

if [[ "${home_is_writable}" != "true" ]]; then
  export HOME="${WS_DIR}/.home"
  mkdir -p "${HOME}/.gazebo"
  # Preserve access to user-downloaded models if present.
  if [[ -n "${ORIG_HOME}" ]] && [[ -d "${ORIG_HOME}/.gazebo/models" ]]; then
    if [[ -n "${GAZEBO_MODEL_PATH:-}" ]]; then
      export GAZEBO_MODEL_PATH="${ORIG_HOME}/.gazebo/models:${GAZEBO_MODEL_PATH}"
    else
      export GAZEBO_MODEL_PATH="${ORIG_HOME}/.gazebo/models"
    fi
  fi
fi

# Keep Gazebo runtime cache/state in workspace when HOME is read-only or restricted.
if [[ -z "${GAZEBO_HOME:-}" ]]; then
  export GAZEBO_HOME="${WS_DIR}/.gazebo_runtime"
fi
mkdir -p "${GAZEBO_HOME}"

# Avoid FastDDS shared-memory permission issues on some managed/containerized environments.
if [[ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ]]; then
  export FASTDDS_BUILTIN_TRANSPORTS="UDPv4"
fi

is_port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -H -ltn 2>/dev/null | awk '{print $4}' | grep -E -q "[:.]${port}$"
    return $?
  fi
  if command -v netstat >/dev/null 2>&1; then
    netstat -ltn 2>/dev/null | awk 'NR > 2 {print $4}' | grep -E -q "[:.]${port}$"
    return $?
  fi
  return 1
}

extract_port_from_uri() {
  local uri="$1"
  if [[ "${uri}" =~ :([0-9]+)$ ]]; then
    echo "${BASH_REMATCH[1]}"
    return 0
  fi
  return 1
}

# Avoid "Address already in use" from stale/default Gazebo master port.
# Keep user override if GAZEBO_MASTER_URI is already provided.
if [[ -n "${GAZEBO_MASTER_URI:-}" ]]; then
  if configured_port="$(extract_port_from_uri "${GAZEBO_MASTER_URI}")"; then
    if is_port_in_use "${configured_port}"; then
      echo "Configured GAZEBO_MASTER_URI (${GAZEBO_MASTER_URI}) is already in use; selecting a new free port."
      unset GAZEBO_MASTER_URI
    fi
  fi
fi

if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then
  base_port=12000
  port_span=20000
  selected_port=""
  for attempt in $(seq 1 300); do
    candidate=$(( base_port + ((BASHPID + RANDOM + attempt) % port_span) ))
    if ! is_port_in_use "${candidate}"; then
      selected_port="${candidate}"
      break
    fi
  done

  if [[ -z "${selected_port}" ]]; then
    echo "Unable to find a free Gazebo master port in candidate range."
    exit 1
  fi

  export GAZEBO_MASTER_URI="http://127.0.0.1:${selected_port}"
  echo "GAZEBO_MASTER_URI was not set. Using free port: ${GAZEBO_MASTER_URI}"
fi

# Isolate each run from stale ROS graphs (for example, another lingering Gazebo
# launch still serving /spawn_entity and /gazebo/model_states).
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID=$(( (BASHPID % 180) + 20 ))
  echo "ROS_DOMAIN_ID was not set. Using isolated domain: ${ROS_DOMAIN_ID}"
fi

ros2 launch mbam_gazebo_tracking run_mbam_gazebo.launch.py "$@"
