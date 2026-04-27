#!/usr/bin/env bash

# Workspace-aware ROS 2 setup wrapper.
# This avoids relying on colcon's build-time absolute paths so the same workspace
# can be sourced on the Humble controller and the Foxy LIMOs.

_mbam_setup_source_script() {
  if [ -f "$1" ]; then
    . "$1"
  else
    echo "not found: \"$1\"" 1>&2
    return 1
  fi
}

_mbam_setup_resolve_prefix_script() {
  if [ -f "$1" ]; then
    printf '%s\n' "$1"
    return 0
  fi
  if [ -f "$1/setup.bash" ]; then
    printf '%s\n' "$1/setup.bash"
    return 0
  fi
  if [ -f "$1/local_setup.bash" ]; then
    printf '%s\n' "$1/local_setup.bash"
    return 0
  fi
  return 1
}

_mbam_setup_source_prefix() {
  _mbam_setup_candidate="$1"
  _mbam_setup_script="$(_mbam_setup_resolve_prefix_script "$_mbam_setup_candidate")" || {
    echo "Unable to resolve ROS setup script from: ${_mbam_setup_candidate}" 1>&2
    return 1
  }
  _mbam_setup_source_script "$_mbam_setup_script"
}

_mbam_setup_source_extra_underlays() {
  if [ -z "${MBAM_EXTRA_UNDERLAYS:-}" ]; then
    return 0
  fi

  _mbam_setup_old_ifs="$IFS"
  IFS=':'
  for _mbam_setup_extra_underlay in $MBAM_EXTRA_UNDERLAYS; do
    if [ -n "$_mbam_setup_extra_underlay" ]; then
      if ! _mbam_setup_source_prefix "$_mbam_setup_extra_underlay"; then
        IFS="$_mbam_setup_old_ifs"
        unset _mbam_setup_old_ifs
        unset _mbam_setup_extra_underlay
        return 1
      fi
    fi
  done
  IFS="$_mbam_setup_old_ifs"
  unset _mbam_setup_old_ifs
  unset _mbam_setup_extra_underlay
}

_mbam_setup_dir="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"
_mbam_ws_dir="${MBAM_ROS2_WS_DIR:-$_mbam_setup_dir}"
_mbam_install_dir="${MBAM_ROS2_INSTALL_DIR:-$_mbam_ws_dir/install}"

if [ ! -f "$_mbam_install_dir/local_setup.bash" ]; then
  echo "Workspace install setup not found: ${_mbam_install_dir}/local_setup.bash" 1>&2
  return 1
fi

if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  _mbam_underlay_sourced=""
  for _mbam_ros_distro in "${MBAM_ROS_DISTRO:-}" "${ROS_DISTRO:-}" humble foxy jazzy iron rolling galactic; do
    if [ -z "$_mbam_ros_distro" ]; then
      continue
    fi
    if [ -f "/opt/ros/${_mbam_ros_distro}/setup.bash" ]; then
      _mbam_setup_source_script "/opt/ros/${_mbam_ros_distro}/setup.bash" || return 1
      _mbam_underlay_sourced="1"
      break
    fi
  done
  if [ -z "$_mbam_underlay_sourced" ]; then
    echo "Unable to locate a ROS 2 underlay. Set MBAM_ROS_DISTRO or source /opt/ros/<distro>/setup.bash first." 1>&2
    return 1
  fi
  unset _mbam_underlay_sourced
fi

_mbam_setup_source_extra_underlays || return 1

export MBAM_ROS2_WS_DIR="$_mbam_ws_dir"
export MBAM_ROS2_INSTALL_DIR="$_mbam_install_dir"

_mbam_previous_colcon_current_prefix="${COLCON_CURRENT_PREFIX-__MBAM_UNSET__}"
COLCON_CURRENT_PREFIX="$_mbam_install_dir"
_mbam_setup_status=0
_mbam_setup_source_script "$_mbam_install_dir/local_setup.bash" || _mbam_setup_status=$?
if [ "$_mbam_previous_colcon_current_prefix" = "__MBAM_UNSET__" ]; then
  unset COLCON_CURRENT_PREFIX
else
  COLCON_CURRENT_PREFIX="$_mbam_previous_colcon_current_prefix"
fi

if [ "$_mbam_setup_status" -ne 0 ]; then
  unset _mbam_setup_status
  return 1
fi

unset _mbam_setup_status
unset _mbam_previous_colcon_current_prefix
unset _mbam_ros_distro
unset _mbam_install_dir
unset _mbam_ws_dir
unset _mbam_setup_dir
unset _mbam_setup_source_extra_underlays
unset _mbam_setup_source_prefix
unset _mbam_setup_resolve_prefix_script
unset _mbam_setup_source_script
