#!/usr/bin/env bash

# Workspace-aware local setup wrapper.
# This only applies the overlay in this workspace and intentionally does not
# source a ROS underlay.

_mbam_local_setup_source_script() {
  if [ -f "$1" ]; then
    . "$1"
  else
    echo "not found: \"$1\"" 1>&2
    return 1
  fi
}

_mbam_local_setup_dir="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"
_mbam_local_ws_dir="${MBAM_ROS2_WS_DIR:-$_mbam_local_setup_dir}"
_mbam_local_install_dir="${MBAM_ROS2_INSTALL_DIR:-$_mbam_local_ws_dir/install}"

if [ ! -f "$_mbam_local_install_dir/local_setup.bash" ]; then
  echo "Workspace install setup not found: ${_mbam_local_install_dir}/local_setup.bash" 1>&2
  return 1
fi

export MBAM_ROS2_WS_DIR="$_mbam_local_ws_dir"
export MBAM_ROS2_INSTALL_DIR="$_mbam_local_install_dir"

_mbam_local_previous_colcon_current_prefix="${COLCON_CURRENT_PREFIX-__MBAM_UNSET__}"
COLCON_CURRENT_PREFIX="$_mbam_local_install_dir"
_mbam_local_setup_status=0
_mbam_local_setup_source_script "$_mbam_local_install_dir/local_setup.bash" || _mbam_local_setup_status=$?
if [ "$_mbam_local_previous_colcon_current_prefix" = "__MBAM_UNSET__" ]; then
  unset COLCON_CURRENT_PREFIX
else
  COLCON_CURRENT_PREFIX="$_mbam_local_previous_colcon_current_prefix"
fi

if [ "$_mbam_local_setup_status" -ne 0 ]; then
  unset _mbam_local_setup_status
  return 1
fi

unset _mbam_local_setup_status
unset _mbam_local_previous_colcon_current_prefix
unset _mbam_local_install_dir
unset _mbam_local_ws_dir
unset _mbam_local_setup_dir
unset _mbam_local_setup_source_script
