#!/usr/bin/env bash
# Use the self-contained integration build; never source a second fast_lio workspace.
_UAV_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
if [[ -f "$_UAV_WS/devel_integration/setup.bash" ]]; then
  source "$_UAV_WS/devel_integration/setup.bash"
fi
export UAV_WS="$_UAV_WS"
export PX4_ROOT="${PX4_ROOT:-$HOME/PX4_Firmware}"
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default" >/dev/null
export ROS_PACKAGE_PATH="$_UAV_WS/src:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo:${ROS_PACKAGE_PATH}"
export GAZEBO_MODEL_PATH="$_UAV_WS/src/sim_env/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$_UAV_WS/devel_integration/lib:${GAZEBO_PLUGIN_PATH}"
export GTSAM_DIR="$HOME/gtsam_install/lib/cmake/GTSAM"
export CMAKE_PREFIX_PATH="$HOME/gtsam_install:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/gtsam_install/lib:${LD_LIBRARY_PATH}"
unset _UAV_WS
