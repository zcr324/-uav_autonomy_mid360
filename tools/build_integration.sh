#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH="$HOME/gtsam_install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/gtsam_install/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$WS/integration/logs/ltaom" "$WS/integration/logs/px4_integrated"
cd "$WS"
catkin_make --build build_integration -j1 -l2 -DCATKIN_DEVEL_PREFIX="$WS/devel_integration" -DCMAKE_BUILD_TYPE=Release -DGTSAM_DIR="$HOME/gtsam_install/lib/cmake/GTSAM"
