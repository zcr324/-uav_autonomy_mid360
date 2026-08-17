#!/usr/bin/env bash
# uav_ws 环境加载脚本（等价于在 .bashrc 中加入这些 source，但不改动 shell 配置）。
# 用法：source ~/uav_ws/env_uav_ws.bash
#
# overlay 顺序（吸取 Mid360_FASTLIO2_检查修改与回退记录.md 第 5 节教训）：
#   noetic -> catkin_ws(gazebo_ros_pkgs) -> slam_ws(修复版 Livox 插件+CustomMsg) -> uav_ws(最后)

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/slam_ws_20260705_share/slam_ws/devel/setup.bash
source ~/uav_ws/devel/setup.bash 2>/dev/null || echo "[env_uav_ws] uav_ws 尚未编译（无 devel），仅加载依赖环境"

# PX4 仿真环境（模型路径、sitl_gazebo 插件）
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo

# Gazebo 插件路径：slam_ws 的 liblivox_laser_simulation.so
export GAZEBO_PLUGIN_PATH=~/slam_ws_20260705_share/slam_ws/devel/lib:$GAZEBO_PLUGIN_PATH

# GTSAM（本地前缀安装，LTA-OM 回环依赖）
export GTSAM_DIR=~/gtsam_install
export CMAKE_PREFIX_PATH=$GTSAM_DIR:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$GTSAM_DIR/lib:$LD_LIBRARY_PATH

echo "[env_uav_ws] noetic + catkin_ws + slam_ws + uav_ws + PX4 + GTSAM 环境已加载"
