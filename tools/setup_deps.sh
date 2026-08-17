#!/usr/bin/env bash
# 拉取外部依赖（ego-planner）并建立工作空间符号链接。
# 用法：bash ~/uav_ws/tools/setup_deps.sh
set -e
WS=/home/zcr/uav_ws
mkdir -p $WS/repos

echo "[setup_deps] 1/2 克隆 ego-planner（未修改的官方源码）..."
if [ ! -d $WS/repos/ego-planner ]; then
  git clone --depth 1 https://github.com/ZJU-FAST-Lab/ego-planner.git $WS/repos/ego-planner
fi

echo "[setup_deps] 2/2 修复 Kinetic 悬空符号链接 + 建立包软链..."
# 上游 src/CMakeLists.txt 是指向 /opt/ros/kinetic 的悬空符号链接，替换为 Noetic toplevel
if [ -L $WS/repos/ego-planner/src/CMakeLists.txt ]; then
  rm $WS/repos/ego-planner/src/CMakeLists.txt
  cp /opt/ros/noetic/share/catkin/cmake/toplevel.cmake $WS/repos/ego-planner/src/CMakeLists.txt
fi
cd $WS/src
for p in plan_env path_searching bspline_opt traj_utils plan_manage; do
  ln -sfn ../repos/ego-planner/src/planner/$p $p
done
ln -sfn ../repos/ego-planner/src/uav_simulator/Utils/quadrotor_msgs quadrotor_msgs

echo "[setup_deps] 完成。接下来："
echo "  source ~/uav_ws/env_uav_ws.bash && cd ~/uav_ws && catkin_make -DCMAKE_BUILD_TYPE=Release"
