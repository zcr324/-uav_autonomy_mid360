#!/usr/bin/env bash
# 彻底停止整套仿真（清理 roslaunch 崩溃/退出后的残留进程）。
# 用法：bash ~/uav_ws/tools/sim_stop.sh
# 建议：每次启动仿真前先跑一遍；Ctrl-C 关不掉时也用它兜底。

echo "[sim_stop] 停止所有仿真相关进程..."

# 1. roslaunch（先停启动器，防止节点重启）
pkill -f "roslaunch" 2>/dev/null
sleep 1

# 2. Gazebo（gzserver 是 daemon 化最顽固的）
pkill -x gzserver 2>/dev/null
pkill -x gzclient 2>/dev/null
pkill -f "gazebo_ros/scripts/gzserver" 2>/dev/null
pkill -f "gazebo_ros/scripts/gzclient" 2>/dev/null

# 3. PX4 SITL
pkill -f "build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -x px4-simulator 2>/dev/null

# 4. MAVROS + 本框架的桥接/任务脚本
pkill -x mavros_node 2>/dev/null
pkill -f "multirotor_communication.py" 2>/dev/null
pkill -f "takeoff.py" 2>/dev/null
pkill -f "lio_transfer.py" 2>/dev/null
pkill -f "control_transfer.py" 2>/dev/null
pkill -f "mission.py" 2>/dev/null

# 5. nodelet（LTA-OM 回环检测用）
pkill -f "nodelet" 2>/dev/null

# 6. 最后 ROS master（roscore）
pkill -f "rosmaster" 2>/dev/null
pkill -f "roscore" 2>/dev/null

sleep 2

LEFT=0
for p in gzserver gzclient mavros_node rosmaster; do
  pgrep -x $p > /dev/null 2>&1 && LEFT=$((LEFT + $(pgrep -x $p | wc -l)))
done
LEFT=$((LEFT + $(pgrep -f "bin/px4" | wc -l)))
LEFT=$((LEFT + $(pgrep -f "px4-simulator" | wc -l)))
LEFT=$((LEFT + $(pgrep -f "roslaunch" | wc -l)))

if [ "$LEFT" -eq 0 ]; then
  echo "[sim_stop] 干净了 ✓"
else
  echo "[sim_stop] 仍有 $LEFT 个残留："
  pgrep -ax gzserver gzclient mavros_node rosmaster 2>/dev/null
  pgrep -af "bin/px4|px4-simulator|roslaunch" 2>/dev/null | grep -v "sim_stop"
fi
