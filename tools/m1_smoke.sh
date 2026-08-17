#!/usr/bin/env bash
# M1 冒烟测试：启动单机仿真（Gazebo+PX4 SITL+MAVROS），检查核心话题与坐标系偏移。
# 用法：bash ~/uav_ws/tools/m1_smoke.sh [world]   world: tunnel(默认) | jiong3
set -e
source ~/uav_ws/env_uav_ws.bash > /dev/null

WORLD=${1:-tunnel}
if [ "$WORLD" == "jiong3" ]; then
  LAUNCH_ARGS="jiong3_single.launch"
else
  LAUNCH_ARGS="uav_single.launch"
fi

echo "[m1_smoke] 启动 $LAUNCH_ARGS（后台），日志: /tmp/m1_sim.log"
roslaunch sim_env $LAUNCH_ARGS > /tmp/m1_sim.log 2>&1 &
LAUNCH_PID=$!
echo "[m1_smoke] roslaunch PID=$LAUNCH_PID"

echo "[m1_smoke] 等待 60s 让 Gazebo/PX4/MAVROS 就绪..."
sleep 60

echo "[m1_smoke] === 关键话题检查 ==="
for t in /iris_0/scan /iris_0/imu_gazebo /iris_0/mavros/local_position/pose /gazebo/model_states /tf; do
  n=$(rostopic list 2>/dev/null | grep -c "^$t$" || true)
  if [ "$n" -ge 1 ]; then echo "  [OK] $t"; else echo "  [MISS] $t"; fi
done

echo "[m1_smoke] === scan 频率（3s 采样）==="
timeout 5 rostopic hz /iris_0/scan 2>&1 | tail -2 || true

echo "[m1_smoke] === PX4 local_position（ENU，由 MAVROS 转换）vs Gazebo 真值 ==="
echo "--- /iris_0/mavros/local_position/pose ---"
timeout 3 rostopic echo -n 1 /iris_0/mavros/local_position/pose 2>/dev/null | grep -E "x:|y:|z:" || true
echo "--- /gazebo/model_states（iris_0）---"
python3 - <<'EOF'
import rospy
from gazebo_msgs.msg import ModelStates
rospy.init_node('m1_truth_check', anonymous=True)
msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=30)
try:
    i = msg.name.index('iris_0')
    p = msg.pose[i].position
    print("iris_0 truth: x=%.3f y=%.3f z=%.3f" % (p.x, p.y, p.z))
except ValueError:
    print("iris_0 not in model_states (names=%s)" % msg.name)
EOF

echo "[m1_smoke] 结束：停止仿真（Ctrl-C 或 kill $LAUNCH_PID）"
echo "kill $LAUNCH_PID"
