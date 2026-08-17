#!/usr/bin/env bash
# M1 诊断：复现 gzserver 段错误并抓取调用栈（catchsegv，无需 gdb/sudo）。
# 用法：bash ~/uav_ws/tools/m1_diag.sh
# 跑完把终端输出贴给 Claude；全部日志在 /tmp/m1_diag/
set -u
source ~/uav_ws/env_uav_ws.bash > /dev/null 2>&1
LOG=/tmp/m1_diag
mkdir -p $LOG
OUT=$LOG/summary.txt
echo "DIAG START $(date)" > $OUT

log() { echo "$@" | tee -a $OUT; }

log "== 1. roscore =="
RCPID=""
if ! pgrep -x rosmaster > /dev/null; then
  roscore > $LOG/roscore.log 2>&1 &
  RCPID=$!
  sleep 4
  log "started roscore pid=$RCPID"
else
  log "roscore already running, reuse it"
fi

log "== 2. gzserver（catchsegv 抓栈）=="
catchsegv gzserver -e ode \
  -s libgazebo_ros_api_plugin.so \
  -s libgazebo_ros_paths_plugin.so \
  $(rospack find sim_env)/worlds/non_degenerate_control_tunnel.world \
  > $LOG/gzserver.log 2>&1 &
GZPID=$!
log "gzserver pid=$GZPID（崩溃栈会写入 $LOG/gzserver.log）"

log "== 3. 等 /gazebo/spawn_sdf_model 服务 =="
READY=no
for i in $(seq 1 20); do
  if timeout 3 rosservice info /gazebo/spawn_sdf_model > /dev/null 2>&1; then READY=yes; break; fi
  sleep 2
done
log "spawn service ready=$READY"

log "== 4. PX4 SITL =="
cd ~/.ros
PX4_SIM_MODEL=iris PX4_ESTIMATOR=ekf2 \
  /home/zcr/PX4_Firmware/build/px4_sitl_default/bin/px4 \
  /home/zcr/PX4_Firmware/build/px4_sitl_default/etc \
  -s etc/init.d-posix/rcS -i 0 -w sitl_iris_0 -d > $LOG/px4.log 2>&1 &
PXPID=$!
log "px4 pid=$PXPID"

log "== 5. spawn iris_mid360（60s 超时保护）=="
timeout 60 rosrun gazebo_ros spawn_model -sdf \
  -file /home/zcr/PX4_Firmware/Tools/sitl_gazebo/models/iris_mid360/iris_mid360.sdf \
  -model iris_0 -x 0 -y 7.5 -z 0.5 > $LOG/spawn.log 2>&1
log "spawn exit=$? 末尾: $(tail -1 $LOG/spawn.log)"

log "== 6. 观察 25s =="
sleep 25
if kill -0 $GZPID 2>/dev/null; then
  log "RESULT: gzserver 存活（本次未复现崩溃）"
else
  log "RESULT: gzserver 崩溃！"
fi

log "== 7. 汇总 =="
log "--- gzserver 日志末尾（含崩溃栈）---"
tail -30 $LOG/gzserver.log | tee -a $OUT
log "--- px4 日志末尾 ---"
tail -8 $LOG/px4.log | tee -a $OUT
log "--- spawn 日志末尾 ---"
tail -3 $LOG/spawn.log | tee -a $OUT
log "DIAG DONE，全部日志: $LOG/"
