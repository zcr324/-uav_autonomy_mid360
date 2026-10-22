# UAV SLAM 与自主飞行框架

2026-10-22 新入口：python3 tools/framework.py。使用、profile、实验和验收边界见 [实验说明](docs/EXPERIMENTS.md)，模块分工见 [架构说明](docs/ARCHITECTURE.md)。
框架启动不会自动起飞；先 baseline，再显式 run。外部定位模式为 navigation_ev，GPS 辅助模式为 navigation。
本轮最终实验结论写入 integration/SESSION_20261022.md；原始日志、bag 和点云保留在 integration/runs/。

# UAV autonomy: 团队模块整合
主工程面向 Ubuntu 20.04 / ROS Noetic / Gazebo 11 / PX4 SITL。
当前整合分支：codex/integrate-team-modules。运行验收结果见 [integration/VALIDATION.md](integration/VALIDATION.md)。

## 当前架构
Gazebo MID-360 Driver2 点云 + 同位 IMU
→ LTA-OM 内置 FAST-LIO 前端
→ 连续里程计/点云 → bridge → EGO-Planner → PositionCommand → bridge → communication → MAVROS/PX4。
STD + GTSAM PGO 并行用于回环验收；修正轨迹暂不直接反馈飞控。
组长经典 FAST-LIO 作为独立基线保存在 integration/baseline_ws（本机生成，不纳入 Git）。
组员规划包快照保存在 repos/ego-planner-team。
来源、接口、修复、架构 survey 和未完成项见 [INTEGRATION.md](INTEGRATION.md)。
旧版记录见 [docs/README_20260817_legacy.md](docs/README_20260817_legacy.md)，其启动方法和话题表已不是当前默认。

## 构建与启动
在 WSL Ubuntu-20.04 中执行：
    cd ~/uav_ws
    bash tools/build_integration.sh
    source env_uav_ws.bash
    bash tools/run_sim.sh gui:=true

默认启动仿真、定位、回环和规划；不解锁，不起飞，不输出控制。
只验定位：loop:=false planning:=false。
已有传感器/rosbag 时，单独启动 modules.launch，避免同时启动第二套 Gazebo。
不要混合 source 组长基线或旧 slam_ws 的 fast_lio。

## 验证
    source ~/uav_ws/env_uav_ws.bash
    python3 ~/uav_ws/tools/test_frame_contracts.py

定位/规划验收后，control:=true 可启用唯一控制链；起飞与航点任务需独立执行。
已在本机完成 GPS 辅助 SITL 的1 m起飞、短航点到达、降落/解除解锁；不代表复杂避障、长轨迹回环收益或实机部署通过。
本轮 PX4 配置 EKF2_AID_MASK=1、EKF2_HGT_MODE=0，视觉位姿已发布，但尚未验收替代 GPS 的外部定位融合。
Driver2 包只提供仿真消息。仿真点内 offset_time=0，不能用于验证真实雷达运动去畸变。


## 受控仿真短链路（本轮已执行）
    bash tools/run_sim.sh gui:=true control:=true
    # 等定位稳定及控制桥接输出 map -> PX4 local 对齐日志
    rosrun mission takeoff.py iris 0 _height:=1.0
    rosrun mission mission.py _takeoff_wait:=0 _waypoints:="[[20.7,4.0,1.2]]"
    # 降落：通过 /xtdrone/iris_0/cmd 发布 AUTO.LAND，确认 armed=false
    bash tools/sim_stop.sh

默认场景为 structured_px4.world，起点 (20,4)。map 和 PX4 本地原点不同，控制适配器在未解锁时标定一次固定变换。
mission.launch 默认不启动第二个通信节点、不自动起飞，航点默认空，避免复用旧起点的任务。
纯长直隧道入口仍有严重定位发散，不是已验收场景。
