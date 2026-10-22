# UAV SLAM 与自主飞行框架

主工作区：WSL Ubuntu-20.04 的 /home/zcr/uav_ws。
环境：ROS Noetic、Gazebo 11、PX4 SITL；整合分支 codex/integrate-team-modules。
原始组员交付目录保留，改动只在本工程副本内进行。

## 六个模块
1. 仿真与传感器：Gazebo/PX4 + 组长的 MID-360 Driver2 模型与插件。
2. 前端定位：LTA-OM 改造的 FAST-LIO2 技术路线分支；组长独立 FAST-LIO2 另留基线，不能同时 source。
3. 后端：STD 回环检测 + GTSAM PGO，结合回环同学的修复记录。
4. 规划：EGO 同学的规划包，通过注册点云和里程计生成轨迹。
5. 桥接与控制：固定坐标对齐、安装补偿、规划控制开关、唯一 MAVROS 指令出口。
6. 任务与验证：起飞、航点、降落、profile、日志、地图导出和原始传感器回放。

Gazebo -> FAST-LIO -> bridge -> EGO -> control_transfer -> communication -> MAVROS/PX4。
FAST-LIO -> STD -> PGO -> 优化路径与地图。全局回环修正进入飞控连续坐标的完整验收仍单独保留。

## 统一入口
~~~bash
cd /home/zcr/uav_ws
bash tools/build_integration.sh
python3 tools/framework.py start --profile navigation_ev --scene obstacle --headless
python3 tools/framework.py baseline
python3 tools/framework.py run --route obstacle --arrival 0.5
python3 tools/framework.py capture
python3 tools/framework.py report
python3 tools/framework.py status
python3 tools/framework.py stop
~~~
start 不解锁、不起飞；baseline 通过后才能 run。每个任务失败或结束均尝试降落并核对解除解锁。
navigation 是 GPS 辅助；navigation_ev 在 PX4 启动前配置外部位置/偏航/高度融合并关闭 GPS 融合。
每次运行使用独立 PX4 工作目录和结果目录。不要混用旧工作区或同时启动第二套仿真/控制器。

## 怎样看结果
- [精选结果图、地图及原始结果JSON](docs/results/20261022/README.md)：含成功与失败案例，回环接受数仍为0。
- [六模块完成度](docs/STATUS.md)：区分接口接通与功能验收。

- [本轮实验结论](integration/SESSION_20261022.md)：通过、失败与停止原因。
- [实验操作说明](docs/EXPERIMENTS.md)：profile、地图、回环判据与复现方法。
- [架构与分工](docs/ARCHITECTURE.md)：模块、坐标系和接口责任。
- integration/session_20261022.json：本轮所有运行目录。
- integration/current_run.txt：最近运行目录；内有 bag、PCD、JSON、CSV 和轨迹图。
- python3 tools/framework.py view：运行中打开 RViz 前后端对照配置。
- [历史整合记录](INTEGRATION.md)、[上一版 README](docs/README_20261016_integration.md)：仅作为历史背景。

## 验收边界
导航到达容差与精度分别统计；0.5m 到达不等于0.3m精度通过。
已验证外部定位融合进入 PX4，GPS 不参与该模式 EKF 融合；仿真 GPS 传感器仍存在。
后端地图保存与路径输出不等于真实回环成功；需检查已接受约束及同数据的优化收益。
已加入0.6–1.6m规划与指令高度边界；两次绕障复测最低真实高度约0.71m和0.75m。当前结果属于 SITL 原型验证，不是实机验收。
LiDAR 点内 offset_time=0，尚不能验证真实逐点去畸变。

## 重点回归
~~~bash
source env_uav_ws.bash
python3 tools/test_frame_contracts.py
python3 tools/test_traj_clock.py
~~~
坐标变换、ENU 指令、控制归属、重置对齐限制及真实轨迹服务器的零时间差行为均有针对性验证。
