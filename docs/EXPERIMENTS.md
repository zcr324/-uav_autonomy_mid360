# 在 WSL 使用框架和复现实验

所有命令在 Ubuntu-20.04 的 /home/zcr/uav_ws 执行。原始组员交付目录不修改。
tools/framework.py 是统一入口；启动不会自动解锁或起飞。

## 只看定位和建图
~~~bash
cd /home/zcr/uav_ws
python3 tools/framework.py start --profile mapping --scene structured --headless
python3 tools/framework.py baseline
python3 tools/framework.py view
python3 tools/framework.py stop
~~~
静止只会重建可见区域。要得到沿途地图，需要运动采集或回放包含运动的数据。
mapping profile 不启用飞行控制；不要把它误当作会自动飞一圈的命令。

## 飞行、建图和规划共同验证
~~~bash
python3 tools/framework.py start --profile navigation_ev --scene obstacle --headless
python3 tools/framework.py baseline
python3 tools/framework.py run --route obstacle --arrival 0.5
python3 tools/framework.py capture
python3 tools/framework.py report
python3 tools/framework.py status
python3 tools/framework.py stop
~~~
baseline 必须通过，run 才会解锁。每条路线结束或失败都会尝试降落，并核对解除解锁。
--arrival 是该次导航到达容差；0.3m 精度另外统计，0.5m 导航成功不能当成0.3m精度通过。
失败重跑要用不同 --label，保留前一次结果。
--scene structured 支持 short、multi、long；obstacle 只给障碍后的目标，让 EGO 决定绕行。
tunnel 支持 loop 候选闭合路线，先过静止验收，不能根据几何连通就宣称可飞。

## 定位融合 profile
navigation 默认 GPS 辅助；navigation_ev 是显式外部定位实验：
- PX4 启动前设置 EKF2_AID_MASK=24、EKF2_HGT_MODE=3、EKF2_EV_DELAY=0。
- 使用本机 PX4 的参数定义：bit3 外部位置、bit4 外部偏航，GPS bit0 关闭。
- 导航用 FAST-LIO 位姿，通过 MAVROS vision_pose 进入 EKF。
- 这表示 GPS 不参与 EKF 融合，仿真 GPS 传感器仍存在；不是实机无GPS验收。
- 每次启动生成独立 PX4 工作目录及 rcS，不修改 /home/zcr/PX4_Firmware。
- COM_RCL_EXCEPT=4 是当前 SITL 无物理遥控链路的 OFFBOARD 例外。
实际融合还须看 estimator_status 的 ev_pos、ev_yaw、ev_hgt 和 gps 标志；参数读回本身不足以证明融合。
位姿桥接和规划器不接收 Gazebo 真值。experiment.py 单独读取真值用于误差评估和中止；新版实验驱动器等待估计与真值同时满足到达容差才推进下一目标。这个实验阶段门控依赖真值，不能冒充可部署的任务层。mission.py 自身仅用估计里程计判到达。

## 控制归属
/control_transfer/set_enabled 默认关闭规划转发。
顺序：关闭规划 -> 地面固定对齐 -> 起飞确认 -> 开启规划并发送新目标 -> 降落前关闭。
飞行中禁止重置对齐。开启后等待新的 trajectory_id，防止恢复上次任务残留轨迹。
mission.py 也使用这个服务；空航点、位姿过期、超时和丢失OFFBOARD均失败退出。
takeoff.py 不再与规划器同时输出位置指令。

## 输出是什么
integration/current_run.txt 指向最近运行，integration/session_20261022.json 汇总本轮各次运行。
每个实验目录：
- result.json：判据、到达误差、定位误差、融合参数、回环候选、降落结果。
- events.json、telemetry.csv：时间、目标、连续轨迹和状态。
- data.bag：原始雷达/IMU、位姿、规划和后端接口，用于重放核查。
- map.pcd：从前端注册点云累计的地图，不是仿真世界模型。
- overview.png：估计轨迹、真值、静态场景、误差和点云。
backend_capture/backend_map.pcd 和 ltaom/cloud_result.pcd 是后端地图导出；坐标系 camera_init。
原始前端与后端路径必须在同一坐标系比较；不要直接与 map 坐标图相叠。
config/experiments/mapping.rviz 同时显示前端点云/路径、后端地图/路径和已接受回环线。

## 回环验收
必须分开记录：
1. STD 是否产生非相邻关键帧匹配；
2. PGO 是否通过重叠与残差检查，接受约束；
3. 接受的两帧在仿真真值中是否确实访问同一地点；
4. 相同数据下优化前后轨迹/地图误差是否改善。
loopclosure.txt 在已接受分支写入；opt_debug.txt 的 FPR accept/reject 提供后端证据。
仅输出 /aft_pgo_path、保存地图、或因为相邻特征点约束产生厘米级变化，都不证明回环收益。
现阶段在线全局修正与飞控连续坐标的完整验收仍需另外完成。

## 明确限制
本轮到达容差是原型导航门槛；不是定位精度或避障安全余量。
绕障早期测试曾下降到约0.25m。现已同时限制规划占据查询和控制转发高度，默认0.6–1.6m；两次独立往返复测最低真实高度约0.71m、0.75m。边界不适用于起飞和降落阶段，实验另有实际高度保护。
图上的距离是机体中心至静态几何表面的距离，不能替代完整机体碰撞检测。
仿真雷达为当前瞬时扫描实现，尚未验证真实逐点运动畸变与实机时序。
同一问题最多两次有依据修复/20分钟；依赖失败就停止对应飞行，保存失败结果。

## 离线复核后端
停止实时仿真后，在 source env_uav_ws.bash 的终端执行：
~~~bash
python3 tools/replay_experiment.py /绝对路径/data.bag --output /home/zcr/uav_ws/integration/replays/唯一名称
~~~
默认 raw 模式使用独立 ROS master，仅重放 /livox/lidar 与 /livox/imu。
该模式重新计算前端，初始状态与在线运行不完全相同，不能冒充逐帧相同的在线后端对照。

~~~bash
# 重用完整飞行中积累的原始子图，只重算 STD/PGO
python3 tools/replay_experiment.py /绝对路径/data.bag --mode submaps --rate 2 --output /绝对路径/新目录
# 单独标记的几何细化对照：调用原 STD plane ICP；不降低回环接受阈值
python3 tools/replay_experiment.py /绝对路径/data.bag --mode submaps --refine-loop-icp --rate 2 --output /绝对路径/另一个新目录
# 重用录制的后端接口；加 --omit-loop-constraints 可做同输入的无回环消融
python3 tools/replay_experiment.py /绝对路径/outputs.bag --mode backend --rate 2 --output /绝对路径/新目录
~~~
输出位于独立目录，不能覆盖在线结果。所有回放都不向算法输入 Gazebo 真值。
--lcd-config 和 --pgo-config 可指定独立配置，结果必须标记参数差异。
回放后读取录制真值进行离线核对，与把真值输入算法不同。
tools/check_loop_evidence.py 检查已接受约束及同坐标误差；--reference-run 可指定回放对应的在线目录。
单看前端与PGO差异仍不能把收益归因到回环，因为相邻特征点因子也会改变轨迹。

## 额外扫描及停止条件
--reference-scan 360 在首个航点后执行真实慢速偏航扫描；
--yaw-scan 360 在最终重访后扫描。二者都在1.05m估计高度保持位置，
不注入人工约束。扫描可能改变后续定位状态，应作为独立实验保留。
默认实验总墙钟预算1200秒；单航点超时、定位误差持续超过1m、
数据过期、丢失OFFBOARD或实际路线高度越界都会停止并尝试降落。
同一问题最多两次有依据的修复/20分钟；失败结果不覆盖。

## 独立定点复访模块测试
~~~bash
python3 tools/framework.py start --profile navigation_ev --scene obstacle --headless
python3 tools/framework.py baseline
python3 tools/framework.py run --route obstacle --rotation-revisits 6 --budget 720 --label local_rotation_revisit
~~~
--rotation-revisits 取0..6。大于0时不发送路线目标，只执行真实慢速偏航重访；
--route仅用于场景兼容检查。结果标记navigation_route_evaluated=false。
达到2条接受日志或圈数上限就停止；success只描述该模块的接受条件，
还需要离线核对真实性和收益，不能当作长路线或优化收益验收。
本轮6圈得到9组真实候选，默认PGO全部拒绝，模块验收记失败。
