# 团队模块整合记录（2026-10-16）

## 方案与来源
主工程 /home/zcr/uav_ws，分支 codex/integrate-team-modules，原始交付目录不修改。
- sim_env：原框架 + 组长 iris_mid360_fl2 模型 + 组员 tunnel_complex.world。
- livox_laser_simulation：组长 fastlio_ws_260825 的 Driver2 仿真插件。
- livox_ros_driver2：组长原始两个 msg 的仿真消息生成包，不包含硬件驱动节点，无需 SDK2。
- lio：保留原框架 LTA-OM 内置 FAST-LIO/STD/PGO，合入同学记录的修复。
- repos/ego-planner-team：组员五个规划包 + quadrotor_msgs 的项目内副本，保留原始规划器副本供对照。
- bridge、mission：原框架修订，统一接口和控制出口。
组长经典 FAST-LIO 在 integration/baseline_ws 独立验证；两套 fast_lio 不能同时 source 或运行。

## 接口契约
|话题|类型|用途|
|---|---|---|
|/livox/lidar|livox_ros_driver2/CustomMsg|四线雷达|
|/livox/imu|sensor_msgs/Imu|同位 IMU|
|/aft_mapped_to_init|nav_msgs/Odometry|连续 LTA-OM 位姿，camera_init|
|/cloud_registered|sensor_msgs/PointCloud2|连续注册点云|
|/iris_0/odom_world|nav_msgs/Odometry|map 下 base_link 位姿，twist 在 base_link|
|/iris_0/map/cloud_registered|sensor_msgs/PointCloud2|同一固定对齐下的 map 点云|
|/iris_0/mavros/vision_pose/pose|geometry_msgs/PoseStamped|base_link 外部位姿|
|/iris_0/move_base_simple/goal|geometry_msgs/PoseStamped|map 航点|
|/iris_0/planning/pos_cmd|quadrotor_msgs/PositionCommand|ENU 位置、yaw、yaw_dot|
|/xtdrone/iris_0/cmd_pose_enu|geometry_msgs/Pose|内部约定 orientation.x=yaw、y=yaw_dot|
|/iris_0/mavros/setpoint_raw/local|mavros_msgs/PositionTarget|ROS 数值保持 ENU，MAVROS 转 MAVLink NED|
|/aft_pgo_path、/aft_pgo_map|Path、PointCloud2|回环验收输出，暂不反馈控制|

base_link -> IMU/LiDAR=(-0.005,0.005,0.147)，旋转相同；LiDAR–IMU 相对外参为零。
桥接点云用固定初始对齐；机体位姿减去旋转后的安装偏移。对齐来自启动参数，不订阅 Gazebo 真值。
前端 TF child 改为 lio_body，避免占用 MAVROS odom。
桥接用连续机体位置差分估计速度；规划器把机体系 twist 旋转回 map。运动噪声需进一步验证。

## 已实施修复
- MID-360 tag=0 支持同时合入两个预处理分支。
- STD NMS 读取 non_max_suppression_radius；补齐 point_skip、touch_filter_enable。
- 保留源码实际使用的 triangle_resolution 键，文档提到的 std_side_resolution 不直接照搬。
- 前端/STD 队列缩小；专用 MID-360 nodelet + PGO launch、单 worker。
- FAST-LIO 补消息生成目标依赖，修复干净编译缺 Pose6D.h。
- 组员规划器直接 Pose 发布关闭；控制出口统一为 bridge -> communication。
- 修复绝对话题与 namespace、目标话题和控制话题不一致。
- 去掉手工 ENU/NED 翻转，修正控制 type_mask 和服务返回检查。
- use_reloc=true 明确拒绝启动，防止语义未确认的位姿流混合输出。
- 保存地图开关保持关闭；同学报告保存分支 SIGSEGV，尚未修复。

## 使用
在 WSL Ubuntu-20.04：
    cd ~/uav_ws
    bash tools/build_integration.sh
    source env_uav_ws.bash
    roslaunch sim_env autonomy.launch gui:=true
默认仿真、定位、回环、规划；不自动解锁/起飞，不输出控制。
只验定位：loop:=false planning:=false。
验证通过后重启可用 control:=true 启动桥接控制和通信，起飞/任务独立执行。
不要混用旧 uav_single/jiong3 和新 team_sim/autonomy 启动入口。

## 初步架构 survey
- [CERLAB](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy)：参考仿真、感知、建图、规划、控制分层。
- [FAR Planner](https://github.com/MichaelFYang/far_planner)：动态可见图可供后续长隧道全局路线层参考，本轮不增加另一套规划控制链。
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)：当前局部避障基础，组员版本有扩展，不等同官方原始版本。
- [LTA-OM](https://github.com/hku-mars/LTAOM)：保留前端、STD、PGO 内部关联接口，经典 FAST-LIO 不能只改话题直接替换。
- [MAVROS 1.20.1](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/plugins/setpoint_raw.cpp)：local_cb 已转换 ENU/NED；本机版本为 1.20.1。
以上是架构接入层面的初步 survey，不是性能评测。
RACER 文件夹提供额外多机探索参考；博客是 MID360/XTDrone/Faster-LIO 基础搭建，并不要求引入 RACER。
组员 test.launch 引用的 sensor_extension 在已查源码目录未找到，新入口不依赖它。
未取得原压缩包或发送方清单，不能证明交付完整，也不能确定依赖缺口由解压导致。

## 验收原则
编译、仿真传感器/定位、规划、起飞悬停、避障航点、回环收益分别记录。
历史报告不能替代本机运行结果；回环检测成功不代表长轨迹精度已验收。

## 本机复现新增发现
1. 同时启动 Gazebo 与 LIO 时，机体落地阶段的 IMU 初始化会导致定位快速发散；等待落稳再启动后消除了该严重发散。增加只用于仿真的传感器就绪门控（重力和角速度稳定 1 秒）。
2. 原交付仿真插件仍保留旧射线长度覆盖逻辑。录制数据最大距离达到约 999 m，而模型配置上限为 30 m。迁移 slam_ws 已记录的射线长度修复，并让过滤阈值直接使用 SDF 配置的 minDist/maxDist。
3. 初次无门控运行、落稳重启、修复后的运行分别保存，不能把修复后结果当作原交付原样复现。

4. CSV 转换后的 zenith 实际作为绕 Y 的 pitch 使用（负值代表朝上），但发布函数用它直接计算 elevation 栅格，丢掉了大部分上方视场。四个发布分支的角度索引统一使用 -zenith；模型仍保持 -7°~55° 垂直 FoV。
5. 仿真模型 downsample 从 1 改为 3（每帧约 8000 条射线），降低 WSL 射线计算负担；不改变视场和逐点时间语义。原始交付模型保留在原目录。

6. 进一步复测发现，直接删除射线长度重置仍不充分：Gazebo 11 的 RayShape::SetPoints 不重置 contactLen，碰撞回调只接受更近的命中，导致每条射线跨帧保留历史最小距离。最终改为每帧从 RelativePoints 计算实际线段长度并 SetLength，再执行碰撞检测，既避免默认量程覆盖，又重置历史命中。
依据：[RayShape.cc](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/RayShape.cc)、[ODERayShape.cc](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/ode/ODERayShape.cc)。
此前仅删除重置的实验结果不作为最终版本验收证据。

7. 规划目标回调原本把所有目标高度硬编码为 1 m。改为保留正值三维目标高度；RViz 二维目标（z=0）沿用 1 m 默认。控制未启用时通过真实 LIO/点云输入验证目标接收和 PositionCommand 输出，不能当作无人机到达目标的证据。
8. 前端 TF 参数修复同步迁入 Driver2 插件：SendRosTf 使用传入父子 frame，并持续发布安装关系。

9. 分离 spawn 高度 z（默认0.5 m）和落稳后的 map 初始高度 init_z（本模型落地间隙约0.055 m）。LIO 等待落稳后初始化，不能继续用抛放高度当初始机体高度。不同模型需重新设定此参数；数值来自本模型验证，不从 Gazebo 真值实时注入。

## 控制参考系与默认场景
默认使用 structured_px4.world，spawn=(20,4,0.5)，落稳后的 map 初始机体高度=0.055 m。
控制适配器在未解锁、时间差不超过0.25 s时，用 map 机体位姿与 MAVROS 本地机体位姿求一次固定 yaw/平移对齐。
这个变换只用于规划指令 map -> PX4 local，不向定位注入仿真真值，也不修改地图。
auto_align=true 为默认，必须在解锁前启动控制适配器。它不补偿随后发生的定位漂移或回环跳变。

启动优先使用 tools/run_sim.sh；它检查 TCP4560 是否仍被占用。
tools/sim_stop.sh 只停止本工程 PID 文件记录的仿真树，并在正常退出超时后清理该树内残留子进程，不再全局 pkill。
本轮复测曾因旧 Gazebo 仍监听4560，让新 PX4 先连到旧实例而卡住；这与场景是否能运行不能混为一谈。
非退化场景的物理步长配置是独立 PX4 profile；不能据此认定1000 Hz 原配置就是唯一卡点根因。

## 本轮闭环飞行边界
SITL 中完成1 m起飞、短航点、降落/解除解锁；到达判据是 LIO map 下误差<0.2 m。
这证明规划/指令/飞控执行接通，不代表定位精度、复杂避障、真实回环收益或实机部署已经通过。
长直隧道入口在最终传感器版本上存在严重定位发散，作为失败用例保留。
STD 静止期间会持续积累子图点，本轮第一关键帧超过230万点；长时间运行的内存控制仍需加固。

本轮读取的实际 EKF 配置为 EKF2_AID_MASK=1、EKF2_HGT_MODE=0，因此飞行是 GPS 辅助 SITL；vision_pose 话题接通不等于 PX4 已融合外部定位。


## 2026-10-22 后续框架与实验记录

最新结论见 [SESSION_20261022.md](integration/SESSION_20261022.md)。本轮飞行建图和绕障有实际记录，但真实回环接受数仍为0，不能沿用历史描述将后端地图导出记作回环优化成功。
