# uav_autonomy_mid360

面向 **PX4 + 览沃 Livox Mid360** 的无人机自主飞行框架：**LTA-OM 定位**（FAST-LIO2 前端 + STD 回环检测 + GTSAM 位姿图优化）、**EGO-Planner 实时避障**、**航迹任务执行**，Gazebo SITL 全链路仿真（ROS Noetic / Ubuntu 20.04）。

> 状态：框架与全部模块**已编译通过**；运行时链路调试中（见 [当前状态](#当前状态与待办)）。

---

## 1. 架构

```
┌─ sim_env（仿真环境）────────────────────────────────────────────┐
│  Gazebo + PX4 SITL + MAVROS（单机，world 可切换）                │
│  修复版 Livox 仿真插件（slam_ws 提供）→ /iris_0/scan            │
│  imu_gazebo 模型 → /iris_0/imu_gazebo                           │
├─ lio（定位，LTA-OM 衍生）───────────────────────────────────────┤
│  fast_lio：FAST-LIO2 前端 → /aft_mapped_to_init + /cloud_registered │
│  std_loop：STD 描述子回环检测                                    │
│  loop_optimization：GTSAM 位姿图优化（离线 PGO）                │
│  multisession 模式：加载先验地图重定位                           │
├─ bridge（桥接，自写 py3）───────────────────────────────────────┤
│  lio_transfer：camera_init → map 坐标对齐（手起参数/reloc 输出） │
│    → /iris_0/mavros/vision_pose/pose + /iris_0/odom_world      │
│  control_transfer：规划器 PositionCommand(ENU) → XTDrone 指令链  │
├─ planning（避障规划，EGO-Planner）──────────────────────────────┤
│  plan_env / path_searching / bspline_opt / traj_utils /         │
│  ego_planner(plan_manage) → /iris_0/planning/pos_cmd            │
├─ mission（任务，自写 py3）──────────────────────────────────────┤
│  multirotor_communication：60Hz MAVROS setpoint 流 + 模式/解锁   │
│  takeoff：起飞时序（OFFBOARD→ARM→HOVER）                         │
│  mission：航点执行（/move_base_simple/goal）                     │
└──────────────────────────────────────────────────────────────────┘
```

模块化原则借鉴 CERLAB-UAV-Autonomy：每模块一个包，定位/规划/控制/仿真互不侵入，话题即接口。

## 2. 目录

| 目录 | 来源 | 说明 |
|---|---|---|
| `src/sim_env` | 自写 | launch / 模型（含 Gazebo11 修复）/ jiong3 + 隧道 world |
| `src/lio` | hku-mars/LTAOM（vendor + 修复） | FAST-LIO2 前端 + 回环 + PGO + 重定位 |
| `src/planning/*` | ZJU-FAST-Lab/ego-planner（符号链接） | 见 setup_deps.sh |
| `src/bridge` | 自写 py3 | lio_transfer / control_transfer |
| `src/mission` | 自写 py3 | communication / takeoff / mission |
| `tools/` | 自写 | sim_stop.sh 清进程 / m1_diag.sh 崩溃诊断 / setup_deps.sh 拉依赖 |
| `env_uav_ws.bash` | 自写 | 环境加载（overlay 顺序 + GTSAM + Gazebo 路径） |

## 3. 话题与消息协议

### 3.1 输入（仿真 → 定位）

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/iris_0/scan` | `livox_ros_driver/CustomMsg`（v1） | 10 Hz | Mid360 仿真点云，frame `laser_livox` |
| `/iris_0/imu_gazebo` | `sensor_msgs/Imu` | ~250 Hz | frame `imu_link_stereo` |

### 3.2 定位输出（lio → 桥接）

| 话题 | 类型 | frame | 说明 |
|---|---|---|---|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | camera_init | LTA-OM 里程计（**注意：不是经典 FAST-LIO2 的 /Odometry**） |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | camera_init | 配准点云/局部地图 |
| `/aft_pgo_path` | `nav_msgs/Path` | — | PGO 修正后轨迹 |

### 3.3 桥接输出（bridge → 飞控/规划）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/iris_0/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | PX4 视觉外部位姿（ENU，frame `map`） |
| `/iris_0/odom_world` | `nav_msgs/Odometry` | 规划器里程计输入（frame `map`，含 twist） |
| `/iris_0/map/cloud_registered` | `sensor_msgs/PointCloud2` | 规划器局部地图（frame `map`） |

### 3.4 规划 → 控制

| 话题 | 类型 | 说明 |
|---|---|---|
| `/iris_0/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | EGO-Planner 轨迹命令（ENU） |
| `/xtdrone/iris_0/cmd_pose_enu` | `geometry_msgs/Pose` | control_transfer 转换（orientation.x=yaw, y=yaw_dot，XTDrone 约定） |
| `/iris_0/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 60Hz 连续流（coordinate_frame=LOCAL_NED，type_mask=960） |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 航点注入（mission.py → EGO-Planner） |

### 3.5 坐标系与 TF 纪律

- **map ≡ world ≡ ENU 原点**；PX4 内部 NED 转换只发生在 control_transfer 输出端（x→x, y→-y, z→-z, yaw→-yaw）
- **产品链路零 TF 依赖**：规划/控制全部走消息；TF 仅用于 rviz 调试
- 三棵 TF 树严格隔离：Gazebo 真值树（`world→livox_base→laser_livox`，仅验证用）、LIO 树（`camera_init→body`）、MAVROS 树（不插入任何节点）
- `map→camera_init` 对齐两阶段：PGO 前 = 手起对准常量参数（spawn 位姿，禁止 Gazebo 真值作弊）；PGO 后 = LTA-OM reloc 输出（`~use_reloc:=true`）
- 静态 TF：`base_link→imu_link_stereo (0,0,0.25)`

## 4. 环境搭建

```bash
# 依赖：ROS Noetic、PCL、libtbb-dev、libceres-dev、libgoogle-glog-dev、libgflags-dev
sudo apt-get install -y libtbb-dev libceres-dev libgoogle-glog-dev libgflags-dev

# GTSAM 4.0.3 + LTA-OM 补丁（backup/recover），装到 ~/gtsam_install
# 补丁见 src/lio/README.md（LTA-OM 官方 Note），已应用的源码在 ~/gtsam

# 前置工作空间 slam_ws（提供修复版 Livox 仿真插件 + CustomMsg），需先编译
# PX4_Firmware（XTDrone 改版 rcS）+ 模型安装：
#   iris_mid360、Mid360_0 → PX4_Firmware/Tools/sitl_gazebo/models/
#   jiong3.world → PX4_Firmware/Tools/sitl_gazebo/worlds/

# 本仓库
source env_uav_ws.bash          # 或用 bash tools/setup_deps.sh 一键
bash tools/setup_deps.sh        # 克隆 ego-planner 并建立符号链接
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 5. 启动流程

```bash
source ~/uav_ws/env_uav_ws.bash
bash ~/uav_ws/tools/sim_stop.sh          # 先清残留（重要！）

# 终端1：仿真（用户终端跑可看 GUI/独显渲染）
roslaunch sim_env uav_single.launch

# 终端2：定位前端
roslaunch fast_lio mapping_mid360.launch rviz:=false

# 终端3：桥接
roslaunch bridge bridge.launch           # PGO 后加 use_reloc:=true

# 终端4：起飞 + 航迹
rosrun mission multirotor_communication.py iris 0
rosrun mission takeoff.py iris 0
rosrun mission mission.py _waypoints:="[[x,y,z],...]"
```

jiong3 小闭环世界（回环验证）：`roslaunch sim_env jiong3_single.launch`。

## 6. 踩坑记录（本仓库已修复）

1. **gzserver 段错误（Gazebo 11）**：XTDrone 老模型把 gps/magnetometer/barometer 插件放 model 级，Gazebo 11 类型检查拒绝 → 清理时 `GpsPlugin::~GpsPlugin()` 调 `world_->Reset()` 段错误。修复：插件包进 `<sensor>` 元素（见 iris_mid360.sdf 注释）
2. **LTA-OM 编译坑三连**：livox_ros_driver 子模块缺 CMake（从 slam_ws 复制）；STD 的 TBB 硬编码作者路径（改系统路径）；GTSAM 需打 backup/recover 补丁
3. **LTA-OM 无独立可执行文件**：上游把 `fastlio_mapping` 的 add_executable 注释 + 源码内 `#define as_node` 写死 → 改为 CMake 目标级宏（库带 as_node、可执行文件不带）
4. **slam_ws 顶层 CMakeLists 损坏**（一行裸路径）→ 替换官方 toplevel.cmake；插件 CSV 硬编码 /home/sitp → 本机路径

## 7. 当前状态与待办

| 里程碑 | 状态 |
|---|---|
| M0 环境与依赖 | ✅ 完成 |
| M1 仿真基线 | 🔶 仿真/PX4/MAVROS/雷达链路健康；**PX4 HIL 数据链卡点**（TCP 4560 已连但插件不发 HIL_SENSOR/GPS → 无 home position，起飞待修） |
| M2 LTA-OM 前端 | 🔶 编译通过、节点运行（订阅正常、CPU 计算中）；**/aft_mapped_to_init 尚无输出**（初始化发布条件待查） |
| M3 离线回环（PGO） | ⏳ |
| M4 重定位（multisession） | ⏳ |
| M5 航迹飞行 | ⏳ |

## 8. 相关仓库

- [hku-mars/LTAOM](https://github.com/hku-mars/LTAOM)（定位栈，已 vendor 并修复）
- [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)（规划栈，setup_deps.sh 拉取）
- slam_ws（团队内部：修复版 Livox 仿真插件，本仓库的前置依赖）
