# LTA-OM 回环链路：问题定位、修复与验证记录

> 归属模块：`src/lio`（FAST_LIO / STD / loop_optimization）
> 记录状态：已在本验证环境端到端复现并修复；两项修复建议直接应用于本仓库
> 关联里程碑：**M2（LTA-OM 前端）/ M3（离线回环 PGO）**

---

## 1. 结论摘要

1. 本仓库 M2 记录的卡点「`/aft_mapped_to_init` 尚无输出」已在独立环境中定位到根因：**`FAST_LIO/src/preprocess.cpp` 中 `avia_handler` 支持 MID-360 的判断分支被注释**，导致仿真点云（`tag == 0`）在预处理阶段被全部丢弃，预处理输出 0 点。**修改一行即可解除**（见 §3.1）。
2. M3「离线回环 PGO」已独立跑通：STD 检出有效回环，GTSAM 输出优化后关键帧位姿。打通前需先修复 **`STD/src/std.cpp` 配置读取键名错误**（见 §3.2），该错误会导致 STD 在任何场景下都几乎提取不到特征。
3. 另记录两项工程性问题：ROS 队列过大导致的**内存膨胀**（§3.3，已修复）；`/save_map` + `/SavePCD` 触发的**段错误**（§3.4，未解决，附绕过方案）。
4. 上述两处源码级问题均与雷达驱动版本无关，**对本仓库现有的 `livox_ros_driver` v1 链路同样适用**。

---

## 2. 验证环境与本仓库的差异对照

| 项 | 整机仓库 | 本次验证环境 |
|---|---|---|
| 雷达驱动 | `livox_ros_driver`（v1） | `livox_ros_driver2`（v2，仿真插件输出） |
| 点云输入 | `/iris_0/scan`，`livox_ros_driver/CustomMsg`，10 Hz | `/livox/lidar`，`livox_ros_driver2/CustomMsg`，9.8 Hz |
| IMU 输入 | `/iris_0/imu_gazebo`，`sensor_msgs/Imu`，约 250 Hz | `/livox/imu`，`sensor_msgs/Imu`，124 Hz |
| 定位输出 | `/aft_mapped_to_init`（`camera_init`） | 同 |
| 运行载体 | PX4 SITL + MAVROS + 无人机模型 | Gazebo Classic 11（无 PX4），脚本驱动地面平台 |
| 运行环境 | Ubuntu 20.04 + ROS Noetic（原生） | Ubuntu 22.04 主机 + Docker 容器（20.04 + Noetic + Gazebo Classic 11） |
| 回环后端 | `loop_optimization`（GTSAM PGO） | 同（本仓库 vendor 版本的 Driver2 适配版） |

**差异带来的影响**：本环境为接入 v2 消息对 `src/lio` 做了 Driver1 → Driver2 适配（CMakeLists / package.xml / preprocess / laserMapping / STD 遗留头文件）。**该适配项与本仓库无关**，本仓库沿用 v1 链路；但 §3.1、§3.2 两项修复与驱动版本无关。

---

## 3. 问题清单与修复

### 3.1 【P0｜解 M2】预处理丢弃全部点云，SLAM 不处理任何帧

**位置**：`src/lio/FAST_LIO/src/preprocess.cpp`，`Preprocess::avia_handler` 的非特征提取分支

**现象**

| 观测项 | 数值 |
|---|---|
| `/livox/lidar` | 9.8 Hz，每帧 4000~8000 点（数据正常） |
| 预处理输出点数 | **0** |
| `/aft_mapped_to_init` | **无任何消息** |
| 节点内存 | 持续上涨（数据只入不出），最高触及 5~7 GB |

**根因**

```cpp
if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10)
   //|| (msg->points[i].tag & 0x30) == 0x00 //mid360     <- 支持 MID-360 的分支被注释
   )
```

该条件只接受 `tag & 0x30 == 0x10`；而 MID-360 仿真插件输出的 `tag` 为 `0`（`tag & 0x30 == 0x00`），因此**所有点被过滤**，`pl_surf` 为空，后续 `sync_packages` 因"点数 ≤ 1 直接丢弃该帧"而永远无法进入 SLAM 主流程。

**修复**

```cpp
if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10
   || (msg->points[i].tag & 0x30) == 0x00 //mid360
   ))
```

**验证**

| 验证项 | 修复前 | 修复后 |
|---|---|---|
| 预处理输出点数 | 0 | 837 ~ 1594 |
| `[ mapping ]: time: IMU + Map + Input Downsample ...` | 无 | 每帧持续输出（约 6 ms） |
| `/aft_mapped_to_init` | 无消息 | 9.7 Hz |

**备注**：`standard_pcl_cbk` 分支（PointCloud2 输入）不含该 tag 判据，因此仅 Livox CustomMsg 链路受影响。

---

### 3.2 【P0｜解 M3】STD 非极大值抑制半径读取错误配置键

**位置**：`src/lio/STD/src/std.cpp`，`ConfigSetting` 读取段（约第 112 行）

**现象**

| 观测项 | 数值 |
|---|---|
| `[Corner] corner size` | 1 ~ 5 |
| `[Corner] descriptor size` | 0 |
| `Loop Sucess` | 0（`loopclosure.txt` 为空） |
| 反直觉现象 | **增大 `max_constrait_dis` 后角点反而更少** |

**根因**

```cpp
config_setting.non_max_suppression_radius_ = fSettings["max_constrait_dis"];   // 键名错误
```

`non_max_suppression_radius_` 被 `max_constrait_dis` 的值顶替。该值实际为 2 ~ 10 m，而 `binary_extractor` 提取出的候选角点数量为 **2035 个/关键帧**，经该半径抑制后仅剩 1 ~ 5 个，导致 `generate_std` 无法构成三角形描述子。**此问题与场景无关，任何数据集都会复现。**

**修复**

```diff
- config_setting.non_max_suppression_radius_ = fSettings["max_constrait_dis"];
+ config_setting.non_max_suppression_radius_ = fSettings["non_max_suppression_radius"];
```

并在 STD 配置文件中补齐缺失键（`src/lio/STD/config/*.yaml`）：

```yaml
non_max_suppression_radius: 0.5
std_side_resolution: 0.2
touch_filter_enable: 0
point_skip: 1
max_constrait_dis: 2
```

**验证**

| 验证项 | 修复前 | 修复后 |
|---|---|---|
| `corner size` | 1 ~ 5 | **60** |
| `descriptor size` | 0 | **371 ~ 881** |
| `Loop Sucess` | 0 | **6 条以上**（`icp` 0.474 ~ 0.715） |

---

### 3.3 【P1】ROS 队列过大导致内存膨胀

**位置**

- `src/lio/FAST_LIO/src/laserMapping.cpp`：点云订阅队列 `200000`；发布队列（`/cloud_registered`、`/aft_mapped_to_init` 等）`10000` / `100`
- `src/lio/STD/src/loop_detection.cpp`：`/cloud_registered`、`/aft_mapped_to_init` 订阅队列 `1000`

**现象**：当消费者（STD）处理速度低于生产者（FAST-LIO）时，消息在队列中累积，nodelet 进程内存可达 5~7 GB，虚拟机出现假死。

**修复建议**

| 位置 | 原值 | 建议值 |
|---|---|---|
| FAST-LIO 点云订阅 | 200000 | 5 |
| FAST-LIO IMU 订阅 | 200000 | 200 |
| FAST-LIO 发布（关键话题） | 10000 | 2 |
| FAST-LIO 发布（其余话题） | 100 | 2 |
| STD 订阅 | 1000 | 20 |

另建议在 `run_all_*.launch` 的 nodelet manager 中显式设置 `<param name="num_worker_threads" value="1" />`（原为注释状态），使回调串行执行，避免多线程乱序触发 `loop back, clear buffer` 而误清空缓冲区。

**备注**：§3.1 修复后（SLAM 正常产出里程计），内存膨胀问题同步消失；§3.3 属健壮性加固。

---

### 3.4 【P1｜未解决】保存修正地图时段错误

**触发条件**：运行期间设置 `/save_map true` 或 `/SavePCD true`

**现象**

- `sample_nodelet_manager` 以 `exit code -11`（SIGSEGV）退出，伴随 `Bond broken, exiting`
- 崩溃发生较早（`lc_debug.txt` 仅记录到关键帧 0）
- 系统内存占用正常（约 1.3 GB），**非 OOM**
- 未产出 `cloud_result.pcd`（保存逻辑位于 `loop_optimization_node.cpp:1583`，由 `savemap` 变量门控，`/save_map` 在约 1796 行读取）

**当前处置（绕过方案）**：不启用保存开关，改为录制优化器发布的话题获取修正地图：

```bash
rosbag record -O corrected_map.bag /aft_pgo_map /cloud_corrected
```

**待办**：定位 `savemap` 分支内的越界访问；在此之前 M3 的产物以 `optimized_poses.txt` 与录制话题为准。

---

### 3.5 问题汇总表

| 编号 | 严重级 | 位置 | 现象 | 根因 | 状态 |
|---|---|---|---|---|---|
| 3.1 | P0 | `FAST_LIO/src/preprocess.cpp` | 点云正常但 SLAM 不处理、内存暴涨 | MID-360 tag 分支被注释 | 已修复验证 |
| 3.2 | P0 | `STD/src/std.cpp` | 角点 1~5、描述子 0、回环 0 | NMS 半径读取错误键 | 已修复验证 |
| 3.3 | P1 | `laserMapping.cpp` / `loop_detection.cpp` / launch | 内存膨胀、回调乱序 | 队列过大、多线程回调 | 建议修复 |
| 3.4 | P1 | `loop_optimization_node.cpp` | 保存地图时 SIGSEGV | 未定位 | 未解决（可绕过） |

---

## 4. 与本仓库已有修复的关系

| 项目 | 本仓库 | 本次记录 |
|---|---|---|
| GTSAM 4.0.3 backup/recover 补丁 | 已修复 | 一致（同样需要源码级补丁） |
| STD 的 TBB 硬编码作者路径 | 已修复 | 一致（改为系统路径 + `IMPORTED_LOCATION`） |
| LTA-OM 无独立可执行文件（`#define as_node`） | 已修复（改为 CMake 目标级宏） | 未采用；本环境走 nodelet 加载路径 |
| `ROS_EDITION=ROS1` 分支（`livox_ros_driver2`） | 不适用 | 本环境需要（每次 `catkin_make -DROS_EDITION=ROS1`） |
| MID-360 tag 过滤（§3.1） | **未记录** | **建议采纳，直接解 M2** |
| NMS 键名错误（§3.2） | **未记录** | **建议采纳，M3 前置条件** |
| 队列与线程数（§3.3） | 未记录 | 建议采纳（健壮性） |

---

## 5. 验证方法与量化结果

### 5.1 复现步骤（三段式）

```bash
# 终端 1：nodelet 链路（FAST-LIO 前端 + STD 回环检测）
roslaunch loop_optimization run_all_<场景>.launch

# 终端 2：GTSAM 位姿图优化
rosrun loop_optimization loop_optimization_node

# 终端 3：数据回放（仅传感器话题）
rosbag play --clock -r 0.6 sensors_only.bag
```

**数据回放注意事项**

- 数据包需先过滤，仅保留激光与 IMU 话题；直接回放含 `/cloud_registered`、`/Odometry`、`/path` 的完整包会与在线节点抢话题，并触发 `loop back, clear buffer`；
- 禁止多个 `rosbag play` 并发（会造成时间戳倒退并清空缓冲）；
- 使用 `use_sim_time=true` 时，`/clock` 必须先于节点启动存在。

### 5.2 前端指标

| 指标 | 数值 |
|---|---|
| `/livox/lidar` | 9.8 Hz，4000 ~ 8000 点/帧 |
| `/livox/imu` | 124 Hz（静止时 `acc.z ≈ 9.80`） |
| `/aft_mapped_to_init` | 9 ~ 10 Hz |
| 静止漂移 | 5 s 约 1 cm |
| 轨迹精度 | 往返 8 m，估计峰值 8.33 m（幅度误差 4%），y/z 波动 ≤ 6 cm |

### 5.3 回环指标

| 指标 | 数值 |
|---|---|
| 关键帧数 | 24 |
| `corner size` / `descriptor size` | 60 / 371 ~ 881（每关键帧） |
| 检出回环 | 6 条以上，`icp` 0.474 ~ 0.715 |
| 优化后位姿修正量 | 最大 6.6 mm，平均 3.6 mm，末端 6.6 mm |

**回环日志样例**

```text
[Loop Sucess] 9--5,   candidate id:0, icp:0.497
[Loop Sucess] 13--2,  candidate id:2, icp:0.525
[Loop Sucess] 15--8,  candidate id:0, icp:0.576
[Loop Sucess] 18--13, candidate id:1, icp:0.474
[Loop Sucess] 23--13, candidate id:1, icp:0.715
```

**指标解读**：本次数据为 8 m 往返短轨迹，本身漂移仅约 0.23 m，因此回环修正量在毫米级。该结果可用于验证链路正确性，但**不足以体现回环收益**，需在更长轨迹上复测（见 §7）。

### 5.4 产物

```text
ltaom_logs/optimized_poses.txt      优化后关键帧位姿（含四元数）
ltaom_logs/odom_poses.txt           优化前关键帧位姿
ltaom_logs/lc_debug.txt             STD 检测日志（corner/descriptor/Loop 记录）
ltaom_logs/opt_debug.txt            GTSAM 优化日志
ltaom_logs/times_*_LTAOM.txt        各阶段耗时
```

---

## 6. 建议纳入本仓库的内容

| 内容 | 建议位置 |
|---|---|
| §3.1 / §3.2 两项源码修复 | `src/lio/FAST_LIO/src/preprocess.cpp`、`src/lio/STD/src/std.cpp` |
| §3.2 的配置补键 | `src/lio/STD/config/*.yaml` |
| §3.3 队列与线程数 | `src/lio/FAST_LIO/src/laserMapping.cpp`、`src/lio/STD/src/loop_detection.cpp`、`src/lio/loop_optimization/launch/run_all_*.launch` |
| 本文档 | `src/lio/README_loop_closure_notes.md`（与 `src/lio/README.md` 同级）；并建议将 §1 摘要并入根 `README.md` 的「踩坑记录」与「当前状态」表 |
| 数据回放注意事项（§5.1） | 根 `README.md` 的启动流程或 `tools/` |

---

## 7. 场景扩展建议（用于量化回环收益）

现有 8 m 往返数据无法体现回环修正效果。下一阶段建议在 `src/sim_env/worlds` 现有 world 基础上扩展场景，要点如下：

**几何**

- 主洞长度 50 ~ 60 m，矩形断面（宽 × 高约 4.5 m × 4.5 m）；
- 2 ~ 3 个支洞/岔口，形成可分叉的巡检路径；
- 路线设计为**环形**（主洞去、辅洞回），使回环成为定位的必要条件而非可选。

**特征可区分性**（针对隧道自相似导致的回环误检）

- 沿程布置差异化地物：岔口腔室、管廊/电缆桥架、检修门与格栅、不同断面段、散落障碍物；
- 目标：相邻但非同一位置的场景，其 STD 描述子具备足够区分度。

**验收指标**

| 指标 | 目标 |
|---|---|
| 无回环时的地图重影 | 明显可见（漂移累积到分米级） |
| 回环检测 | 真阳性可检出，且相邻关键帧之间不产生误检 |
| 修正前后对比 | 位姿修正量与地图一致性有明确改善 |

**建议同步调整的回环门槛**（抑制自相似场景误检）：`icp_threshold` 0.45 → 0.6，`similarity_threshold` 0.5 → 0.7。

---

## 8. 待办

- [ ] §3.1、§3.2 两项修复合入本仓库，并复测 M2 的 `/aft_mapped_to_init` 输出；
- [ ] §3.3 队列与线程数加固；
- [ ] §3.4 保存路径段错误的根因定位；
- [ ] 按 §7 扩展隧道场景并录制长轨迹数据；
- [ ] 在长轨迹上给出「修正前 / 修正后」的地图与轨迹对比，输出量化验收数据。
