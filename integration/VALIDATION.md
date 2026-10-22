# 本机验证记录
本机 ROS Noetic / Gazebo 11 / MAVROS 1.20.1，WSL2 内存约 8 GiB。

|检查|结果|证据|
|---|---|---|
|原交付经典 FAST-LIO 干净编译|补消息目标依赖后通过|logs/baseline_build_retry.log|
|无门控同步启动|严重定位发散|logs/baseline_metrics.json|
|落稳后重启前端|消除严重发散，静止仍漂移|logs/baseline_settled_metrics.json|
|原交付原始传感器|LiDAR 约10 Hz、IMU约249.5 Hz（仿真时钟）；存在约999米异常点|logs/sensor_bag_metrics.json|
|射线长度修复|距离过滤恢复，但发现视场索引符号错误|logs/baseline_fixed_metrics.json|
|坐标/安装/控制数值测试|3项通过|tools/test_frame_contracts.py|
|XML及Python语法|通过|工具执行记录|
|原始交付内容哈希|未变化|original_sources_sha256.json|
|规划包本地头文件引用检查|没有发现缺失|静态扫描，非全量文件完整性证明|

后续运行结果继续追加。logs/ 是实际测试产物，不纳入 Git。
world 的 map 对齐采用声明的起始机体位置参数；模型落地后与 spawn 高度不同，应以起飞前标定的初始机体位置设定 map 对齐。

## 完整新入口的失败结果
最终移除无命中射线的浮点量程边界点后，tunnel_complex.world 的 (0,0) 入口测试中 LTA-OM 虽保持约10 Hz，但静止估计沿 x 漂移超过160 m。
该测试明确失败，不得用此前含虚假量程边界点的低漂移结果作为最终精度证据。
定位与 PX4 不一致，控制/解锁/起飞未启用。正在用非退化场景区分场景与算法接入问题。
前端 TF/odom child 已确认是最终版本 lio_body。

## 最终整合版结果
- 14 个 catkin 包构建通过，构建脚本退出0：logs/integration_verified_build.log。
- 坐标/安装/ENU指令/map到PX4原点对齐：4项数值测试通过。
- structured_px4.world、(20,4) 起点：LiDAR/LTA-OM/桥接约10 Hz（仿真时钟），IMU约206.6 Hz；约30秒静止位移0.186 m。属于基本稳定性结果，精度仍待验收。
- 短链路起飞：进入OFFBOARD并armed=true；1 m起飞目标达到（logs/takeoff_metrics.json）。
- 规划执行：目标(20.70665,4.11681,1.2)，共接收237条map指令；最后指令高度1.19915 m，已保留目标高度。
- 到达：LIO map下误差0.18355 m，小于本测试0.2 m阈值；PX4本地x从-0.01511移动到0.33218 m。
- 结束：AUTO.LAND且armed=false，随后清理本轮仿真进程。
- STD：两个运动关键帧，13个角点，211/203个描述子；没有真实回环（loopclosure.txt为空）。
- PGO：收到/aft_pgo_path，2个位姿，生成optimized_poses.txt。没有回环约束，因此不能作为回环修正收益证据。
- PX4实际配置：EKF2_AID_MASK=1（GPS）、EKF2_HGT_MODE=0。当前飞行测试是GPS辅助，尚未完成无GPS外部定位融合验收。
- map/控制原点用估计位姿做固定对齐，桥接没有Gazebo真值订阅。

## 后续验收顺序
1. 长直隧道退化定位与静止漂移评估，区分场景、前端参数和仿真质量。
2. STD静止子图积累、内部缓冲和长时间运行内存加固。
3. 无GPS外部定位融合配置与悬停验收。
4. 多航点和障碍物绕行、失效与恢复测试。
5. 录制长闭环轨迹，验证真回环、误检和修正收益。
6. 定义连续控制坐标与PGO全局坐标的修正关系，再接入在线回环反馈。
