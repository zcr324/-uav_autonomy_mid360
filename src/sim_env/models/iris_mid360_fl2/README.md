# iris_mid360_fl2

用途：XTDrone / PX4 Iris + MID-360 + FAST-LIO2（Driver2）。

## 文件来源

- Iris 基础模型：
  `~/PX4_Firmware/Tools/sitl_gazebo/models/iris/iris.sdf`
- MID-360 基础模型：
  `~/fastlio_ws/src/livox_laser_simulation/urdf/Mid360.sdf`
- 雷达插件：
  `~/fastlio_ws/devel/lib/liblivox_laser_simulation.so`

原始 `iris` 和已有 `iris_mid360` 均未修改。

## ROS 接口

- `/livox/lidar`
  - 类型：`livox_ros_driver2/CustomMsg`
  - 频率：10 Hz
  - frame：`laser_livox`
- `/livox/imu`
  - 类型：`sensor_msgs/Imu`
  - 频率：200 Hz
  - frame：`livox_imu`

雷达与新增 ROS IMU 使用相同位姿，FAST-LIO2 初始外参可设为单位阵。

## 启动前环境

必须执行：

`source ~/fastlio_ws/devel/setup.bash`

## Local mesh assets

MID-360 visual and collision meshes are stored in `meshes/` and referenced through `model://iris_mid360_fl2/meshes/` for Gazebo Classic compatibility.
