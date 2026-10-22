#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/zcr/uav_ws/integration/baseline_ws/devel/setup.bash
source /home/zcr/PX4_Firmware/Tools/setup_gazebo.bash /home/zcr/PX4_Firmware /home/zcr/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="/home/zcr/PX4_Firmware:$ROS_PACKAGE_PATH"
export GAZEBO_MODEL_PATH="/home/zcr/uav_ws/integration/baseline_ws/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="/home/zcr/uav_ws/integration/baseline_ws/devel/lib:$GAZEBO_PLUGIN_PATH"
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_MASTER_URI=http://localhost:11321
export GAZEBO_MASTER_URI=http://localhost:11355
export ROS_LOG_DIR=/home/zcr/uav_ws/integration/logs/ros_baseline
exec roslaunch -p 11321 /home/zcr/uav_ws/integration/baseline.launch gui:=false
