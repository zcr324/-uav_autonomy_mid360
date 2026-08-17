#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# takeoff: 单机起飞序列（等价 racer hover.py，单机 + py3）。
# 时序：cmd_vel_flu 上推 1.0（触发 communication 切 OFFBOARD）-> OFFBOARD -> ARM -> HOVER。
# 依赖 multirotor_communication.py 已在运行（60Hz setpoint 流）。

import sys
import time

import rospy
import mavros_msgs.msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def _in_mode(ns, mode):
    """查询 /ns/mavros/state 的当前飞行模式（1s 超时，失败返回 False）"""
    try:
        state = rospy.wait_for_message("/{}/mavros/state".format(ns), mavros_msgs.msg.State, timeout=1.0)
        return state.mode == mode
    except rospy.ROSException:
        return False


def main():
    vehicle_type = sys.argv[1] if len(sys.argv) > 1 else "iris"
    vehicle_id = sys.argv[2] if len(sys.argv) > 2 else "0"

    rospy.init_node("takeoff")

    ns = "{}_{}".format(vehicle_type, vehicle_id)
    cmd_pub = rospy.Publisher("/xtdrone/{}/cmd".format(ns), String, queue_size=10)
    vel_pub = rospy.Publisher("/xtdrone/{}/cmd_vel_flu".format(ns), Twist, queue_size=10)

    # 等 MAVROS 就绪（心跳 + 可用的模式/解锁服务），否则一次性序列会打在空处
    rospy.loginfo("[%s] waiting for mavros connection...", ns)
    try:
        rospy.wait_for_message("/{}/mavros/state".format(ns), mavros_msgs.msg.State, timeout=120)
    except rospy.ROSException:
        rospy.logerr("[%s] mavros state timeout, abort", ns)
        return
    rospy.loginfo("[%s] mavros connected, waiting 3s for setpoint stream...", ns)
    rospy.sleep(3.0)

    twist = Twist()

    # 1. 上推 1.0 m/s（communication 检测非零速度 -> OFFBOARD）
    twist.linear.z = 1.0
    rospy.sleep(1.0)
    for _ in range(10):
        vel_pub.publish(twist)
        rospy.sleep(0.1)

    # 2. OFFBOARD（失败重试，PX4 需要足够的 setpoint 流才接受切换）
    for attempt in range(5):
        cmd_pub.publish(String("OFFBOARD"))
        rospy.loginfo("[%s] OFFBOARD (attempt %d)", ns, attempt + 1)
        rospy.sleep(2.0)
        if _in_mode(ns, "OFFBOARD"):
            break
    else:
        rospy.logwarn("[%s] failed to enter OFFBOARD after retries, continuing anyway", ns)

    # 3. ARM
    cmd_pub.publish(String("ARM"))
    rospy.loginfo("[%s] ARM", ns)
    rospy.sleep(2.0)

    # 4. HOVER（速度归零，communication 锁定当前位置悬停）
    twist.linear.z = 0.0
    vel_pub.publish(twist)
    cmd_pub.publish(String("HOVER"))
    rospy.loginfo("[%s] HOVER", ns)

    # 保持节点存活，期间 communication 持续 60Hz 悬停 setpoint
    rospy.spin()


if __name__ == "__main__":
    main()
