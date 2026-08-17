#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# control_transfer: 规划器 PositionCommand（ENU）-> 飞控控制指令。
#
# 两种输出模式（~output）：
#   xtdrone（默认）: Pose -> /xtdrone/iris_N/cmd_pose_enu，由 multirotor_communication.py
#                    转成 60Hz MAVROS setpoint 流（沿用 racer/XTDrone 已验证链路）。
#                    Pose.orientation.x = yaw, Pose.orientation.y = yaw_dot（XTDrone 约定）。
#   mavros:          PositionTarget -> /iris_N/mavros/setpoint_raw/local，本节点 30Hz
#                    缓存最近一帧连续重发（PX4 offboard 需要持续 setpoint）。
#
# 坐标转换（~enu_to_ned, 默认 true）：x->x, y->-y, z->-z, yaw->-yaw。
# 该约定在 M1 用真值一次性测量校准；若实际 SITL 无镜像则置 false 并记录。
# 保护：相邻指令位置跳变 > ~max_jump(5m) 时保持旧值并告警（防规划器异常输出）。

import rospy
from geometry_msgs.msg import Pose
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand


class ControlTransfer(object):
    def __init__(self):
        drone_id = rospy.get_param("~drone_id", 0)
        self.ns = "iris_{}".format(drone_id)
        self.enu_to_ned = rospy.get_param("~enu_to_ned", True)
        self.output = rospy.get_param("~output", "xtdrone")  # xtdrone | mavros
        self.max_jump = rospy.get_param("~max_jump", 5.0)

        if self.output == "xtdrone":
            self.pose_pub = rospy.Publisher("/xtdrone/{}/cmd_pose_enu".format(self.ns), Pose, queue_size=10)
        elif self.output == "mavros":
            self.target_pub = rospy.Publisher("/{}/mavros/setpoint_raw/local".format(self.ns), PositionTarget, queue_size=10)
            self.last_target = None
            self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.stream_cb)
        else:
            rospy.logerr("unknown ~output mode: %s", self.output)
            raise SystemExit(1)

        self.sub = rospy.Subscriber("/{}/planning/pos_cmd".format(self.ns), PositionCommand, self.cmd_cb, queue_size=10)
        self.last_pos = None
        rospy.loginfo("control_transfer started: ns=%s output=%s enu_to_ned=%s",
                      self.ns, self.output, self.enu_to_ned)

    def convert(self, x, y, z, yaw, yaw_dot):
        if self.enu_to_ned:
            return (x, -y, -z, -yaw, -yaw_dot)
        return (x, y, z, yaw, yaw_dot)

    def cmd_cb(self, msg):
        x, y, z, yaw, yaw_dot = self.convert(msg.position.x, msg.position.y,
                                             msg.position.z, msg.yaw, msg.yaw_dot)

        if self.last_pos is not None:
            d = ((x - self.last_pos[0]) ** 2 + (y - self.last_pos[1]) ** 2 + (z - self.last_pos[2]) ** 2) ** 0.5
            if d > self.max_jump:
                rospy.logwarn_throttle(1.0, "[%s] position command jump %.2f m > %.2f m, keeping last command",
                                       self.ns, d, self.max_jump)
                return
        self.last_pos = (x, y, z)

        if self.output == "xtdrone":
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.x = yaw      # XTDrone 约定：yaw 放 orientation.x
            p.orientation.y = yaw_dot  # XTDrone 约定：yaw_rate 放 orientation.y
            self.pose_pub.publish(p)
        else:  # mavros
            t = PositionTarget()
            t.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            t.type_mask = 0b001111000000  # 960: 用 position + yaw + yaw_rate，忽略 vel/accel/force
            t.position.x = x
            t.position.y = y
            t.position.z = z
            t.yaw = yaw
            t.yaw_rate = yaw_dot
            self.last_target = t

    def stream_cb(self, event):
        if self.last_target is not None:
            self.last_target.header.stamp = rospy.Time.now()
            self.target_pub.publish(self.last_target)


if __name__ == "__main__":
    rospy.init_node("control_transfer")
    node = ControlTransfer()
    rospy.spin()
