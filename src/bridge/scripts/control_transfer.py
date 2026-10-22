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
# MAVROS ROS input is ENU; MAVROS performs the ENU/NED conversion.
# ROS ENU 数值保持不变；enu_to_ned=true 会拒绝启动，避免重复转换。
# 保护：相邻指令位置跳变 > ~max_jump(5m) 时保持旧值并告警（防规划器异常输出）。

import math
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class ControlTransfer(object):
    def __init__(self):
        drone_id = rospy.get_param("~drone_id", 0)
        self.ns = "iris_{}".format(drone_id)
        self.enu_to_ned = rospy.get_param("~enu_to_ned", False)
        if self.enu_to_ned:
            raise ValueError("MAVROS converts ENU to NED; extra conversion is unsupported")
        self.output = rospy.get_param("~output", "xtdrone")  # xtdrone | mavros
        self.max_jump = rospy.get_param("~max_jump", 5.0)
        self.min_flight_z = rospy.get_param("/framework/min_flight_z", -1e6)
        self.max_flight_z = rospy.get_param("/framework/max_flight_z", 1e6)
        self.auto_align = rospy.get_param("~auto_align", True)
        self.aligned = not self.auto_align
        self.local_pose = None
        self.map_pose = None
        self.state = None
        self.align_yaw = 0.0
        self.align_translation = (0.0, 0.0, 0.0)
        if self.auto_align:
            rospy.Subscriber("/{}/mavros/local_position/pose".format(self.ns), PoseStamped, self.local_cb, queue_size=1)
            rospy.Subscriber("/{}/odom_world".format(self.ns), Odometry, self.map_cb, queue_size=1)
            rospy.Subscriber("/{}/mavros/state".format(self.ns), State, self.state_cb, queue_size=1)


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
        # Explicit command ownership: takeoff/landing and the planner must not
        # race as publishers of the same downstream position setpoint.
        self.enabled = rospy.get_param("~enabled", False)
        self.latest_trajectory_id = None
        self.await_new_trajectory = False
        self.blocked_trajectory_id = None
        self.enable_service = rospy.Service("~set_enabled", SetBool, self.set_enabled)
        self.reset_service = rospy.Service("~reset_alignment", Trigger, self.reset_alignment)
        rospy.set_param("~enabled", self.enabled)
        rospy.loginfo("control_transfer started: ns=%s output=%s enu_to_ned=%s",
                      self.ns, self.output, self.enu_to_ned)

    def set_enabled(self, request):
        if request.data and not self.aligned:
            return SetBoolResponse(False, "Map/PX4 alignment is not ready")
        self.enabled = request.data
        self.blocked_trajectory_id = self.latest_trajectory_id
        self.await_new_trajectory = request.data and self.latest_trajectory_id is not None
        self.last_pos = None
        if self.output == "mavros":
            self.last_target = None
        rospy.set_param("~enabled", self.enabled)
        return SetBoolResponse(True, "planner forwarding " + ("enabled" if self.enabled else "disabled"))

    def reset_alignment(self, request):
        if self.enabled or self.state is None or self.state.armed:
            return TriggerResponse(False, "Alignment reset requires disabled planner and disarmed vehicle")
        self.aligned = not self.auto_align
        self.local_pose = None
        self.map_pose = None
        rospy.set_param("~aligned", self.aligned)
        return TriggerResponse(True, "Waiting for fresh disarmed pose pair")

    def local_cb(self, msg):
        self.local_pose = msg
        self.try_align()

    def map_cb(self, msg):
        self.map_pose = msg
        self.try_align()

    def state_cb(self, msg):
        self.state = msg
        self.try_align()

    @staticmethod
    def yaw_of(q):
        return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def try_align(self):
        if self.aligned or self.local_pose is None or self.map_pose is None or self.state is None:
            return
        if not self.state.connected or self.state.armed:
            return
        if abs((self.local_pose.header.stamp - self.map_pose.header.stamp).to_sec()) > 0.25:
            return
        local = self.local_pose.pose
        mapped = self.map_pose.pose.pose
        self.align_yaw = self.yaw_of(local.orientation) - self.yaw_of(mapped.orientation)
        c, s = math.cos(self.align_yaw), math.sin(self.align_yaw)
        p, q = local.position, mapped.position
        self.align_translation = (p.x - c*q.x + s*q.y, p.y - s*q.x - c*q.y, p.z - q.z)
        self.aligned = True
        rospy.set_param("~aligned", True)
        rospy.set_param("~alignment", {"yaw":self.align_yaw,"translation":list(self.align_translation)})
        rospy.loginfo("Fixed map -> PX4 local alignment: yaw=%.4f translation=%s",
                      self.align_yaw, self.align_translation)

    def convert(self, x, y, z, yaw, yaw_dot):
        if self.enu_to_ned:
            raise ValueError("Double ENU/NED conversion is unsupported")
        c, s = math.cos(self.align_yaw), math.sin(self.align_yaw)
        tx, ty, tz = self.align_translation
        return (c*x-s*y+tx, s*x+c*y+ty, z+tz, yaw+self.align_yaw, yaw_dot)

    def cmd_cb(self, msg):
        if rospy.is_shutdown():
            return
        self.latest_trajectory_id = msg.trajectory_id
        if not self.enabled:
            return
        if self.await_new_trajectory:
            if msg.trajectory_id == self.blocked_trajectory_id:
                return
            self.await_new_trajectory = False
        values = (msg.position.x, msg.position.y, msg.position.z, msg.yaw, msg.yaw_dot)
        if not all(math.isfinite(value) for value in values) or msg.header.frame_id != "map":
            rospy.logerr_throttle(2, "Rejected non-finite or non-map planner command")
            return
        if not self.min_flight_z <= msg.position.z <= self.max_flight_z:
            rospy.logerr_throttle(2, "Rejected planner command outside configured flight height envelope")
            return
        if not self.aligned:
            rospy.logwarn_throttle(5, "Waiting for disarmed map/PX4 local frame alignment")
            return
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
            try:
                self.pose_pub.publish(p)
            except rospy.ROSException:
                if not rospy.is_shutdown():
                    raise
        else:  # mavros
            t = PositionTarget()
            t.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            t.type_mask = 504  # ignore velocity and acceleration; keep position/yaw/yaw_rate
            t.position.x = x
            t.position.y = y
            t.position.z = z
            t.yaw = yaw
            t.yaw_rate = yaw_dot
            self.last_target = t

    def stream_cb(self, event):
        if self.last_target is not None:
            self.last_target.header.stamp = rospy.Time.now()
            try:
                self.target_pub.publish(self.last_target)
            except rospy.ROSException:
                if not rospy.is_shutdown():
                    raise


if __name__ == "__main__":
    rospy.init_node("control_transfer")
    node = ControlTransfer()
    rospy.spin()
