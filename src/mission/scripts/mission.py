#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# mission: 航迹执行器。逐点发布 /iris_N/move_base_simple/goal（frame: map，ENU），
# 用 /iris_N/odom_world（lio_transfer 输出）判到达（< ~arrival_dist），全部完成后发 /iris_N/mission/done。
#
# 用法：
#   rosrun mission mission.py _waypoints:="[[0,8,1.5],[5,8,1.5],[5,12,1.5]]"
# 参数：
#   ~drone_id       (int, 默认 0)
#   ~waypoints      (string, JSON 风格列表 [[x,y,z(,yaw)],...]，map 系 ENU)
#   ~odom_topic     (默认 /iris_0/odom_world)
#   ~goal_topic     (默认 /iris_0/move_base_simple/goal)
#   ~arrival_dist   (默认 0.5 m)
#   ~timeout        (默认 60 墙钟秒 / 航点)
#   ~takeoff_wait   (默认 20 s，等待起飞悬停后再开始任务)

import ast
import math
import time
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from mavros_msgs.msg import State


class Mission(object):
    def __init__(self):
        drone_id = rospy.get_param("~drone_id", 0)
        self.ns = "iris_{}".format(drone_id)

        wp_str = rospy.get_param("~waypoints", "[]")
        self.waypoints = ast.literal_eval(wp_str) if isinstance(wp_str, str) else wp_str
        if not isinstance(self.waypoints, (list, tuple)):
            raise ValueError("waypoints must be a list")
        if not self.waypoints:
            rospy.logwarn("[%s] empty waypoint list", self.ns)

        self.arrival_dist = rospy.get_param("~arrival_dist", 0.5)
        self.timeout = rospy.get_param("~timeout", 60.0)
        self.takeoff_wait = rospy.get_param("~takeoff_wait", 20.0)

        odom_topic = rospy.get_param("~odom_topic", "/{}/odom_world".format(self.ns))
        # ego-planner 的 waypoint_generator 订阅全局 /move_base_simple/goal（在其命名空间内 remap 到 ~goal）
        goal_topic = rospy.get_param("~goal_topic", "/{}/move_base_simple/goal".format(self.ns))

        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        self.done_pub = rospy.Publisher("/{}/mission/done".format(self.ns), Bool, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=10)

        self.cur_pos = None
        self.odom_received = 0.0
        self.state = None
        self.state_sub = rospy.Subscriber("/{}/mavros/state".format(self.ns), State, self.state_cb, queue_size=1)
        min_z=rospy.get_param("/framework/min_flight_z", -1e6)
        max_z=rospy.get_param("/framework/max_flight_z", 1e6)
        for wp in self.waypoints:
            if len(wp) not in (3, 4) or not all(math.isfinite(float(v)) for v in wp):
                raise ValueError("Each waypoint must contain 3 or 4 finite numbers")
            if not min_z <= wp[2] <= max_z:
                raise ValueError("Waypoint height lies outside configured flight envelope")
        rospy.loginfo("[%s] mission loaded with %d waypoints: %s", self.ns, len(self.waypoints), self.waypoints)

    def odom_cb(self, msg):
        self.cur_pos = msg.pose.pose.position
        self.odom_received = time.monotonic()

    def state_cb(self, msg):
        self.state = msg

    def planner_enable(self, enabled):
        rospy.wait_for_service("/control_transfer/set_enabled", timeout=10)
        response = rospy.ServiceProxy("/control_transfer/set_enabled", SetBool)(enabled)
        if not response.success:
            raise RuntimeError(response.message)

    @staticmethod
    def dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def yaw_between(self, a, b):
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def run(self):
        if not self.waypoints:
            self.done_pub.publish(Bool(False))
            return False
        time.sleep(self.takeoff_wait)
        deadline = time.monotonic() + 60
        while not rospy.is_shutdown():
            if self.cur_pos is not None and time.monotonic()-self.odom_received < 2 and self.state and self.state.armed and self.state.mode == "OFFBOARD" and self.goal_pub.get_num_connections() > 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("Mission requires fresh odometry, armed OFFBOARD and goal subscriber")
            time.sleep(.1)
        self.planner_enable(True)

        for i, wp in enumerate(self.waypoints):
            if len(wp) >= 4:
                x, y, z, yaw = wp[0], wp[1], wp[2], wp[3]
            else:
                x, y, z = wp[0], wp[1], wp[2]
                yaw = 0.0
                # 面向下一航点
                if i + 1 < len(self.waypoints):
                    nxt = self.waypoints[i + 1]
                    yaw = self.yaw_between((x, y), (nxt[0], nxt[1]))

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z
            q = _q_from_yaw(yaw)
            goal.pose.orientation = q

            rospy.loginfo("[%s] sending waypoint %d/%d: (%.1f, %.1f, %.1f)",
                          self.ns, i + 1, len(self.waypoints), x, y, z)
            self.goal_pub.publish(goal)

            deadline = time.monotonic() + self.timeout
            arrived = False
            while not rospy.is_shutdown():
                if time.monotonic()-self.odom_received > 2:
                    raise RuntimeError("Mission odometry stale for >2 wall seconds")
                if not self.state.armed or self.state.mode != "OFFBOARD":
                    raise RuntimeError("Mission lost armed OFFBOARD")
                if self.cur_pos is not None and self.dist(self.cur_pos, (x, y, z)) < self.arrival_dist:
                    arrived = True
                    break
                if time.monotonic() > deadline:
                    break
                time.sleep(0.2)

            if arrived:
                rospy.loginfo("[%s] waypoint %d/%d reached", self.ns, i + 1, len(self.waypoints))
            else:
                rospy.logerr("[%s] waypoint %d/%d timeout; mission failed", self.ns, i + 1, len(self.waypoints))
                self.done_pub.publish(Bool(False))
                self.planner_enable(False)
                return False

        self.planner_enable(False)
        self.done_pub.publish(Bool(True))
        rospy.loginfo("[%s] mission finished", self.ns)
        return True


def _q_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


if __name__ == "__main__":
    rospy.init_node("mission")
    m = Mission()
    try:
        success = m.run()
    except Exception as error:
        rospy.logerr("Mission failed: %s", error)
        m.done_pub.publish(Bool(False))
        try: m.planner_enable(False)
        except Exception: pass
        success = False
    sys.exit(0 if success else 1)
