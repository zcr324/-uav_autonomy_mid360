#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# lio_transfer: 把 LIO 里程计（camera_init 系）转换到全局 map 系（map ≡ world，ENU，z 向上）并发布。
#
# 与 racer 版 lio_transfer_new.py 的区别（也是本节点的存在意义）：
#   - 不订阅 /gazebo/model_states，不做任何真值覆盖；
#   - 对齐来自启动参数的一次性"手起对准"（spawn 位姿 + 初始 yaw），等价于真机起飞前对准；
#   - 单机版，无 BIAS 点云平移、无其它机点剔除。
#
# 发布：
#   /iris_N/mavros/vision_pose/pose   PoseStamped  (frame: map)  -> PX4 视觉外部位姿
#   /iris_N/odom_world                Odometry      (frame: map)  -> 规划器
#   /iris_N/map/cloud_registered      PointCloud2   (frame: map)  -> 规划器局部地图

import math

import rospy
import tf.transformations as tft
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class LioTransfer(object):
    def __init__(self):
        drone_id = rospy.get_param("~drone_id", 0)
        self.ns = "iris_{}".format(drone_id)

        # 一次性手起对准参数（阶段 A；PGO 前）
        self.init_x = rospy.get_param("~init_x", 0.0)
        self.init_y = rospy.get_param("~init_y", 7.5)
        self.init_z = rospy.get_param("~init_z", 0.5)
        init_yaw = rospy.get_param("~init_yaw", 0.0)
        self.cos_yaw = math.cos(init_yaw)
        self.sin_yaw = math.sin(init_yaw)

        # 阶段 B：重定位模式（PGO 后，用 LTA-OM reloc 输出替换常量对齐）
        self.use_reloc = rospy.get_param("~use_reloc", False)
        self.reloc_topic = rospy.get_param("~reloc_topic", "/{}/lidar_odom_to_init".format(self.ns))

        odom_in = rospy.get_param("~odom_in_topic", "/{}/aft_mapped_to_init".format(self.ns))
        cloud_in = rospy.get_param("~cloud_in_topic", "/{}/cloud_registered".format(self.ns))

        self.vision_pub = rospy.Publisher("/{}/mavros/vision_pose/pose".format(self.ns), PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/{}/odom_world".format(self.ns), Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher("/{}/map/cloud_registered".format(self.ns), PointCloud2, queue_size=10)

        self.sub_odom = rospy.Subscriber(odom_in, Odometry, self.odom_cb, queue_size=10)
        self.sub_cloud = rospy.Subscriber(cloud_in, PointCloud2, self.cloud_cb, queue_size=10)
        if self.use_reloc:
            self.sub_reloc = rospy.Subscriber(self.reloc_topic, Odometry, self.reloc_cb, queue_size=10)

        self.last_odom_time = None
        rospy.loginfo("lio_transfer started: ns=%s use_reloc=%s init=(%f,%f,%f) yaw=%f",
                      self.ns, self.use_reloc, self.init_x, self.init_y, self.init_z, init_yaw)

    # ---------- helpers ----------

    def check_rate(self):
        now = rospy.Time.now()
        if self.last_odom_time is not None and (now - self.last_odom_time).to_sec() > 0.2:
            rospy.logwarn_throttle(5.0, "[%s] LIO odometry rate < 5 Hz (dt=%.2fs)",
                                   self.ns, (now - self.last_odom_time).to_sec())
        self.last_odom_time = now

    @staticmethod
    def rotate_yaw(x, y, cos_yaw, sin_yaw):
        return (x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw)

    def enu_to_map(self, x, y, z):
        rx, ry = self.rotate_yaw(x, y, self.cos_yaw, self.sin_yaw)
        return (rx + self.init_x, ry + self.init_y, z + self.init_z)

    # ---------- callbacks ----------

    def odom_cb(self, msg):
        self.check_rate()

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = msg.child_frame_id

        p = msg.pose.pose
        x, y, z = self.enu_to_map(p.position.x, p.position.y, p.position.z)
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        # 旋转：R_map = R_yaw * R_odom
        q_odom = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        q_yaw = tft.quaternion_from_euler(0, 0, math.atan2(self.sin_yaw, self.cos_yaw))
        q_map = tft.quaternion_multiply(q_yaw, q_odom)
        out.pose.pose.orientation.x = q_map[0]
        out.pose.pose.orientation.y = q_map[1]
        out.pose.pose.orientation.z = q_map[2]
        out.pose.pose.orientation.w = q_map[3]

        v = msg.twist.twist.linear
        vx, vy = self.rotate_yaw(v.x, v.y, self.cos_yaw, self.sin_yaw)
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.linear.z = v.z
        w = msg.twist.twist.angular
        wx, wy = self.rotate_yaw(w.x, w.y, self.cos_yaw, self.sin_yaw)
        out.twist.twist.angular.x = wx
        out.twist.twist.angular.y = wy
        out.twist.twist.angular.z = w.z

        self.odom_pub.publish(out)

        # vision_pose（PX4 视觉外部位姿，ENU，frame map）
        vis = PoseStamped()
        vis.header = out.header
        vis.pose = out.pose.pose
        self.vision_pub.publish(vis)

    def reloc_cb(self, msg):
        """阶段 B：重定位输出直接视为 map 系（LTA-OM reloc 语义，M4 用真值一次性验证）。"""
        self.check_rate()
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        self.odom_pub.publish(out)

        vis = PoseStamped()
        vis.header = out.header
        vis.pose = out.pose.pose
        self.vision_pub.publish(vis)

    def cloud_cb(self, msg):
        fields = msg.fields
        names = [f.name for f in fields]
        try:
            xi, yi, zi = names.index("x"), names.index("y"), names.index("z")
        except ValueError:
            rospy.logwarn_throttle(10.0, "[%s] cloud has no x/y/z fields: %s", self.ns, names)
            return

        new_points = []
        for p in pc2.read_points(msg, skip_nans=True):
            pl = list(p)
            pl[xi], pl[yi], pl[zi] = self.enu_to_map(pl[xi], pl[yi], pl[zi])
            new_points.append(tuple(pl))

        header = msg.header
        header.frame_id = "map"
        out = pc2.create_cloud(header, fields, new_points)
        self.cloud_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("lio_transfer")
    node = LioTransfer()
    rospy.spin()
