#!/usr/bin/env python3
"""Align continuous LIO pose/cloud to map and compensate simulated IMU mounting."""
import math
import numpy as np
import rospy
import tf.transformations as tft
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class LioTransfer:
    def __init__(self):
        self.ns = "iris_{}".format(rospy.get_param("~drone_id", 0))
        if rospy.get_param("~use_reloc", False):
            raise ValueError("Online PGO alignment is not implemented. Keep continuous LIO for control; inspect /aft_pgo_path separately.")
        yaw = rospy.get_param("~init_yaw", 0.0)
        self.q_align = tft.quaternion_from_euler(0, 0, yaw)
        self.rotation = tft.quaternion_matrix(self.q_align)[:3, :3]
        # SDF: base_link -> livox_base (0,0,0.10), sensor (-.005,.005,.047).
        self.imu_in_base = np.array(rospy.get_param("~imu_in_base", [-0.005, 0.005, 0.147]), dtype=float)
        base_start = np.array([rospy.get_param("~init_x", 0.0), rospy.get_param("~init_y", 0.0),
                               rospy.get_param("~init_z", 0.055)])
        self.translation = base_start + self.rotation.dot(self.imu_in_base)
        self.previous = None
        self.vision_pub = rospy.Publisher("/{}/mavros/vision_pose/pose".format(self.ns), PoseStamped, queue_size=2)
        self.odom_pub = rospy.Publisher("/{}/odom_world".format(self.ns), Odometry, queue_size=2)
        self.cloud_pub = rospy.Publisher("/{}/map/cloud_registered".format(self.ns), PointCloud2, queue_size=2)
        rospy.Subscriber(rospy.get_param("~odom_in_topic", "/aft_mapped_to_init"), Odometry, self.odom_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param("~cloud_in_topic", "/cloud_registered"), PointCloud2, self.cloud_cb, queue_size=2)

    def odom_cb(self, msg):
        pose = msg.pose.pose
        q = tft.quaternion_multiply(self.q_align,
              [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        position = self.rotation.dot([pose.position.x, pose.position.y, pose.position.z]) + self.translation
        # Output is base_link position, not the displaced IMU origin.
        position -= tft.quaternion_matrix(q)[:3, :3].dot(self.imu_in_base)
        out = Odometry()
        out.header = Header(stamp=msg.header.stamp, frame_id="map")
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = q
        # LTA-OM vendor odometry does not populate twist. Differentiate aligned base position.
        if self.previous is not None:
            stamp, previous_position = self.previous
            dt = (msg.header.stamp - stamp).to_sec()
            if 0 < dt <= 0.5:
                velocity_map = (position - previous_position) / dt
                # nav_msgs/Odometry twist is expressed in child_frame_id.
                velocity_body = tft.quaternion_matrix(q)[:3, :3].T.dot(velocity_map)
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity_body
            else:
                rospy.logwarn_throttle(5, "LIO timestamp discontinuity; reset velocity estimate")
        self.previous = (msg.header.stamp, position.copy())
        self.odom_pub.publish(out)
        vis = PoseStamped(header=out.header, pose=out.pose.pose)
        self.vision_pub.publish(vis)

    def cloud_cb(self, msg):
        names = [field.name for field in msg.fields]
        if not all(name in names for name in ("x", "y", "z")):
            return
        xi, yi, zi = [names.index(name) for name in ("x", "y", "z")]
        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            values = list(point)
            values[xi], values[yi], values[zi] = self.rotation.dot([values[xi], values[yi], values[zi]]) + self.translation
            points.append(values)
        header = Header(stamp=msg.header.stamp, frame_id="map")
        self.cloud_pub.publish(pc2.create_cloud(header, msg.fields, points))

if __name__ == "__main__":
    rospy.init_node("lio_transfer")
    LioTransfer()
    rospy.spin()
