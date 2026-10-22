#!/usr/bin/env python3
"""Focused numeric tests for frame alignment and the MAVROS command contract."""
import importlib.util
import math
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rospy
import tf.transformations as tft
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_srvs.srv import SetBoolRequest, TriggerRequest

ROOT = Path(__file__).resolve().parents[1]
def module(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

lio = module("lio_transfer", "src/bridge/scripts/lio_transfer.py")
control = module("control_transfer", "src/bridge/scripts/control_transfer.py")

class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg)

class FrameContractTests(unittest.TestCase):
    def setUp(self):
        self.params = {"~init_x": 3., "~init_y": 4., "~init_z": .5, "~init_yaw": math.pi/2}
        self.mocks = [
            patch.object(rospy, "get_param", side_effect=lambda name, default=None: self.params.get(name, default)),
            patch.object(rospy, "Publisher", Publisher),
            patch.object(rospy, "Subscriber"),
            patch.object(rospy, "Service"),
            patch.object(rospy, "set_param"),
            patch.object(rospy, "is_shutdown", return_value=False),
        ]
        for item in self.mocks: item.start()
        self.addCleanup(lambda: [item.stop() for item in reversed(self.mocks)])

    def odom(self, x, time):
        msg = Odometry()
        msg.header.stamp = rospy.Time.from_sec(time)
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1
        return msg

    def test_mount_alignment_velocity_and_cloud(self):
        node = lio.LioTransfer()
        first = self.odom(0, 1)
        node.odom_cb(first)
        out = node.odom_pub.messages[-1]
        np.testing.assert_allclose([out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z], [3,4,.5])
        node.odom_cb(self.odom(.1, 1.1))
        out = node.odom_pub.messages[-1]
        np.testing.assert_allclose([out.pose.pose.position.x, out.pose.pose.position.y], [3,4.1])
        np.testing.assert_allclose([out.twist.twist.linear.x, out.twist.twist.linear.y], [1,0], atol=1e-9)
        cloud = pc2.create_cloud_xyz32(Header(stamp=rospy.Time(1), frame_id="camera_init"), [(1,0,0)])
        node.cloud_cb(cloud)
        point = next(pc2.read_points(node.cloud_pub.messages[-1]))
        np.testing.assert_allclose(point, [2.995,4.995,.647], atol=1e-6)
        self.assertEqual(cloud.header.frame_id, "camera_init")

    def test_relocation_does_not_mix_pose_streams(self):
        self.params["~use_reloc"] = True
        with self.assertRaises(ValueError): lio.LioTransfer()

    def test_global_map_to_px4_local_origin(self):
        self.params.update({"~output": "mavros", "~auto_align": True})
        with patch.object(rospy, "Timer"), patch.object(rospy, "loginfo"):
            node = control.ControlTransfer()
            mapped = self.odom(20, 1)
            mapped.pose.pose.position.y = 4
            mapped.pose.pose.position.z = .055
            local = PoseStamped()
            local.header.stamp = rospy.Time(1)
            local.pose.orientation.w = 1
            local.pose.position.z = -.17
            node.map_cb(mapped)
            node.local_cb(local)
            node.state_cb(State(connected=True, armed=False))
        self.assertTrue(node.aligned)
        np.testing.assert_allclose(node.convert(21,4,1.055,0,0)[:3], [1,0,.83], atol=1e-9)

    def test_command_is_enu_with_yaw_and_yaw_rate(self):
        self.params.update({"~output": "mavros", "~enu_to_ned": False, "~auto_align": False})
        with patch.object(rospy, "Timer"):
            node = control.ControlTransfer()
        node.set_enabled(SetBoolRequest(True))
        cmd = PositionCommand()
        cmd.header.frame_id = "map"
        cmd.position.x, cmd.position.y, cmd.position.z = 2,3,1
        cmd.yaw, cmd.yaw_dot = .3,.1
        node.cmd_cb(cmd)
        out = node.last_target
        self.assertEqual([out.position.x,out.position.y,out.position.z], [2,3,1])
        self.assertAlmostEqual(out.yaw,.3)
        self.assertEqual(out.type_mask,504)

    def test_takeoff_ownership_blocks_old_trajectory(self):
        self.params.update({"~output": "mavros", "~auto_align": False})
        with patch.object(rospy, "Timer"):
            node = control.ControlTransfer()
        cmd = PositionCommand()
        cmd.header.frame_id = "map"
        cmd.trajectory_id = 7
        cmd.position.z = 1
        node.cmd_cb(cmd)
        self.assertIsNone(node.last_target)
        node.set_enabled(SetBoolRequest(True))
        node.cmd_cb(cmd)
        self.assertIsNone(node.last_target)
        cmd.trajectory_id = 8
        node.cmd_cb(cmd)
        self.assertIsNotNone(node.last_target)
        node.set_enabled(SetBoolRequest(False))
        self.assertIsNone(node.last_target)

    def test_height_envelope_rejects_unsafe_planner_target(self):
        self.params.update({"~output":"mavros", "~auto_align":False,
                            "/framework/min_flight_z":.6,"/framework/max_flight_z":1.6})
        with patch.object(rospy, "Timer"):
            node = control.ControlTransfer()
        node.set_enabled(SetBoolRequest(True))
        cmd = PositionCommand()
        cmd.header.frame_id = "map"
        cmd.position.z = .25
        with patch.object(rospy, "logerr_throttle"):
            node.cmd_cb(cmd)
        self.assertIsNone(node.last_target)
        cmd.position.z = 1.0
        node.cmd_cb(cmd)
        self.assertIsNotNone(node.last_target)

    def test_alignment_cannot_reset_while_armed(self):
        self.params.update({"~output": "mavros", "~auto_align": True})
        with patch.object(rospy, "Timer"):
            node = control.ControlTransfer()
        node.state = State(connected=True, armed=True)
        self.assertFalse(node.reset_alignment(TriggerRequest()).success)
        node.state.armed = False
        node.enabled = False
        self.assertTrue(node.reset_alignment(TriggerRequest()).success)
        self.assertFalse(node.aligned)

    def test_invalid_frame_or_number_cannot_replace_target(self):
        self.params.update({"~output": "mavros", "~auto_align": False})
        with patch.object(rospy, "Timer"):
            node = control.ControlTransfer()
        node.set_enabled(SetBoolRequest(True))
        cmd = PositionCommand()
        cmd.header.frame_id = "odom"
        with patch.object(rospy, "logerr_throttle"):
            node.cmd_cb(cmd)
            self.assertIsNone(node.last_target)
            cmd.header.frame_id = "map"
            cmd.position.x = float("nan")
            node.cmd_cb(cmd)
            self.assertIsNone(node.last_target)

if __name__ == "__main__":
    unittest.main()
