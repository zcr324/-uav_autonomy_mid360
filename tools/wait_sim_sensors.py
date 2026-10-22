#!/usr/bin/env python3
"""Simulation startup: wait for a stationary, gravity-bearing IMU before LIO init."""
import math
import os
import sys
import time
import rospy
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

command = sys.argv[1:]
if not command:
    raise SystemExit("usage: wait_sim_sensors.py EXECUTABLE [ARGS...]")
rospy.init_node("wait_sim_sensors", anonymous=True, disable_signals=True)
state = {"since": None, "ready": False, "cloud": False}
def imu(msg):
    a=msg.linear_acceleration
    g=msg.angular_velocity
    stationary = abs(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)-9.81)<0.5 and math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z)<0.1
    stamp=msg.header.stamp.to_sec()
    if not stationary or (state["since"] is not None and stamp<state["since"]):
        state["since"]=None
    elif state["since"] is None:
        state["since"]=stamp
    else:
        state["ready"]=stamp-state["since"]>=1.0
def cloud(msg):
    state["cloud"]=len(msg.points)>100
rospy.Subscriber("/livox/imu",Imu,imu,queue_size=10)
rospy.Subscriber("/livox/lidar",CustomMsg,cloud,queue_size=1)
deadline=time.monotonic()+120
while not (state["ready"] and state["cloud"]):
    if rospy.is_shutdown() or time.monotonic()>deadline:
        raise SystemExit("Simulation sensor readiness timed out")
    time.sleep(.05)
rospy.loginfo("Simulation sensors stable; starting LIO")
rospy.signal_shutdown("launch child")
os.execv(command[0], command)
