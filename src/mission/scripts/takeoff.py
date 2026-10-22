#!/usr/bin/env python3
"""Take off using an ENU position target, and verify mode, arming and height."""
import math
import sys
import time
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import SetBool

def main():
    vehicle = sys.argv[1] if len(sys.argv)>1 else "iris"
    ident = sys.argv[2] if len(sys.argv)>2 else "0"
    rospy.init_node("takeoff")
    ns = "{}_{}".format(vehicle,ident)
    height = float(rospy.get_param("~height",1.5))
    timeout = float(rospy.get_param("~timeout",90.0))
    state = {"state":None,"pose":None}
    rospy.Subscriber("/{}/mavros/state".format(ns),State,lambda m:state.update(state=m),queue_size=1)
    rospy.Subscriber("/{}/mavros/local_position/pose".format(ns),PoseStamped,lambda m:state.update(pose=m),queue_size=1)
    commands=rospy.Publisher("/xtdrone/{}/cmd".format(ns),String,queue_size=2)
    targets=rospy.Publisher("/xtdrone/{}/cmd_pose_enu".format(ns),Pose,queue_size=2)
    # Acquire takeoff ownership before producing any motion command.
    rospy.wait_for_service("/control_transfer/set_enabled", timeout=10)
    response=rospy.ServiceProxy("/control_transfer/set_enabled",SetBool)(False)
    if not response.success:raise RuntimeError("Could not disable planner control: "+response.message)
    deadline=time.monotonic()+timeout
    while not rospy.is_shutdown():
        if state["state"] and state["state"].connected and state["pose"] and targets.get_num_connections()>0:
            break
        if time.monotonic()>deadline:raise RuntimeError("MAVROS/local pose/communication readiness timeout")
        time.sleep(.05)
    pose=state["pose"].pose
    target=Pose()
    target.position.x,target.position.y,target.position.z=pose.position.x,pose.position.y,pose.position.z
    target.orientation.x=tft.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])[2]
    # Prime a hold target before OFFBOARD and arming.
    end=time.monotonic()+3
    while time.monotonic()<end:
        targets.publish(target);time.sleep(.05)
    for request,predicate in [
        ("OFFBOARD",lambda:state["state"].mode=="OFFBOARD"),
        ("ARM",lambda:state["state"].armed)]:
        last_request=0
        while not rospy.is_shutdown() and not predicate():
            targets.publish(target)
            if time.monotonic()-last_request>2:
                commands.publish(String(data=request));last_request=time.monotonic()
            if time.monotonic()>deadline:raise RuntimeError(request+" not acknowledged")
            time.sleep(.05)
    target.position.z += height
    while not rospy.is_shutdown():
        targets.publish(target)
        if state["state"].mode!="OFFBOARD" or not state["state"].armed:
            raise RuntimeError("Lost OFFBOARD/arming during takeoff")
        if abs(state["pose"].pose.position.z-target.position.z)<.2:
            rospy.loginfo("Takeoff reached %.2f m; communication holds target",target.position.z)
            return
        if time.monotonic()>deadline:raise RuntimeError("Takeoff height timeout")
        time.sleep(.05)

if __name__=="__main__":
    main()
