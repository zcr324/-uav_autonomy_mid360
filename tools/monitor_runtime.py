#!/usr/bin/env python3
import argparse, collections, json, math, time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg
from mavros_msgs.msg import State
p=argparse.ArgumentParser()
p.add_argument("--odom",default="/Odometry")
p.add_argument("--duration",type=float,default=45)
p.add_argument("--output",required=True)
args=p.parse_args()
rospy.init_node("integration_monitor",anonymous=True,disable_signals=True)
samples=collections.defaultdict(list)
latest={}
stamps=collections.defaultdict(list)
positions=collections.defaultdict(list)
def callback(name):
 def cb(msg):
  samples[name].append(time.monotonic())
  if hasattr(msg,"header"): stamps[name].append(msg.header.stamp.to_sec())
  if isinstance(msg,CustomMsg):
   latest[name]={"point_num":msg.point_num,"points_size":len(msg.points),"lines":sorted(set(x.line for x in msg.points)),
                 "offset_max":max((x.offset_time for x in msg.points),default=0),
                 "range_max":max((math.sqrt(x.x*x.x+x.y*x.y+x.z*x.z) for x in msg.points),default=0)}
  elif isinstance(msg,Odometry):
   v=msg.pose.pose.position
   positions[name].append([v.x,v.y,v.z])
   latest[name]={"position":[v.x,v.y,v.z],"frame":msg.header.frame_id,"child_frame":msg.child_frame_id}
  elif isinstance(msg,State):
   latest[name]={"connected":msg.connected,"armed":msg.armed,"mode":msg.mode}
  elif isinstance(msg,PointCloud2):
   latest[name]={"points":msg.width*msg.height,"frame":msg.header.frame_id}
 return cb
for topic,typ in [("/livox/lidar",CustomMsg),("/livox/imu",Imu),(args.odom,Odometry),
 ("/cloud_registered",PointCloud2),("/iris_0/mavros/state",State),("/iris_0/odom_world",Odometry)]:
 rospy.Subscriber(topic,typ,callback(topic),queue_size=5)
end=time.monotonic()+args.duration
while time.monotonic()<end and not rospy.is_shutdown():time.sleep(.2)
result={}
for name in ["/livox/lidar","/livox/imu",args.odom,"/cloud_registered","/iris_0/mavros/state","/iris_0/odom_world"]:
 ts=samples[name]
 result[name]={"messages":len(ts),"wall_hz":(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0,"latest":latest.get(name)}
for name,data in result.items():
 ss=stamps[name]
 if len(ss)>1 and ss[-1]>ss[0]:
  data["sim_hz"]=(len(ss)-1)/(ss[-1]-ss[0])
 pp=positions[name]
 if len(pp)>1:
  data["start_end_displacement"]=math.sqrt(sum((a-b)**2 for a,b in zip(pp[0],pp[-1])))
with open(args.output,"w") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
