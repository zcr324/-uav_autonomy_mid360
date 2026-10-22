#!/usr/bin/env python3
"""Exercise the real traj_server with an isolated ROS master and synthetic /clock.
No simulator or flight commands. Regressions: first timer dt=0 and future start.
"""
import os,signal,subprocess,sys,time,socket
from process_utils import stop_tree
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11321'
import rosgraph
import rospy
from rosgraph_msgs.msg import Clock
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Point
def main():
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',11321)) == 0:
            raise RuntimeError('Clock regression ROS master port is occupied')
    master=subprocess.Popen(['roscore','-p','11321'],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
    server=None
    try:
        end=time.monotonic()+15
        while time.monotonic()<end:
            try:rosgraph.Master('/clock_test').getPid();break
            except Exception:time.sleep(.1)
        else:raise RuntimeError('Isolated ROS master did not start')
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('clock_regression',anonymous=True,disable_signals=True)
        pub=rospy.Publisher('/clock',Clock,queue_size=1)
        spline=rospy.Publisher('/clock_test/bspline',Bspline,queue_size=1,latch=True)
        msgs=[]
        rospy.Subscriber('/clock_test/command',PositionCommand,lambda m:msgs.append(m),queue_size=100)
        server=subprocess.Popen(['rosrun','ego_planner','traj_server','__name:=clock_test_server',
            '/position_cmd:=/clock_test/command','~planning/bspline:=/clock_test/bspline','_traj_server/time_forward:=1.0'],
            stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
        now=10.0
        def step(n=1):
            nonlocal now
            for _ in range(n):
                now+=.01;pub.publish(Clock(rospy.Time.from_sec(now)));time.sleep(.005)
        end=time.monotonic()+15
        while spline.get_num_connections()==0 and time.monotonic()<end:step()
        step(150)
        def send(ident,start):
            b=Bspline();b.order=3;b.traj_id=ident;b.start_time=rospy.Time.from_sec(start)
            b.pos_pts=[Point(1,2,1) for _ in range(8)]
            b.knots=[(i-3)*.2 for i in range(12)]
            spline.publish(b)
        send(1,now);step(80)
        assert len(msgs)>5,'No PositionCommand output'
        import math
        assert all(math.isfinite(v) for m in msgs for v in [m.position.x,m.position.y,m.position.z,m.yaw,m.yaw_dot]),'Non-finite first-callback yaw rate'
        # Future-start messages must not emit an artificial zero position.
        send(2,now+1.0);step(80);step(80)
        assert all(abs(m.position.x-1)<1e-6 and abs(m.position.y-2)<1e-6 for m in msgs),'Published a spurious zero target before start'
        assert any(m.trajectory_id==2 for m in msgs),'Future trajectory never started'
        print('PASS: real traj_server first-callback dt=0, finite yaw rate, future-start target; messages=%d'%len(msgs))
    finally:
        for p in [server,master]:
            if p and p.poll() is None:
                stop_tree(p.pid)
                p.wait(timeout=3)
if __name__=='__main__':main()
