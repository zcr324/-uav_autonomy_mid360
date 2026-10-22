#!/usr/bin/env python3
"""Bounded SITL validation. Gazebo truth is recorded/evaluated, never fed to LIO.
Run inside the framework environment; output contains JSON, CSV, bag and map PCD.
"""
import fcntl
import argparse, collections, csv, json, math, os, signal, subprocess, sys, threading, time
from pathlib import Path
import numpy as np
import tf.transformations as tft
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, ParamGet
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseWithCovariance
from quadrotor_msgs.msg import PositionCommand

ROOT=Path(__file__).resolve().parents[1]
ROUTES=json.loads((ROOT/'config/experiments/routes.json').read_text())

def pos(p): return [p.x,p.y,p.z]
def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

class Experiment:
    def __init__(self,args):
        self.process_lock=open(ROOT/'integration/experiment.lock','w')
        try:fcntl.flock(self.process_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Another experiment owns this framework')
        self.a=args
        self.total_deadline=time.monotonic()+args.budget
        self.out=Path(args.output);self.out.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock()
        self.latest={};self.received={};self.count=collections.Counter()
        self.voxels={};self.last_cloud_time=-1e9
        self.rows=[];self.events=[];self.submaps=[];self.loops=[];self.paths={};self.backend_cloud=None
        self.active=False;self.bad_since=None;self.height_bad_since=None;self.bag=None;self.takeoff=None
        self.result={'mode':args.mode,'route':args.route,'success':False,'wall_budget_seconds':args.budget,'arrival_tolerance':args.arrival,'precision_tolerance':.3,'evaluation_gates_next_goal':True,'truth_usage':'evaluation, abort and next-goal acceptance; no estimator/planner/control-adapter pose injection'}
        # The CLI may start immediately after roslaunch. Initialize ROS time
        # only after the SITL clock parameter exists; otherwise rospy binds
        # to wall time for the entire process and invalidates comparisons.
        startup_deadline=time.monotonic()+90
        while True:
            try:
                if rospy.get_param('/use_sim_time',False):break
            except Exception:pass
            if time.monotonic()>startup_deadline:
                raise RuntimeError('SITL /use_sim_time readiness timeout')
            time.sleep(.1)
        rospy.init_node('framework_experiment',anonymous=True,disable_signals=True)
        self.min_height=rospy.get_param('/framework/min_flight_z',-1e6)
        self.max_height=rospy.get_param('/framework/max_flight_z',1e6)
        self.result['flight_height_envelope']=[self.min_height,self.max_height]
        self.direct=rospy.Publisher('/xtdrone/iris_0/cmd_pose_enu',Pose,queue_size=2)
        self.goal=rospy.Publisher('/iris_0/move_base_simple/goal',PoseStamped,queue_size=1)
        self.pgopub=rospy.Publisher('/pub_pgopath',Bool,queue_size=1,latch=True)
        self.subs=[
            rospy.Subscriber('/iris_0/odom_world',Odometry,self.odom_cb,queue_size=2),
            rospy.Subscriber('/gazebo/model_states',ModelStates,self.truth_cb,queue_size=1),
            rospy.Subscriber('/iris_0/mavros/state',State,lambda m:self.put('state',m),queue_size=1),
            rospy.Subscriber('/iris_0/mavros/local_position/pose',PoseStamped,lambda m:self.put('local',m),queue_size=1),
            rospy.Subscriber('/iris_0/planning/pos_cmd',PositionCommand,lambda m:self.put('cmd',m),queue_size=2),
            rospy.Subscriber('/iris_0/map/cloud_registered',PointCloud2,self.cloud_cb,queue_size=1),
            rospy.Subscriber('/submap_ids',Int32,lambda m:self.submaps.append(m.data),queue_size=10),
            rospy.Subscriber('/loop_closure_tranformation',PoseWithCovariance,self.loop_cb,queue_size=10),
            rospy.Subscriber('/aft_pgo_path',RosPath,lambda m:self.paths.update(pgo=m),queue_size=1),
            rospy.Subscriber('/aft_pgo_map',PointCloud2,lambda m:setattr(self,'backend_cloud',m),queue_size=1),
        ]
    def put(self,name,msg):
        with self.lock:
            self.latest[name]=msg;self.received[name]=time.monotonic();self.count[name]+=1
    def odom_cb(self,m): self.put('odom',m)
    def truth_cb(self,m):
        candidates=[(i,n) for i,n in enumerate(m.name) if n=='iris_0' or n.startswith('iris_mid360')]
        if not candidates:return
        i,n=candidates[0]
        self.put('truth',(rospy.get_time(),pos(m.pose[i].position),n))
    def loop_cb(self,m):
        self.loops.append({'sim_time':rospy.get_time(),'covariance':list(m.covariance),
                           'translation':pos(m.pose.position)})
    def cloud_cb(self,m):
        self.put('cloud',m)
        t=m.header.stamp.to_sec()
        if t-self.last_cloud_time<0.5:return
        self.last_cloud_time=t
        try:
            updates={}
            for i,p in enumerate(pc2.read_points(m,field_names=('x','y','z'),skip_nans=True)):
                if i%2:continue
                key=tuple(int(math.floor(v/0.15)) for v in p)
                updates[key]=tuple(p)
            with self.lock:
                self.voxels.update(updates)
                if len(self.voxels)>500000:
                    raise RuntimeError('Map capture exceeded 500000 voxels')
        except Exception as e:
            self.result['map_capture_error']=str(e)
    def event(self,name,**values):
        e={'event':name,'wall_time':time.time(),'sim_time':rospy.get_time(),**values}
        self.events.append(e);print(json.dumps(e),flush=True)
        (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
    def sample(self):
        with self.lock:
            o=self.latest.get('odom');truth=self.latest.get('truth');s=self.latest.get('state')
            if self.active:
                if time.monotonic()>self.total_deadline:
                    raise RuntimeError('Experiment wall-time budget exhausted')
                ages={k:time.monotonic()-self.received.get(k,0) for k in ['odom','truth','state']}
                if ages['odom']>2 or ages['truth']>2 or ages['state']>5:
                    raise RuntimeError('Required stream stale (wall seconds): '+str(ages))
            if o is None or truth is None:return
            mapped=pos(o.pose.pose.position)
            synced=abs(o.header.stamp.to_sec()-truth[0])<.2
            err=distance(mapped,truth[1]) if synced else None
            local=self.latest.get('local');cmd=self.latest.get('cmd')
            row={'wall':time.time(),'sim':rospy.get_time(),'odom_stamp':o.header.stamp.to_sec(),
                 'map_x':mapped[0],'map_y':mapped[1],'map_z':mapped[2],
                 'truth_x':truth[1][0],'truth_y':truth[1][1],'truth_z':truth[1][2],
                 'truth_error':err,'navigation_active':self.active,'armed':bool(s.armed) if s else False,'mode':s.mode if s else '',
                 'local_x':local.pose.position.x if local else None,'local_y':local.pose.position.y if local else None,
                 'local_z':local.pose.position.z if local else None,
                 'cmd_x':cmd.position.x if cmd else None,'cmd_y':cmd.position.y if cmd else None,
                 'cmd_z':cmd.position.z if cmd else None}
            self.rows.append(row)
            if self.active:
                if time.monotonic()-self.received.get('odom',0)>2:raise RuntimeError('Odometry stale for >2 wall seconds')
                if s is None or not s.connected or not s.armed or s.mode!='OFFBOARD':
                    raise RuntimeError('Lost connected/armed/OFFBOARD state')
                if err is not None and err>self.a.max_error:
                    if self.bad_since is None:self.bad_since=time.monotonic()
                    if time.monotonic()-self.bad_since>1:raise RuntimeError('LIO world error %.3fm exceeds %.3fm for >1s'%(err,self.a.max_error))
                else:self.bad_since=None
                if not self.min_height-.1 <= mapped[2] <= self.max_height+.1:
                    if self.height_bad_since is None:self.height_bad_since=time.monotonic()
                    if time.monotonic()-self.height_bad_since>1:
                        raise RuntimeError('Estimated flight height outside configured envelope for >1s')
                else:self.height_bad_since=None
    def wait(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not rospy.is_shutdown():
            self.sample();time.sleep(.1)
    def ready(self):
        end=time.monotonic()+90
        while time.monotonic()<end:
            self.sample()
            with self.lock:
                if all(k in self.latest for k in ['odom','truth','state','local','cloud']):
                    s=self.latest['state']
                    if s.connected and time.monotonic()-self.received['odom']<1 and (self.a.mode=='baseline' or self.goal.get_num_connections()>0):
                        self.event('ready',map_position=pos(self.latest['odom'].pose.pose.position),
                                   truth_position=self.latest['truth'][1],armed=s.armed,mode=s.mode)
                        return
            time.sleep(.1)
        raise RuntimeError('Readiness timeout: '+str(dict(self.count)))
    def record(self):
        topics=['/clock','/livox/lidar','/livox/imu','/aft_mapped_to_init','/iris_0/odom_world',
                '/gazebo/model_states','/iris_0/mavros/state','/iris_0/mavros/local_position/pose',
                '/iris_0/planning/pos_cmd','/iris_0/move_base_simple/goal',
                '/submap_ids','/submap_pose','/clouds_submap','/inter_triangles','/inter2_triangles','/lc_triangles','/loop_closure_tranformation','/loopclosure','/aft_pgo_path','/aft_pgo_map']
        self.baglog=open(self.out/'record.log','w')
        self.bag=subprocess.Popen(['rosbag','record','--lz4','--buffsize=64','-O',str(self.out/'data.bag')]+topics,
                                  stdout=self.baglog,stderr=subprocess.STDOUT,start_new_session=True)
    def get_params(self):
        out={}
        for name in ['EKF2_AID_MASK','EKF2_HGT_MODE']:
            end=time.monotonic()+20
            while True:
                try:
                    response=rospy.ServiceProxy('/iris_0/mavros/param/get',ParamGet)(name)
                    out[name]={'success':response.success,'integer':response.value.integer,'real':response.value.real}
                    if response.success:break
                except Exception as e:out[name]={'success':False,'error':str(e)}
                if time.monotonic()>end:break
                self.wait(.5)
        self.result['px4_parameters']=out
        if self.a.mode=='flight' and not all(v.get('success') for v in out.values()):
            raise RuntimeError('PX4 estimator configuration readback unavailable; flight not started')

    def baseline(self):
        self.event('static_start')
        start=rospy.get_time();deadline=time.monotonic()+max(90,self.a.duration*4)
        initial=pos(self.latest['odom'].pose.pose.position)
        while rospy.get_time()-start<self.a.duration:
            self.sample()
            errors=[v['truth_error'] for v in self.rows[-10:] if v['truth_error'] is not None]
            if errors and min(errors)>2:
                raise RuntimeError('Static localization diverged >2m; stopped before flight')
            if time.monotonic()>deadline:raise RuntimeError('Static experiment wall timeout / simulation stalled')
            time.sleep(.1)
        final=pos(self.latest['odom'].pose.pose.position)
        errors=[v['truth_error'] for v in self.rows if v['truth_error'] is not None]
        self.result.update(static_displacement=distance(initial,final),
                           max_truth_error=max(errors) if errors else None)
        if not errors or max(errors)>.5 or distance(initial,final)>.5:
            raise RuntimeError('Static localization gate failed (>0.5m displacement or truth error)')
        self.result['success']=True
    def planner_enable(self,value):
        rospy.wait_for_service('/control_transfer/set_enabled',timeout=10)
        response=rospy.ServiceProxy('/control_transfer/set_enabled',SetBool)(value)
        if not response.success:raise RuntimeError(response.message)
        self.event('planner_control',enabled=value)

    def flight(self):
        self.planner_enable(False)
        if self.latest['state'].armed:raise RuntimeError('Flight must start disarmed')
        response=rospy.ServiceProxy('/control_transfer/reset_alignment',Trigger)()
        if not response.success:raise RuntimeError(response.message)
        wall_start=time.monotonic();sim_start=rospy.get_time()
        self.wait(3)
        self.real_time_factor=max(.1,min(2,(rospy.get_time()-sim_start)/(time.monotonic()-wall_start)))
        self.result['measured_real_time_factor']=self.real_time_factor
        errors=[v['truth_error'] for v in self.rows[-20:] if v['truth_error'] is not None]
        if not errors or max(errors)>.5:raise RuntimeError('Preflight localization error >0.5m')
        self.event('takeoff_start')
        f=open(self.out/'takeoff.log','w')
        self.takeoff=subprocess.Popen(['rosrun','mission','takeoff.py','iris','0','_height:=1.0','_timeout:=90'],
                                      stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+95
        while self.takeoff.poll() is None:
            self.sample()
            if time.monotonic()>deadline:raise RuntimeError('Takeoff subprocess timeout')
            time.sleep(.1)
        if self.takeoff.returncode:raise RuntimeError('Takeoff failed; see takeoff.log')
        self.active=True;self.planner_enable(True);self.wait(2)
        self.event('takeoff_complete')
        if self.a.rotation_revisits:
            self.result['experiment_type']='stationary_rotation_revisit_component_probe'
            self.result['navigation_route_evaluated']=False
            self.result['evaluation_gates_next_goal']=False
            self.result['truth_usage']='evaluation and abort only; no route goals are sent in this component probe'
            self.result['scan_passes_completed']=0
            for scan_pass in range(self.a.rotation_revisits):
                self.yaw_scan(degrees=360,label='component_revisit_%d'%(scan_pass+1),map_height=1.05)
                self.result['scan_passes_completed']=scan_pass+1
                accepted=self.out.parent/'ltaom/loopclosure.txt'
                count=len([line for line in accepted.read_text().splitlines() if line.strip()]) if accepted.exists() else 0
                self.result['accepted_loop_log_lines']=count
                self.event('component_scan_evidence',pass_index=scan_pass+1,accepted_loop_log_lines=count)
                if count>=2:break
            self.result['success']=self.result.get('accepted_loop_log_lines',0)>=2
            if not self.result['success']:self.result['error']='Bounded component probe ended without two accepted loop constraints'
            return
        reached=[]
        for i,wp in enumerate(ROUTES[self.a.route]['waypoints']):
            self.event('goal_sent',index=i,target=wp)
            goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=rospy.Time.now()
            goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=wp[:3];goal.pose.orientation.w=1
            self.goal.publish(goal)
            initial=pos(self.latest['odom'].pose.pose.position)
            deadline=time.monotonic()+min(180,max(45,(distance(initial,wp)/.5+10)*2/self.real_time_factor))
            hold_since=None
            while time.monotonic()<deadline:
                self.sample()
                mapped=pos(self.latest['odom'].pose.pose.position)
                err=distance(mapped,wp)
                truth_err=distance(self.latest['truth'][1],wp)
                if err<self.a.arrival and truth_err<self.a.arrival:
                    if hold_since is None:hold_since=time.monotonic()
                    if time.monotonic()-hold_since>.6:break
                else:hold_since=None
                time.sleep(.1)
            else:raise RuntimeError('Waypoint %d timeout, map error %.3fm, truth error %.3fm'%(i,err,truth_err))
            true_err=distance(self.latest['truth'][1],wp)
            reached.append({'index':i,'map_error':err,'truth_error':true_err,
                            'precision_pass':err<.3 and true_err<.3})
            if true_err>self.a.arrival:
                raise RuntimeError('Waypoint %d truth error %.3fm exceeds navigation tolerance %.3fm'%(i,true_err,self.a.arrival))
            self.event('goal_reached',**reached[-1])
            self.result['reached']=reached
            if i==0 and self.a.reference_scan:
                self.yaw_scan(degrees=self.a.reference_scan,label='reference',map_height=wp[2])
                self.planner_enable(True)
        if self.a.yaw_scan:
            self.yaw_scan(map_height=ROUTES[self.a.route]['waypoints'][-1][2])
        self.wait(3)
        self.result['precision_success']=all(v['precision_pass'] for v in reached)
        self.result['success']=True
    def yaw_scan(self,degrees=None,label='final',map_height=None):
        # A real additional sensor observation at the revisit location.
        # The planner is disabled while a fixed local position is held.
        self.planner_enable(False)
        p=self.latest['local'].pose
        q=p.orientation
        yaw=tft.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        target=Pose()
        target.position.x,target.position.y,target.position.z=p.position.x,p.position.y,p.position.z
        if map_height is not None:
            target.position.z += map_height-self.latest['odom'].pose.pose.position.z
        target.orientation.x=yaw
        settle_deadline=time.monotonic()+20
        while map_height is not None and abs(self.latest['odom'].pose.pose.position.z-map_height)>.08:
            self.direct.publish(target);self.sample()
            if time.monotonic()>settle_deadline:raise RuntimeError('Reference scan height did not settle')
            time.sleep(.05)
        angle=math.radians(self.a.yaw_scan if degrees is None else degrees);speed=.15
        start=rospy.get_time();deadline=time.monotonic()+min(300,angle/speed/max(.2,self.real_time_factor)*2+20)
        self.event('yaw_scan_start',phase=label,degrees=math.degrees(angle),rate_radians=speed)
        while True:
            phase=min(angle,max(0,(rospy.get_time()-start)*speed))
            target.orientation.x=math.atan2(math.sin(yaw+phase),math.cos(yaw+phase))
            target.orientation.y=speed if phase<angle else 0
            self.direct.publish(target);self.sample()
            if phase>=angle:break
            if time.monotonic()>deadline:raise RuntimeError('Yaw scan wall timeout')
            time.sleep(.05)
        self.wait(3);self.event('yaw_scan_complete',phase=label)

    def land(self):
        self.active=False
        try:self.planner_enable(False)
        except Exception as e:self.event('disable_planner_error',error=str(e))
        if self.takeoff and self.takeoff.poll() is None:
            os.killpg(self.takeoff.pid,signal.SIGTERM)
        if not self.latest.get('state') or not self.latest['state'].armed:return
        self.event('land_start')
        end=time.monotonic()+65;last=0
        while time.monotonic()<end:
            s=self.latest['state']
            if not s.armed:
                self.event('landed_disarmed');self.result['landed_disarmed']=True;return
            if time.monotonic()-last>3:
                try:rospy.ServiceProxy('/iris_0/mavros/set_mode',SetMode)(custom_mode='AUTO.LAND')
                except Exception as e:self.event('land_service_error',error=str(e))
                last=time.monotonic()
            self.sample();time.sleep(.2)
        self.result['landed_disarmed']=False
        self.result['success']=False
        self.event('land_timeout')
    def finish(self):
        if self.bag and self.bag.poll() is None:
            os.killpg(self.bag.pid,signal.SIGINT)
            try:self.bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.bag.pid,signal.SIGTERM);self.bag.wait(timeout=5)
        with self.lock:
            rows=list(self.rows);points=list(self.voxels.values())
        if rows:
            with open(self.out/'telemetry.csv','w') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            errors=[v['truth_error'] for v in rows if v['truth_error'] is not None]
            self.result['metrics']={'samples':len(rows),'max_truth_error':max(errors) if errors else None,
              'rmse_truth_error':math.sqrt(sum(x*x for x in errors)/len(errors)) if errors else None,
              'truth_path_length':sum(distance([a['truth_x'],a['truth_y'],a['truth_z']],[b['truth_x'],b['truth_y'],b['truth_z']]) for a,b in zip(rows,rows[1:]))}
        self.result.update(message_counts=dict(self.count),submap_ids=self.submaps,
                           loop_messages=self.loops,map_voxels=len(points))
        (self.out/'result.json').write_text(json.dumps(self.result,indent=2))
        (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
        with open(self.out/'map.pcd','w') as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'%(len(points),len(points)))
            f.writelines('%.4f %.4f %.4f\n'%p for p in points)
        if self.backend_cloud is not None:
            pgo_points=list(pc2.read_points(self.backend_cloud,field_names=('x','y','z'),skip_nans=True))
            with open(self.out/'backend_map.pcd','w') as f:
                f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'%(len(pgo_points),len(pgo_points)))
                f.writelines('%.4f %.4f %.4f\n'%tuple(p) for p in pgo_points)
        for name,path in self.paths.items():
            (self.out/(name+'_path.json')).write_text(json.dumps({'frame':path.header.frame_id,
                'poses':[{'stamp':p.header.stamp.to_sec(),'position':pos(p.pose.position),
                          'quaternion':[p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]} for p in path.poses]},indent=2))
        print(json.dumps(self.result,indent=2),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['baseline','flight'])
    p.add_argument('--route',choices=list(ROUTES),default='short')
    p.add_argument('--output',required=True)
    p.add_argument('--duration',type=float,default=30,help='static duration in simulation seconds')
    p.add_argument('--budget',type=float,default=1200,help='maximum experiment wall time before landing (seconds)')
    p.add_argument('--rotation-revisits',type=int,default=0,help='Independent stationary backend component probe, 0..6 real yaw turns; no route evaluation')
    p.add_argument('--reference-scan',type=float,default=0,help='real yaw reference sweep after the first waypoint, degrees (0..360)')
    p.add_argument('--yaw-scan',type=float,default=0,help='additional real yaw sweep at final revisit, degrees (0..360)')
    p.add_argument('--arrival',type=float,default=.3,help='navigation arrival tolerance; 0.3m precision is reported separately')
    p.add_argument('--max-error',type=float,default=1.0,help='abort flight on sustained LIO/world error (metres)')
    args=p.parse_args()
    if not math.isfinite(args.arrival) or not 0 < args.arrival <= 1:
        p.error('--arrival must be finite and in (0,1] metres')
    if not math.isfinite(args.max_error) or args.max_error <= 0:
        p.error('--max-error must be positive and finite')
    if not math.isfinite(args.yaw_scan) or not 0 <= args.yaw_scan <= 360:p.error('--yaw-scan must be in [0,360]')
    if not math.isfinite(args.reference_scan) or not 0 <= args.reference_scan <= 360:p.error('--reference-scan must be in [0,360]')
    if not math.isfinite(args.budget) or args.budget<=0:p.error('--budget must be positive and finite')
    if not 0<=args.rotation_revisits<=6:p.error('--rotation-revisits must be 0..6')
    e=Experiment(args)
    try:
        e.ready();e.get_params();e.record()
        if args.mode=='baseline':e.baseline()
        else:e.flight()
    except Exception as ex:
        e.result.update(success=False,error=str(ex));e.event('failed',error=str(ex))
    finally:
        if args.mode=='flight':e.land()
        try:
            rospy.set_param('/pub_pgopath',True);e.wait(3)
        except Exception as ex:e.result['pgo_capture_error']=str(ex)
        e.finish()
    return 0 if e.result['success'] else 1
if __name__=='__main__':sys.exit(main())
