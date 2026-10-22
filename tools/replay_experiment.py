#!/usr/bin/env python3
"""Replay raw sensors, exact STD submaps, or recorded backend inputs on an isolated master.
Gazebo truth, planning and control are never fed to algorithm nodes.
"""
import argparse,json,os,signal,socket,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from process_utils import stop_tree
def stop(p):
    if p:
        stop_tree(p.pid)
        try:p.wait(timeout=2)
        except subprocess.TimeoutExpired:pass
def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('bag');a.add_argument('--output',required=True)
    a.add_argument('--rate',type=float,default=1)
    a.add_argument('--mode',choices=['raw','submaps','backend'],default='raw')
    a.add_argument('--refine-loop-icp',action='store_true',help='Submaps-only original plane ICP refinement; default off, FPR thresholds unchanged')
    a.add_argument('--port',type=int,default=11331)
    a.add_argument('--omit-loop-constraints',action='store_true',help='Backend-only ablation: preserve recorded submaps/triangles, omit LC transformations')
    a.add_argument('--pgo-config',default=str(ROOT/'src/lio/loop_optimization/launch/loopopt_config_avia.yaml'))
    a.add_argument('--lcd-config',default=str(ROOT/'src/lio/STD/config/config_mid360.yaml'))
    args=a.parse_args()
    if args.refine_loop_icp and args.mode!='submaps':a.error('--refine-loop-icp requires --mode submaps')
    if args.omit_loop_constraints and args.mode!='backend':a.error('--omit-loop-constraints requires --mode backend')
    bag=Path(args.bag).resolve();out=Path(args.output).resolve()
    if not bag.is_file():raise RuntimeError('Input bag missing')
    if out.exists() and any(out.iterdir()):raise RuntimeError('Output directory must be new/empty')
    out.mkdir(parents=True,exist_ok=True);(out/'ltaom').mkdir()
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.port
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',args.port))==0:raise RuntimeError('Replay master port occupied')
    import rosbag
    with rosbag.Bag(str(bag)) as b:
        duration=b.get_end_time()-b.get_start_time();end_sim=b.get_end_time()
        topic_names=set(b.get_type_and_topic_info().topics)
    topics={'raw':['/livox/lidar','/livox/imu'],
            'submaps':['/submap_pose','/clouds_submap'],
            'backend':['/submap_pose','/clouds_submap','/inter_triangles','/lc_triangles','/loop_closure_tranformation']}[args.mode]
    if args.omit_loop_constraints:topics.remove('/loop_closure_tranformation')
    if not set(topics[:2]).issubset(topic_names):raise RuntimeError('Bag lacks required input topics')
    if not 0 < args.rate <= 10:raise RuntimeError('Replay rate must be in (0,10]')
    core=modules=player=recorder=None
    result={'success':False,'input_bag':str(bag),'input_topics':topics,'mode':args.mode,
            'refine_loop_icp':args.refine_loop_icp,'omit_loop_constraints':args.omit_loop_constraints,'rate':args.rate,'pgo_config':args.pgo_config,'lcd_config':str(Path(args.lcd_config).resolve()),'duration':duration}
    try:
        core=subprocess.Popen(['roscore','-p',str(args.port)],stdout=open(out/'roscore.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        import rosgraph,rospy
        from rosgraph_msgs.msg import Clock
        deadline=time.monotonic()+15
        while True:
            try:rosgraph.Master('/replay_runner').getPid();break
            except Exception:
                if time.monotonic()>deadline:raise RuntimeError('Replay ROS master timeout')
                time.sleep(.1)
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('replay_runner',anonymous=True,disable_signals=True)
        clock=rospy.Publisher('/clock',Clock,queue_size=1)
        clock.publish(Clock(rospy.Time.from_sec(max(.001,end_sim-duration-.1))))
        launch=['roslaunch','sim_env','modules.launch','bridge:=false','planning:=false','control:=false','loop:=true'] if args.mode=='raw' else [
            'roslaunch','loop_optimization','submap_replay.launch','start_std:='+str(args.mode=='submaps').lower()]
        if args.mode=='submaps':launch+=['refine_loop_icp:='+str(args.refine_loop_icp).lower()]
        launch+=['save_dir:='+str(out/'ltaom')+'/','lcd_config:='+str(Path(args.lcd_config).resolve()),
                 'pgo_config:='+str(Path(args.pgo_config).resolve())]
        modules=subprocess.Popen(launch,stdout=open(out/'modules.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(3)
        recorder=subprocess.Popen(['rosbag','record','--lz4','-O',str(out/'outputs.bag'),
            '/aft_mapped_to_init','/cloud_registered','/submap_pose','/submap_ids','/clouds_submap',
            '/inter_triangles','/inter2_triangles','/lc_triangles','/loop_closure_tranformation','/loopclosure','/aft_pgo_path','/aft_pgo_map'],
            stdout=open(out/'record.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        remaps=['/submap_pose:=/recorded/submap_pose','/clouds_submap:=/recorded/clouds_submap'] if args.mode=='submaps' else []
        player=subprocess.Popen(['rosbag','play','--clock','--delay=2','--rate='+str(args.rate),str(bag)]+remaps+['--topics']+topics,
            stdout=open(out/'play.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+duration/args.rate*2+60
        while player.poll() is None:
            if modules.poll() is not None:raise RuntimeError('Replay modules exited early')
            if time.monotonic()>deadline:raise RuntimeError('Replay wall-time budget exceeded')
            time.sleep(.2)
        if player.returncode:raise RuntimeError('rosbag play failed')
        # Keep ROS time advancing after the bag ends so Rate/Timers can flush.
        rospy.set_param('/pub_pgopath',True);rospy.set_param('/save_map',True)
        for i in range(100):
            clock.publish(Clock(rospy.Time.from_sec(end_sim+(i+1)*.05)));time.sleep(.05)
        result['success']=True
    except Exception as e:result['error']=str(e)
    finally:
        for p in [player,recorder,modules,core]:stop(p)
        poses=out/'ltaom/optimized_poses.txt'
        loops=out/'ltaom/loopclosure.txt'
        result['optimized_poses']=len(poses.read_text().splitlines()) if poses.exists() else 0
        result['accepted_loop_log_lines']=len(loops.read_text().splitlines()) if loops.exists() else 0
        result['native_map_saved']=(out/'ltaom/cloud_result.pcd').exists()
        if not result['optimized_poses']:
            result.update(success=False,error=result.get('error','No optimized keyframes were generated'))
        (out/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
