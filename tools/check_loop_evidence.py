#!/usr/bin/env python3
"""Audit accepted PGO constraints against recorded Gazebo IMU poses.
Truth is read offline only. No algorithm inputs are modified.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
import rosbag,rospy
import tf.transformations as tf
ROOT=Path(__file__).resolve().parents[1]
MOUNT=np.array([-.005,.005,.147])
def quat(p):return [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def transform(position,q):
    t=tf.quaternion_matrix(q);t[:3,3]=position;return t
def audit(run,reference=None):
    d=Path(run);reference=Path(reference) if reference else d;out={}
    rawfile=d/'ltaom/odom_poses.txt';optfile=d/'ltaom/optimized_poses.txt'
    raw=np.atleast_2d(np.loadtxt(rawfile));opt=np.atleast_2d(np.loadtxt(optfile))
    rawxyz={int(v[0]):v[1:4] for v in raw}
    optposes={i:v for i,v in enumerate(opt)}
    bags=[]
    for p in sorted(reference.glob('*/data.bag')):
        b=rosbag.Bag(str(p));bags.append((b,b.get_start_time(),b.get_end_time()))
    keyposes={}
    try:
        for b,_,_ in bags:
            for _,m,_ in b.read_messages(topics=['/submap_pose']):
                keyposes[int(m.twist.covariance[0])]=m
        cache={}
        def truth_at(stamp):
            key=round(stamp,6)
            if key in cache:return cache[key]
            best=None
            for b,start,end in bags:
                if not start-.1<=stamp<=end+.1:continue
                for _,m,t in b.read_messages(topics=['/gazebo/model_states'],
                      start_time=rospy.Time.from_sec(max(0,stamp-.04)),
                      end_time=rospy.Time.from_sec(stamp+.04)):
                    if 'iris_0' not in m.name:continue
                    p=m.pose[m.name.index('iris_0')];delta=abs(t.to_sec()-stamp)
                    if best is None or delta<best[0]:
                        tb=transform([p.position.x,p.position.y,p.position.z],quat(p))
                        ti=tb.copy();ti[:3,3]+=tb[:3,:3]@MOUNT
                        best=(delta,ti)
            cache[key]=best[1] if best and best[0]<.04 else None
            return cache[key]
        constraints=[]
        seen=set()
        for line in (d/'ltaom/loopclosure.txt').read_text().splitlines():
            v=[float(x) for x in line.split()]
            if len(v)<8:continue
            i,j=int(v[0]),int(v[1])
            if (i,j) in seen:continue
            seen.add((i,j));entry={'previous':i,'current':j,'index_separation':abs(j-i)}
            if i in optposes and j in optposes:
                ti,tj=truth_at(optposes[i][0]),truth_at(optposes[j][0])
                if ti is not None and tj is not None:
                    expected=np.linalg.inv(ti)@tj
                    observed=tf.euler_matrix(*v[5:8]);observed[:3,3]=v[2:5]
                    delta=np.linalg.inv(expected)@observed
                    angle=math.degrees(math.acos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))
                    entry.update(translation_error=float(np.linalg.norm(delta[:3,3])),rotation_error_degrees=angle,
                                 actual_pose_separation=float(np.linalg.norm(ti[:3,3]-tj[:3,3])))
                    entry['consistent_with_truth']=entry['translation_error']<.5 and angle<5
            constraints.append(entry)
        out={'accepted_unique_constraints':len(constraints),'constraints':constraints,
             'truth_consistent_constraints':sum(e.get('consistent_with_truth',False) for e in constraints),
             'note':'Consistency thresholds are provisional 0.5m/5deg; accepted matches alone do not prove trajectory benefit'}
        if not constraints:
            out['loop_benefit_demonstrated']=False
            (d/'loop_evidence.json').write_text(json.dumps(out,indent=2))
            print(json.dumps(out,indent=2))
            return
        metadata=json.loads((reference/'launch.json').read_text())['settings']
        initial=np.array([metadata['x'],metadata['y'],metadata['init_z']])
        rows=[]
        for ident,v in optposes.items():
            if ident not in rawxyz or ident not in keyposes:continue
            truth=truth_at(v[0])
            if truth is None:continue
            # Same fixed declaration as lio_transfer: no best-fit alignment to truth.
            rraw=tf.quaternion_matrix(quat(keyposes[ident].pose.pose))[:3,:3]
            ropt=tf.quaternion_matrix(v[4:8])[:3,:3]
            praw=keyposes[ident].pose.pose.position
            rp=np.array([praw.x,praw.y,praw.z])+initial+MOUNT-rraw@MOUNT
            op=v[1:4]+initial+MOUNT-ropt@MOUNT
            # Convert truth sensor origin back to base origin.
            tp=truth[:3,3]-truth[:3,:3]@MOUNT
            rows.append({'id':ident,'stamp':v[0],'raw':rp.tolist(),'optimized':op.tolist(),'truth':tp.tolist(),
                         'raw_error':float(np.linalg.norm(rp-tp)),'optimized_error':float(np.linalg.norm(op-tp))})
        if rows:
            re=np.array([v['raw_error'] for v in rows]);oe=np.array([v['optimized_error'] for v in rows])
            out['trajectory']={'matched_keyframes':len(rows),'raw_rmse':float(np.sqrt(np.mean(re**2))),
                'optimized_rmse':float(np.sqrt(np.mean(oe**2))),
                'raw_max':float(re.max()),'optimized_max':float(oe.max())}
            cutoff=min(e['current'] for e in constraints)
            historical=[v for v in rows if v['id']<=cutoff]
            if historical:
                hr=np.array([v['raw_error'] for v in historical]);ho=np.array([v['optimized_error'] for v in historical])
                out['historical_trajectory']={'maximum_id':cutoff,'matched_keyframes':len(historical),
                    'raw_rmse':float(np.sqrt(np.mean(hr**2))),'optimized_rmse':float(np.sqrt(np.mean(ho**2)))}
                out['descriptive_historical_improvement']=out['historical_trajectory']['optimized_rmse']<out['historical_trajectory']['raw_rmse']-.001
            # Adjacent point factors also alter PGO poses. A no-loop replay of the
            # SAME recorded backend inputs is needed to attribute any benefit.
            out['loop_benefit_demonstrated']=False
            out['causal_comparison_required']='Identical recorded backend input with and without loop constraints'
            out['raw_pose_source']='Recorded /submap_pose; no mixed post-correction odom_poses baseline'
            out['truth_alignment']='Fixed declared spawn and sensor mount; no best-fit truth alignment'
        else:out['loop_benefit_demonstrated']=False
        (d/'loop_truth_poses.json').write_text(json.dumps(rows,indent=2))
        (d/'loop_evidence.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
    finally:
        for b,_,_ in bags:b.close()
def main():
    p=argparse.ArgumentParser();p.add_argument('run_directory');p.add_argument('--reference-run',help='Live run providing recorded truth and spawn when auditing a replay');a=p.parse_args();audit(a.run_directory,a.reference_run)
if __name__=='__main__':main()
