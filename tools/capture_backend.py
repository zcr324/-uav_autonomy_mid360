#!/usr/bin/env python3
"""Request backend path + native saved map; capture ROS outputs without truth injection."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as RosPath
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    if not rospy.get_param('/use_sim_time',False):raise RuntimeError('Start the SITL framework before capture')
    rospy.init_node('capture_backend',anonymous=True)
    latest={}
    rospy.Subscriber('/aft_pgo_map',PointCloud2,lambda m:latest.update(cloud=m),queue_size=1)
    rospy.Subscriber('/aft_pgo_path',RosPath,lambda m:latest.update(path=m),queue_size=1)
    end=time.monotonic()+30;last=-1e9
    while time.monotonic()<end:
        if time.monotonic()-last>2:
            rospy.set_param('/save_map',True);rospy.set_param('/pub_pgopath',True);last=time.monotonic()
        if 'cloud' in latest and 'path' in latest:break
        time.sleep(.1)
    if not all(k in latest for k in ['cloud','path']):raise RuntimeError('Backend path/map capture timeout: '+str(list(latest)))
    m=latest['cloud']
    xyz=np.array(list(pc2.read_points(m,field_names=('x','y','z'),skip_nans=True)),dtype='<f4')
    header=('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'%(len(xyz),len(xyz))).encode()
    (out/'backend_map.pcd').write_bytes(header+xyz.tobytes())
    path=latest['path']
    poses=[{'stamp':p.header.stamp.to_sec(),'position':[p.pose.position.x,p.pose.position.y,p.pose.position.z],
        'quaternion':[p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]} for p in path.poses]
    (out/'backend_path.json').write_text(json.dumps({'frame':path.header.frame_id,'poses':poses},indent=2))
    res={'success':True,'map_frame':m.header.frame_id,'map_points':len(xyz),'path_poses':len(poses),
         'native_map':str(Path(rospy.get_param('/SaveDir'))/'cloud_result.pcd'),
         'note':'Map export success alone does not prove an accepted loop or accuracy improvement'}
    (out/'result.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
if __name__=='__main__':main()
