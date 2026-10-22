#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def main():
    p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();d=Path(a.run)
    cloud=d/'backend_capture/backend_map.pcd'
    if not cloud.exists():cloud=d/'ltaom/cloud_result.pcd'
    raw=cloud.read_bytes()
    marker=b'DATA binary'+bytes([10]);i=raw.index(marker)+len(marker)
    header={}
    for line in raw[:i].decode('ascii').splitlines():
        parts=line.split()
        if parts and not parts[0].startswith('#'):header[parts[0]]=parts[1:]
    types={'F':'f','I':'i','U':'u'}
    fields=header['FIELDS'];sizes=header['SIZE'];kinds=header['TYPE'];counts=header.get('COUNT',['1']*len(fields))
    dtype=np.dtype([(name,'<'+types[kind]+size,int(count)) if int(count)>1 else (name,'<'+types[kind]+size)
                    for name,size,kind,count in zip(fields,sizes,kinds,counts)])
    data=np.frombuffer(raw[i:],dtype=dtype)
    xyz=np.column_stack([data[name] for name in ['x','y','z']])
    xyz=xyz[np.isfinite(xyz).all(axis=1)]
    pathfile=d/'backend_capture/backend_path.json'
    if pathfile.exists():
        path=json.loads(pathfile.read_text())['poses']
        poses=np.array([p['position'] for p in path])
    else:
        poses=np.atleast_2d(np.loadtxt(d/'ltaom/optimized_poses.txt'))[:,1:4]
    lc=(d/'ltaom/loopclosure.txt')
    loops=[l for l in lc.read_text().splitlines() if l.strip()] if lc.exists() else []
    fig=plt.figure(figsize=(13,6))
    ax=fig.add_subplot(121,projection='3d')
    pts=xyz[::max(1,len(xyz)//40000)]
    ax.scatter(pts[:,0],pts[:,1],pts[:,2],s=.5,c=pts[:,2],cmap='viridis')
    ax.plot(poses[:,0],poses[:,1],poses[:,2],color='black',linewidth=1)
    ax.set_xlabel('camera_init X [m]');ax.set_ylabel('Y [m]');ax.set_zlabel('Z [m]')
    ax.set_title('Exported backend point cloud')
    ax.view_init(elev=36,azim=-65)
    bx=fig.add_subplot(122)
    bx.scatter(pts[:,0],pts[:,1],s=.5,c=pts[:,2],cmap='viridis')
    bx.plot(poses[:,0],poses[:,1],color='black',label='PGO keyframes')
    bx.set_aspect('equal',adjustable='datalim');bx.legend();bx.grid(alpha=.3)
    bx.set_title('Top view / camera_init frame')
    fig.suptitle('Sensor reconstruction: %d points / %d poses / accepted-loop log lines=%d'%(len(xyz),len(poses),len(loops)))
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(d/'backend_map.png',dpi=160)
    print(d/'backend_map.png')
if __name__=='__main__':main()
