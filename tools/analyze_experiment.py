#!/usr/bin/env python3
"""Render recorded results with ground truth and world geometry (evaluation only)."""
import argparse,csv,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import tf.transformations as tft
ROOT=Path(__file__).resolve().parents[1]
def world_shapes(path):
    shapes=[]
    for m in ET.parse(path).findall('.//world/model'):
        model=tft.euler_matrix(*([float(x) for x in (m.findtext('pose') or '0 0 0 0 0 0').split()][3:]))
        model[:3,3]=[float(x) for x in (m.findtext('pose') or '0 0 0 0 0 0').split()][:3]
        for link in m.findall('link'):
            lv=[float(x) for x in (link.findtext('pose') or '0 0 0 0 0 0').split()]
            lm=tft.euler_matrix(*lv[3:]);lm[:3,3]=lv[:3]
            for c in link.findall('collision'):
                cv=[float(x) for x in (c.findtext('pose') or '0 0 0 0 0 0').split()]
                cm=tft.euler_matrix(*cv[3:]);cm[:3,3]=cv[:3]
                tr=model@lm@cm
                box=c.findtext('geometry/box/size')
                cyl=c.find('geometry/cylinder')
                if box:
                    half=np.array([float(x)/2 for x in box.split()])
                    shapes.append((m.get('name'),tr,'box',half))
                elif cyl is not None:
                    shapes.append((m.get('name'),tr,'cylinder',np.array([float(cyl.findtext('radius')),float(cyl.findtext('length'))/2])))
    return shapes
def clearance(point,shapes):
    out=(1e9,'')
    for name,tr,kind,size in shapes:
        if name=='large_ground_plane':continue
        v=tr[:3,:3].T@(np.array(point)-tr[:3,3])
        if kind=='box':
            q=np.abs(v)-size
            dist=np.linalg.norm(np.maximum(q,0))+min(max(q),0)
        else:
            q=np.array([np.linalg.norm(v[:2])-size[0],abs(v[2])-size[1]])
            dist=np.linalg.norm(np.maximum(q,0))+min(max(q),0)
        if dist<out[0]:out=(float(dist),name)
    return out
def main():
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--world',default=str(ROOT/'src/sim_env/worlds/structured_px4.world'))
    a=p.parse_args();d=Path(a.directory);shapes=world_shapes(a.world)
    rows=list(csv.DictReader(open(d/'telemetry.csv')))
    def col(k):return np.array([float(r[k]) if r[k] else float('nan') for r in rows])
    t=col('sim');t-=t[0];x,y,z=col('map_x'),col('map_y'),col('map_z')
    tx,ty,tz=col('truth_x'),col('truth_y'),col('truth_z')
    err=col('truth_error')
    res=json.loads((d/'result.json').read_text())
    active=[i for i,r in enumerate(rows) if r['armed']=='True' and tz[i]>.5]
    clear=[clearance([tx[i],ty[i],tz[i]],shapes) for i in active[::5]]
    if clear:
        nearest=min(clear,key=lambda v:v[0])
        res['geometry_evaluation']={'minimum_truth_center_surface_distance':nearest[0],'nearest_static_object':nearest[1],
           'note':'center-to-static-shape distance, not full vehicle collision/contact proof; floor excluded'}
    (d/'result.json').write_text(json.dumps(res,indent=2))
    fig,ax=plt.subplots(2,2,figsize=(12,9))
    for name,tr,kind,size in shapes:
        if kind!='box' or tr[2,3]-size[2]>1.5 or tr[2,3]+size[2]<.4:continue
        corners=np.array([[sx*size[0],sy*size[1],0,1] for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]]).T
        poly=(tr@corners)[:2,:].T
        ax[0,0].add_patch(Polygon(poly,color='lightgray'))
    ax[0,0].plot(tx,ty,label='Gazebo truth',color='black',linewidth=2)
    ax[0,0].plot(x,y,label='LIO estimate',color='tab:blue',alpha=.8)
    events=json.loads((d/'events.json').read_text())
    goals=[e['target'] for e in events if e['event']=='goal_sent']
    if goals:ax[0,0].scatter([g[0] for g in goals],[g[1] for g in goals],marker='x',color='red',label='Goals')
    margin=2
    ax[0,0].set_xlim(min(x.min(),tx.min())-margin,max(x.max(),tx.max())+margin)
    ax[0,0].set_ylim(min(y.min(),ty.min())-margin,max(y.max(),ty.max())+margin)
    ax[0,0].set_aspect('equal',adjustable='box');ax[0,0].legend();ax[0,0].set_title('Trajectory / static geometry')
    ax[0,1].plot(t,err);ax[0,1].axhline(1,color='red',linestyle='--',label='Flight abort threshold')
    ax[0,1].set_title('LIO position error against world truth [m]');ax[0,1].set_xlabel('Simulation time [s]')
    ax[1,0].plot(t,z,label='LIO');ax[1,0].plot(t,tz,label='Truth');ax[1,0].plot(t,col('cmd_z'),label='Planner command',alpha=.6)
    ax[1,0].set_title('Height [m]');ax[1,0].legend();ax[1,0].set_xlabel('Simulation time [s]')
    pts=[]
    with open(d/'map.pcd') as f:
        data=False
        for line in f:
            if data:pts.append([float(v) for v in line.split()])
            elif line.startswith('DATA'):data=True
    if pts:
        pts=np.array(pts);keep=(pts[:,2]>.25)&(pts[:,2]<1.8)
        ax[1,1].scatter(pts[keep,0],pts[keep,1],s=1,c=pts[keep,2],cmap='viridis')
        ax[1,1].set_aspect('equal',adjustable='box')
    ax[1,1].set_title('Accumulated LIO point cloud (0.15m capture voxels)')
    for axy in ax.flat:axy.grid(alpha=.25)
    if res.get('navigation_route_evaluated') is False:
        fig.suptitle('%s / component probe / accepted=%d / candidates=%d / route not evaluated'%(
            d.name,res.get('accepted_loop_log_lines',0),len(res.get('loop_messages',[]))))
    else:
        fig.suptitle('%s / success=%s / arrival=%sm / precision_0.3m=%s / loop_messages=%d'%(d.name,res['success'],res.get('arrival_tolerance','n/a'),res.get('precision_success','n/a'),len(res.get('loop_messages',[]))))
    fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(d/'overview.png',dpi=150);plt.close(fig)
    print(json.dumps({'output':str(d/'overview.png'),'geometry':res.get('geometry_evaluation')},indent=2))
if __name__=='__main__':main()
