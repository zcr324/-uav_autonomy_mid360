#!/usr/bin/env python3
"""Plot raw and optimized keyframe trajectories in the SAME camera_init frame."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def main():
    p=argparse.ArgumentParser();p.add_argument('run_directory');a=p.parse_args();d=Path(a.run_directory)
    raw=np.atleast_2d(np.loadtxt(d/'ltaom/odom_poses.txt'))
    opt=np.atleast_2d(np.loadtxt(d/'ltaom/optimized_poses.txt'))
    n=min(len(raw),len(opt));raw=raw[:n,1:4];corrected=opt[:n,1:4]
    delta=np.linalg.norm(raw-corrected,axis=1)
    loopfile=d/'ltaom/loopclosure.txt'
    loops=[l for l in loopfile.read_text().splitlines() if l.strip()] if loopfile.exists() else []
    result={'matched_keyframes':n,'maximum_pose_change':float(delta.max()),'mean_pose_change':float(delta.mean()),
        'loop_log_lines':len(loops),'frame':'camera_init',
        'interpretation':'Trajectory change may include adjacent keypoint factors; accepted non-neighbor loops and truth comparison are required to claim loop benefit'}
    (d/'backend_comparison.json').write_text(json.dumps(result,indent=2))
    fig,ax=plt.subplots(1,2,figsize=(12,5))
    ax[0].plot(raw[:,0],raw[:,1],'.-',label='Raw keyframes')
    ax[0].plot(corrected[:,0],corrected[:,1],'.-',label='PGO keyframes')
    ax[0].set_aspect('equal',adjustable='datalim');ax[0].legend();ax[0].set_title('Same-frame trajectory comparison')
    ax[1].plot(delta);ax[1].set_title('Position change [m]');ax[1].set_xlabel('Keyframe index')
    for x in ax:x.grid(alpha=.3)
    fig.suptitle('PGO output / loop log lines=%d (inspect accepted factors separately)'%len(loops))
    fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(d/'backend_comparison.png',dpi=150)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
