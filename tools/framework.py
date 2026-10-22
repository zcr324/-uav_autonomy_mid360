#!/usr/bin/env python3
"""One entry point for framework profiles and reproducible experiment artifacts.
start -> baseline -> run --route NAME -> report -> stop
All child commands source only env_uav_ws.bash. No arming happens in start.
"""
import argparse,datetime,json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
CONFIG=json.loads((ROOT/'config/experiments/profiles.json').read_text())
CURRENT=ROOT/'integration/current_run.txt'
def envrun(args,**kwargs):
    return subprocess.run(['bash','-c','source "$1"; shift; exec "$@"','uav-framework',str(ROOT/'env_uav_ws.bash')]+list(args),**kwargs)
def directory(a):
    p=Path(a.run_dir) if a.run_dir else Path(CURRENT.read_text().strip())
    return p.resolve()
def start(a):
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    d=(ROOT/'integration/runs'/stamp).resolve()
    # Validate no live old simulator before allocating a run directory.
    occupied=subprocess.check_output(['ss','-H','-ltn','sport = :4560'],text=True).strip()
    if occupied:raise RuntimeError('TCP4560 occupied; stop the existing framework simulation first')
    d.mkdir(parents=True);(d/'ltaom').mkdir()
    settings={**CONFIG['profiles'][a.profile],**CONFIG['scenes'][a.scene]}
    settings['world']=str(ROOT/'src/sim_env/worlds'/settings['world'])
    settings['gui']=not a.headless
    settings['px4_workdir']=str(d/'px4')
    settings['save_dir']=str(d/'ltaom')+'/'
    settings['lcd_config']=str(ROOT/'src/lio/STD/config/config_mid360.yaml')
    args=[str(ROOT/'tools/run_sim.sh')]+['%s:=%s'%(k,str(v).lower() if isinstance(v,bool) else v) for k,v in settings.items()]
    log=open(d/'simulation.log','w')
    p=subprocess.Popen(['bash','-c','source "$1"; shift; exec bash "$@"','uav-framework',str(ROOT/'env_uav_ws.bash')]+args,
                       stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (d/'launch.json').write_text(json.dumps({'profile':a.profile,'scene':a.scene,'settings':settings,'pid':p.pid,'command':args},indent=2))
    CURRENT.write_text(str(d)+'\n')
    print(json.dumps({'run_dir':str(d),'pid':p.pid,'next':'framework.py baseline'},indent=2))
def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    s=sub.add_parser('start');s.add_argument('--profile',choices=list(CONFIG['profiles']),default='navigation')
    s.add_argument('--scene',choices=list(CONFIG['scenes']),default='structured');s.add_argument('--headless',action='store_true')
    for name in ['baseline','run','report','status','capture']:
        s=sub.add_parser(name);s.add_argument('--run-dir')
        if name=='run':
            s.add_argument('--route',choices=list(json.loads((ROOT/'config/experiments/routes.json').read_text())),required=True)
            s.add_argument('--arrival',type=float,default=.3)
            s.add_argument('--yaw-scan',type=float,default=0)
            s.add_argument('--reference-scan',type=float,default=0)
            s.add_argument('--rotation-revisits',type=int,default=0)
            s.add_argument('--budget',type=float,default=1200)
            s.add_argument('--label',help='unique experiment directory label for an explicit retry')
    sub.add_parser('stop')
    sub.add_parser('view')
    a=p.parse_args()
    if a.command=='start':start(a);return 0
    if a.command=='view':return envrun(['rviz','-d',str(ROOT/'config/experiments/mapping.rviz')]).returncode
    if a.command=='stop':return envrun(['python3',str(ROOT/'tools/stop_owned_sim.py')]).returncode
    d=directory(a)
    if a.command=='capture':return envrun(['python3',str(ROOT/'tools/capture_backend.py'),'--output',str(d/'backend_capture')]).returncode
    if a.command=='status':
        print('Run:',d)
        for f in sorted(d.glob('*/result.json')):
            r=json.loads(f.read_text());print(f.parent.name,'PASS' if r['success'] else 'FAIL',r.get('error',''),r.get('metrics',{}))
        return 0
    if a.command=='report':
        for f in sorted(d.glob('*/result.json')):
            if (f.parent/'telemetry.csv').exists():
                world=json.loads((d/'launch.json').read_text())['settings']['world'] if (d/'launch.json').exists() else str(ROOT/'src/sim_env/worlds/structured_px4.world')
                code=envrun(['python3',str(ROOT/'tools/analyze_experiment.py'),str(f.parent),'--world',world]).returncode
                if code:return code
        return 0
    label='baseline' if a.command=='baseline' else (a.label or a.route)
    if Path(label).name!=label or label in ('.','..'):raise RuntimeError('Invalid experiment label')
    output=d/label
    if (output/'result.json').exists():raise RuntimeError('Result already exists; preserve it and start a new run')
    if a.command=='run':
        base=d/'baseline/result.json'
        if not base.exists() or not json.loads(base.read_text())['success']:
            raise RuntimeError('This run has no passed static baseline; flight is gated')
        metadata=json.loads((d/'launch.json').read_text()) if (d/'launch.json').exists() else None
        if metadata:
            route=json.loads((ROOT/'config/experiments/routes.json').read_text())[a.route]
            if Path(metadata['settings']['world']).name!=route['world']:raise RuntimeError('Route/world mismatch')
            if not metadata['settings']['control']:raise RuntimeError('Selected profile has control disabled')
    args=['python3',str(ROOT/'tools/experiment.py'),'baseline' if a.command=='baseline' else 'flight','--output',str(output)]
    if a.command=='run':args+=['--route',a.route,'--arrival',str(a.arrival),'--yaw-scan',str(a.yaw_scan),'--reference-scan',str(a.reference_scan),'--rotation-revisits',str(a.rotation_revisits),'--budget',str(a.budget)]
    return envrun(args).returncode
if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print('Framework error:',e,file=sys.stderr);sys.exit(2)
