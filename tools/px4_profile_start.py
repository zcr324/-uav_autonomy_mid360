#!/usr/bin/env python3
"""Generate a run-local PX4 rcS; never edit the installed PX4 tree."""
import hashlib,json,os,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
args=sys.argv[1:]
mode=os.environ.get('UAV_ESTIMATOR','gps')
if mode not in ('gps','external_vision'):raise SystemExit('Unknown UAV_ESTIMATOR '+mode)
work=Path(args[args.index('-w')+1]).resolve()
try:work.relative_to(root/'integration')
except ValueError:raise SystemExit('PX4 workdir must stay inside workspace integration/')
work.mkdir(parents=True,exist_ok=True)
source=Path(os.environ['PX4_ROOT'])/'build/px4_sitl_default/etc/init.d-posix/rcS'
text=source.read_text()
needle='. px4-rc.params'
if text.count(needle)!=1:raise SystemExit('Installed PX4 rcS changed; inspect its parameter hook before using profiles')
values={'EKF2_AID_MASK':24,'EKF2_HGT_MODE':3,'EKF2_EV_DELAY':0} if mode=='external_vision' else {'EKF2_AID_MASK':1,'EKF2_HGT_MODE':0,'EKF2_EV_DELAY':175}
values['COM_RCL_EXCEPT']=4  # SITL: allow OFFBOARD without a physical RC link.
text=text.replace(needle,needle+'\n# Framework estimator profile, applied before EKF startup.\n'+'\n'.join('param set %s %s'%kv for kv in values.items()))
startup=work/'framework_rcS'
startup.write_text(text)
(work/'estimator_profile.json').write_text(json.dumps({'mode':mode,'parameters':values,'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()},indent=2))
args[args.index('-s')+1]=str(startup)
os.execv(args[0],args)
