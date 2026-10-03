#!/usr/bin/env python3
"""Predeclared single-aircraft follow-ups, preserving all failed trials."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from scripts.analyze_backend_dynamics import analyze
from scripts.audit_measured_feedback import audit


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    p.add_argument('--ramp-rate',type=float,default=.15)
    p.add_argument('--radius',type=float,default=240.)
    args=p.parse_args();root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=False)
    cfg=json.loads(Path('config/experiments/backend_single_rotor_turn.json').read_text())
    cfg['name']='development_single_rotor_radius_%g'%args.radius
    cfg['planner']['nominal_radius']=args.radius;cfg['planner']['radii']=[args.radius]
    config_path=root/'rotor_radius.json';config_path.write_text(json.dumps(cfg,indent=2)+'\n')
    jobs=[]
    for shape in ('linear','quintic'):
        jobs.append(('plane_'+shape,['scripts/calibrate_sitl.py','--vehicle','plane',
            '--speeds','14,16,14','--ramp-rate',str(args.ramp_rate),'--ramp-shape',shape],False))
    jobs.append(('plane_path_turn',['scripts/calibrate_turns.py','--vehicle','plane',
        '--speeds','14','--banks','7,12,14','--controller','course','--path-guidance',
        '--continue-after-tracking-failure'],False))
    jobs.append(('rotor_tangent_r%g'%args.radius,['scripts/run_paper_sitl.py','--config',str(config_path),
        '--development','--speedup','2','--copter-yaw-rate','--copter-synchronous',
        '--copter-analytic-gradient','--copter-tangent-yaw'],True))
    report=dict(development_only=True,formal_experiments_started=False,jobs=[])
    for index,(name,command,fleet) in enumerate(jobs):
        if shutil.disk_usage(root).free<300*1024**2:
            report['stopped_reason']='insufficient disk reserve';break
        target=root/name
        cmd=[sys.executable,*command,'--output',str(target),
             '--base-port' if fleet else '--port',str(34600+index*10)]
        print('START',name,flush=True)
        with (root/(name+'.stdout.log')).open('x') as stream:
            run=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT,check=False)
        entry=dict(name=name,command=cmd,returncode=run.returncode)
        try:
            if fleet and not (target/'flight_status.json').exists():
                entry['candidate_rejection']=json.loads((target/'candidates.json').read_text())
            elif fleet:
                with (root/(name+'.audit.log')).open('x') as stream:
                    check=subprocess.run([sys.executable,'scripts/audit_tracking_run.py',str(target)],
                        stdout=stream,stderr=subprocess.STDOUT,check=False)
                entry['audit_returncode']=check.returncode
                entry['feedback']=audit(target)
                entry['acceptance']=json.loads((target/'acceptance.json').read_text())
            else:
                (target/'dynamics.json').write_text(json.dumps(analyze(target),indent=2,allow_nan=False)+'\n')
                entry['result']=json.loads((target/'result.json').read_text())
        except Exception as exc:
            entry['analysis_error']=repr(exc)
        report['jobs'].append(entry)
        (root/'batch.json').write_text(json.dumps(report,indent=2)+'\n')
        print('FINISH',name,'returncode',run.returncode,flush=True)
    (root/'batch.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if len(report['jobs'])==len(jobs) and all(j['returncode']==0 for j in report['jobs']) else 1


if __name__=='__main__':raise SystemExit(main())
