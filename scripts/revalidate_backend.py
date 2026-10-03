#!/usr/bin/env python3
"""Bounded independent calibration batch; never starts formal paper trials."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from scripts.analyze_backend_dynamics import analyze


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--base-port',type=int,default=34200)
    args=parser.parse_args()
    out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=False)
    jobs=[]
    for direction in ('backward','left','right'):
        jobs.append((direction,['scripts/calibrate_sitl.py','--vehicle','copter',
            '--direction',direction,'--speeds','0,4,8,12,16,20','--ramp-rate','0.3','--ppva']))
    jobs.extend([
        ('plane_speed',['scripts/calibrate_sitl.py','--vehicle','plane',
            '--speeds','14,12,16,20,24,14','--ramp-rate','0.15']),
        ('copter_turn',['scripts/calibrate_turns.py','--vehicle','copter',
            '--speeds','12','--banks','7,12,16','--controller','course',
            '--accel-feedforward','--yaw-rate-feedforward']),
        ('plane_turn',['scripts/calibrate_turns.py','--vehicle','plane',
            '--speeds','14','--banks','7,12,14','--controller','course'])])
    report=dict(formal_experiments_started=False,capability_envelope_certified=False,jobs=[])
    for index,(name,command) in enumerate(jobs):
        target=out/name
        cmd=[sys.executable,*command,'--output',str(target),'--port',str(args.base_port+index*10)]
        print('START',name,flush=True)
        with (out/(name+'.stdout.log')).open('x') as log:
            completed=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=False)
        entry=dict(name=name,command=cmd,returncode=completed.returncode)
        if (target/'result.json').exists():
            entry['result']=json.loads((target/'result.json').read_text())
        if (target/'telemetry.csv').exists():
            try:
                diagnostic=analyze(target)
                (target/'dynamics.json').write_text(json.dumps(diagnostic,indent=2,allow_nan=False)+'\n')
            except Exception as exc:
                entry['analysis_error']=repr(exc)
        report['jobs'].append(entry)
        (out/'batch.json').write_text(json.dumps(report,indent=2)+'\n')
        print('FINISH',name,'returncode',completed.returncode,flush=True)
    return 0 if all(job['returncode']==0 for job in report['jobs']) else 1


if __name__=='__main__':
    raise SystemExit(main())
