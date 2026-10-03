#!/usr/bin/env python3
"""Same-route, single-Copter control-period diagnosis; development evidence only."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from scripts.audit_measured_feedback import audit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--base-port',type=int,default=34400)
    args=p.parse_args()
    root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=False)
    result=dict(development_only=True,formal_qualification=False,runs=[])
    for index,period in enumerate((1/3,.1,.05)):
        if shutil.disk_usage(root).free<300*1024**2:
            result['stopped_reason']='less than 300 MiB free; existing evidence preserved'
            break
        name=('period_%.6f'%period).replace('.','_');out=root/name
        cmd=[sys.executable,'scripts/run_paper_sitl.py','--config','config/experiments/backend_single_rotor_turn.json',
            '--output',str(out),'--base-port',str(args.base_port+index*10),'--development','--speedup','2',
            '--copter-yaw-rate','--copter-synchronous','--copter-analytic-gradient',
            '--copter-control-period',str(period)]
        print('START',name,flush=True)
        with (root/(name+'.stdout.log')).open('x') as log:
            run=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=False)
        entry=dict(period_s=period,command=cmd,returncode=run.returncode)
        if (out/'flight_status.json').exists():
            with (root/(name+'.audit.log')).open('x') as log:
                audit_run=subprocess.run([sys.executable,'scripts/audit_tracking_run.py',str(out)],
                                         stdout=log,stderr=subprocess.STDOUT,check=False)
            entry['audit_returncode']=audit_run.returncode
            try:
                entry['feedback']=audit(out)
            except Exception as exc:
                entry['feedback_error']=repr(exc)
            if (out/'acceptance.json').exists():
                entry['acceptance']=json.loads((out/'acceptance.json').read_text())
        result['runs'].append(entry)
        (root/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
        print('FINISH',name,'returncode',run.returncode,
              'actual_constraints',entry.get('acceptance',{}).get('measured_constraints_passed'),flush=True)
    (root/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if len(result['runs'])==3 and all(
        r.get('acceptance',{}).get('measured_constraints_passed') and
        r.get('acceptance',{}).get('tracking_qualified') for r in result['runs']) else 1


if __name__=='__main__':raise SystemExit(main())
