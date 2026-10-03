#!/usr/bin/env python3
"""Run one selected measured-capability experiment and produce its metrics/plots."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import numpy as np

import yaml

from uav_formation.experiment_artifacts import ROOT, freeze_run, export_plan
from uav_formation.experiment_config import load_config, coordinator_from_config
from uav_formation.progress_profile import ProgressProfile, PlanningError, validate_profile
from uav_formation.paper_metrics import compute
from scripts.plot_paper_run import plot


def available_port(count):
    for base in range(28000,30000,40):
        sockets=[]
        try:
            for member in range(count):
                for offset in (0,2,3):
                    s=socket.socket(); sockets.append(s)
                    s.bind(('127.0.0.1',base+10*member+offset))
            return base
        except OSError:
            continue
        finally:
            for s in sockets:
                s.close()
    raise RuntimeError('no free local SITL port group')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',choices=['s1','s2','s3','s3_swap'])
    parser.add_argument('--method',choices=['F','A3','C'],default='F')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--speedup',type=float,default=4.)
    parser.add_argument('--output')
    parser.add_argument('--plan-only',action='store_true')
    args=parser.parse_args()
    cfg,members=load_config(ROOT/'runs/paper_hetero/frozen_v1'/(args.scene.split('_')[0]+'.yaml'))
    cfg=deepcopy(cfg)
    if args.scene=='s3_swap':
        cfg['members'][0]['r_y'],cfg['members'][2]['r_y']=cfg['members'][2]['r_y'],cfg['members'][0]['r_y']
    if args.method=='C':
        cfg['planner'].update(nominal_radius=320.,radii=[320.],nominal_speed=14.,start_speed=14.,end_speed=14.)
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    directory=Path(args.output).resolve() if args.output else ROOT/'runs'/('paper_demo_'+args.scene+'_'+args.method+'_'+stamp)
    directory.mkdir(parents=True,exist_ok=False)
    if not args.plan_only and shutil.disk_usage(directory).free<700*1024**2:
        raise RuntimeError('less than 700 MiB free; choose a local filesystem with enough recording space')
    config=directory/'request.yaml'; config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    run=directory/'flight'
    if args.plan_only:
        cfg,members=load_config(config); cfg['method']=args.method
        target=freeze_run(directory/'plan',cfg,'reference_only')
        result=coordinator_from_config(cfg,members,nominal_only=args.method!='F').plan(cfg['waypoints'])
        if args.method=='C' and result.feasible:
            p=result.profile
            try:
                constant=ProgressProfile(p.s,np.full(len(p.s),cfg['planner']['nominal_speed']**2),
                    p.team_lower,p.team_upper,p.active_lower,p.active_upper,{})
                constant.margins=validate_profile(result.route,members,constant)
                result.profile=constant
                result.candidates[0].duration=float(constant.time[-1])
                result.candidates[0].margins=constant.margins
            except PlanningError as exc:
                result.candidates[0].status='CONSTANT_PROGRESS_REJECTED'
                result.candidates[0].detail=exc.detail
                result.route=result.profile=result.selected_radius=None
        export_plan(target,result,members)
        print('Verified reference:',result.feasible,'radius:',result.selected_radius)
        print('Artifacts:',directory)
        return 0 if result.feasible else 2
    else:
        cmd=[sys.executable,str(ROOT/'scripts/run_paper_sitl.py'),'--config',str(config),'--output',str(run),
             '--base-port',str(available_port(len(members))),'--speedup',str(args.speedup),'--seed',str(args.seed),'--method',args.method]
    code=subprocess.call(cmd,cwd=ROOT)
    if not args.plan_only and (run/'flight_status.json').exists():
        metrics=compute(run)
        plot(run)
        print('Task completed:',metrics['completed'])
        print('Measured constraint violations are recorded in:',run/'metrics.json')
    print('Artifacts:',directory)
    return code


if __name__=='__main__':
    raise SystemExit(main())
