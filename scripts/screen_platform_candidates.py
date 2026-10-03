#!/usr/bin/env python3
"""Predeclared development screening, not new capability certification."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

from scripts.analyze_backend_dynamics import analyze
from uav_formation.sitl_vehicle import AP, ROOT


def candidates():
    defaults=AP/'Tools/autotest/default_params'
    models=AP/'Tools/autotest/models'
    return {
        'freestyle': ('x:freestyle.json',str(models/'freestyle.parm')),
        'standard_x': ('x',','.join(str(p) for p in (defaults/'copter.parm',defaults/'copter-X.parm'))),
        'standard_x_45deg': ('x',','.join(str(p) for p in (defaults/'copter.parm',defaults/'copter-X.parm',
            ROOT/'config/experiments/standard_x_45deg.parm'))),
        'standard_x_position_gain': ('x',','.join(str(p) for p in (defaults/'copter.parm',defaults/'copter-X.parm',
            ROOT/'config/experiments/standard_x_45deg.parm',ROOT/'config/experiments/standard_x_position_gain.parm'))),
        'callisto': ('octa-quad-cwx:Callisto.json',','.join(str(p) for p in (
            defaults/'copter.parm',defaults/'copter-octaquad.parm',models/'Callisto.parm'))),
    }


def jobs(stage):
    result=[]
    if stage=='rotor_straight':
        for name,(model,defaults) in candidates().items():
            result.append((name,['scripts/calibrate_sitl.py','--vehicle','copter','--model',model,
                '--defaults',defaults,'--speeds','12,14','--ramp-rate','.3','--ppva','--continuous-position']))
    elif stage=='rotor_reference':
        model,defaults=candidates()['freestyle']
        result.append(('freestyle_quintic',['scripts/calibrate_sitl.py','--vehicle','copter',
            '--model',model,'--defaults',defaults,'--speeds','12,14,12','--ramp-rate','.1',
            '--ramp-shape','quintic','--ppva','--continuous-position']))
    elif stage=='rotor_low_rate':
        model,defaults=candidates()['freestyle']
        result.append(('freestyle_quintic_low_rate',['scripts/calibrate_sitl.py','--vehicle','copter',
            '--model',model,'--defaults',defaults,'--speeds','12,13,12','--ramp-rate','.05',
            '--ramp-shape','quintic','--ppva','--continuous-position']))
    elif stage=='rotor_turn':
        for name,(model,defaults) in candidates().items():
            result.append((name,['scripts/calibrate_turns.py','--vehicle','copter','--model',model,
                '--defaults',defaults,'--speeds','12','--banks','7,12','--controller','course',
                '--accel-feedforward','--yaw-rate-feedforward','--continue-after-tracking-failure']))
    elif stage=='plane_heading':
        for name,period in [('stream',0.),('scheduled_5hz',.2)]:
            result.append((name,['scripts/calibrate_turns.py','--vehicle','plane','--speeds','14',
                '--banks','7','--controller','course','--path-guidance',
                '--continue-after-tracking-failure','--plane-heading-period',str(period)]))
    elif stage in ('plane_preview','plane_handover'):
        name='preview_60m_handover' if stage=='plane_handover' else 'preview_60m'
        result.append((name,['scripts/calibrate_turns.py','--vehicle','plane','--speeds','14',
            '--banks','7','--controller','course','--path-guidance','--lookahead','60',
            '--continue-after-tracking-failure',
            *(['--handover-command-speed'] if stage=='plane_handover' else [])]))
    elif stage=='plane_speed_config':
        for name,extra in [('baseline',[]),('speedweight_zero',['--defaults',','.join([
            str(AP/'Tools/autotest/models/plane.parm'),str(ROOT/'config/experiments/plane_speedweight_zero.parm')])]),
            ('speed_filter',['--defaults',','.join([str(AP/'Tools/autotest/models/plane.parm'),
                str(ROOT/'config/experiments/plane_speed_filter.parm')])]),
            ('speed_combined',['--defaults',','.join([str(AP/'Tools/autotest/models/plane.parm'),
                str(ROOT/'config/experiments/plane_speed_combined.parm')])])]:
            result.append((name,['scripts/calibrate_sitl.py','--vehicle','plane','--speeds','14,16,14',
                '--ramp-rate','.05',*extra]))
    elif stage=='plane_low_rate':
        result.append(('plane_low_rate',['scripts/calibrate_sitl.py','--vehicle','plane','--speeds','14,15,14',
            '--ramp-rate','.02']))
    elif stage=='plane_speed':
        for name,period in [('stream',0.),('scheduled_5hz',.2)]:
            result.append((name,['scripts/calibrate_sitl.py','--vehicle','plane','--speeds','14,16,14',
                '--ramp-rate','.05','--plane-heading-period',str(period)]))
    else:
        raise ValueError('unknown screening stage: '+stage)
    return result


def save(path,data):
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


def execute(command,stream):
    process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        return process.wait(timeout=900)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=130)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',choices=['rotor_straight','rotor_reference','rotor_low_rate','rotor_turn','plane_heading','plane_speed','plane_preview','plane_handover','plane_speed_config','plane_low_rate'],required=True)
    p.add_argument('--only',help='optional candidate name')
    p.add_argument('--output',required=True)
    p.add_argument('--base-port',type=int,default=36000)
    args=p.parse_args()
    selected=[(name,command) for name,command in jobs(args.stage) if args.only in (None,name)]
    if not selected: p.error('unknown candidate')
    root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=False)
    frozen=ROOT/'runs/paper_hetero/frozen_v1/capabilities_verified.yaml'
    before=hashlib.sha256(frozen.read_bytes()).hexdigest()
    report=dict(stage=args.stage,development_only=True,envelope_certified=False,formal_qualification=False,
        frozen_sha256_before=before,planned_jobs=selected,jobs=[],
        interpretation='steady tracking and actual dynamics assessed separately against unchanged v1 limits')
    save(root/'batch.json',report)
    shutil.copy2(Path(__file__),root/'screening_source.py')
    for index,(name,command) in enumerate(selected):
        if shutil.disk_usage(root).free<700*1024**2:
            report['stopped_reason']='less than 700 MiB disk reserve';break
        target=root/name
        cmd=[sys.executable,*command,'--output',str(target),'--port',str(args.base_port+index*10)]
        print('START',name,flush=True)
        entry=dict(name=name,command=cmd)
        try:
            with (root/(name+'.stdout.log')).open('x') as stream:
                entry['returncode']=execute(cmd,stream)
            entry['result']=json.loads((target/'result.json').read_text())
            if (target/'telemetry.csv').exists():
                diagnostics=analyze(target)
                save(target/'dynamics.json',diagnostics)
                entry['dynamics_path']=str(target/'dynamics.json')
        except Exception as exc:
            entry['error']=repr(exc)
        report['jobs'].append(entry)
        report['frozen_sha256_after']=hashlib.sha256(frozen.read_bytes()).hexdigest()
        report['frozen_unchanged']=report['frozen_sha256_after']==before
        save(root/'batch.json',report)
        print('FINISH',name,entry.get('returncode'),entry.get('error',''),flush=True)
    save(root/'batch.json',report)
    return 0 if len(report['jobs'])==len(selected) and all(j.get('returncode')==0 and 'error' not in j for j in report['jobs']) else 1


if __name__=='__main__': raise SystemExit(main())
