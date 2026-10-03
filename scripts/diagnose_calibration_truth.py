#!/usr/bin/env python3
"""Compare simulator truth with telemetry for each recorded steady calibration window."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from pymavlink import mavutil

from scripts.analyze_backend_dynamics import distribution
from uav_formation.paper_metrics import signal_derivative


def acceleration_stats(samples, limit):
    samples=np.asarray(samples,dtype=float)
    if samples.ndim!=2 or samples.shape[1]!=3 or len(samples)<3:
        return {'unavailable':'at least three timestamped horizontal velocity samples required'}
    try:
        speed=np.hypot(samples[:,1],samples[:,2])
        acceleration=signal_derivative(speed,samples[:,0])
        result=distribution(acceleration,samples[:,0],-limit,limit)
        result['minimum_time_s']=float(samples[np.argmin(acceleration),0])
        result['maximum_time_s']=float(samples[np.argmax(acceleration),0])
        return result
    except ValueError as exc:
        return {'unavailable':str(exc)}


def compare_window(truth, rows, window, limit):
    lo,hi=window
    selected=truth[(truth[:,0]>=lo)&(truth[:,0]<=hi)]
    recorded=[r for r in rows if lo<=float(r['velocity_sim_s'])<=hi]
    measured=np.array([[float(r['velocity_sim_s']),float(r['ve']),float(r['vn'])] for r in recorded])
    entry=dict(window=[lo,hi],simulator_truth=acceleration_stats(selected,limit),
               telemetry=acceleration_stats(measured,limit))
    entry['telemetry_peak_context']={}
    for extremum in ('minimum','maximum'):
        timestamp=entry['telemetry'].get(extremum+'_time_s')
        if timestamp is None:
            continue
        row=min(recorded,key=lambda r:abs(float(r['velocity_sim_s'])-timestamp))
        context=dict(velocity_time_s=timestamp,elapsed_since_window_start_s=timestamp-lo,
            context_attitude_time_s=float(row['sim_s']),
            groundspeed_m_s=float(np.hypot(float(row['ve']),float(row['vn']))))
        for key in ('phase','ref_speed','ref_path_course','ref_course','sent_ve','sent_vn',
                    'sent_ae','sent_an','sent_yaw_rate','roll','pitch','throttle','alt'):
            context[key]=row.get(key)
        entry['telemetry_peak_context'][extremum]=context
    return entry


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run');p.add_argument('--output',required=True)
    args=p.parse_args();root=Path(args.run)
    report=json.loads((root/'result.json').read_text())
    platform=json.loads((root/'manifest.json').read_text())['vehicle']
    truth=[]
    bins=sorted((root/'logs').glob('*.BIN'))
    if not bins:raise ValueError('onboard simulation logs missing')
    for path in bins:
        reader=mavutil.mavlink_connection(str(path))
        try:
            while True:
                msg=reader.recv_match(type='SIM2')
                if msg is None:break
                truth.append([msg.TimeUS/1e6,msg.VE,msg.VN])
        finally:reader.close()
    truth=np.asarray(truth).reshape((-1,3))
    with (root/'telemetry.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    result=dict(source=str(root.resolve()),primary_acceptance_unchanged=True,
        definition='SIM2 unique source samples, no smoothing; independent telemetry comparison',
        context_alignment='nearest logged velocity stamp; attitude/commands retain their record time, not a causal attribution',
        windows=[],phases={})
    limit=.2 if platform=='plane' else .4
    for step in report['steps']:
        entry=compare_window(truth,rows,step['window'],limit)
        entry.update(speed=step['speed'],bank=step.get('bank'),sign=step.get('sign'))
        result['windows'].append(entry)
    for phase in sorted({r['phase'] for r in rows if r['phase'].startswith(('step_','turn_'))}):
        recorded=[r for r in rows if r['phase']==phase]
        window=[float(recorded[0]['velocity_sim_s']),float(recorded[-1]['velocity_sim_s'])]
        result['phases'][phase]=compare_window(truth,recorded,window,limit)
    result['source_sha256']={}
    for path in [*bins,root/'telemetry.csv',root/'result.json',Path(__file__)]:
        digest=hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
        result['source_sha256'][str(path.resolve())]=digest.hexdigest()
    with Path(args.output).open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
