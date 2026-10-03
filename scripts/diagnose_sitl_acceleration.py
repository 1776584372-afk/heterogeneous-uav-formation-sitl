#!/usr/bin/env python3
"""Exploratory simulator-truth check; never replace frozen primary metrics."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pymavlink import mavutil

from uav_formation.experiment_config import load_config
from uav_formation.experiment_artifacts import write_json
from uav_formation.paper_metrics import signal_derivative
from scripts.paper_batch import file_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run'); parser.add_argument('--member',type=int,required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args(); root=Path(args.run).resolve(); out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=False)
    _,members=load_config(root/'config.json'); member=members[args.member]
    with (root/'flight.csv').open() as stream:
        rows=[r for r in csv.DictReader(stream) if r['phase']=='track' and int(r['member'])==args.member]
    times=np.array([float(r.get('velocity_t',r.get('attitude_t'))) for r in rows])
    measured=np.array([np.hypot(float(r['vx']),float(r['vy'])) for r in rows])
    estimated_accel=signal_derivative(measured,times)
    bins=sorted((root/('vehicle_%d'%args.member)/'logs').glob('*.BIN'))
    if not bins:
        raise ValueError('unpack the full raw evidence first')
    truth=[]; attitude_truth=[]
    for path in bins:
        reader=mavutil.mavlink_connection(str(path))
        while True:
            msg=reader.recv_match(type=['SIM2','SIM'])
            if msg is None:
                break
            t=msg.TimeUS/1e6
            if t>times[-1]:
                break
            if t>=times[0]:
                if msg.get_type()=='SIM2':
                    truth.append([t,msg.VE,msg.VN,msg.As,msg.ASpdU,msg.UFC])
                else:
                    attitude_truth.append([t,np.radians(msg.Yaw)])
        reader.close()
    truth=np.asarray(truth)
    if len(truth)<3:
        raise ValueError('insufficient SIM2 truth samples')
    accel=signal_derivative(np.hypot(truth[:,1],truth[:,2]),truth[:,0])
    def stats(t,a):
        w=np.diff(t,append=t[-1]); duration=t[-1]-t[0]
        return dict(samples=len(t),minimum=float(min(a)),maximum=float(max(a)),
            lower_violation_fraction=float(np.dot(a<member.a_t_min-1e-3,w)/duration),
            upper_violation_fraction=float(np.dot(a>member.a_t_max+1e-3,w)/duration))
    with (out/'simulator_truth.csv').open('w',newline='') as stream:
        writer=csv.writer(stream); writer.writerow(['boot_time','ve','vn','airspeed','speedup','serial_pause_count','tangential_accel'])
        writer.writerows(np.column_stack([truth,accel]))
    with (out/'estimator.csv').open('w',newline='') as stream:
        writer=csv.writer(stream); writer.writerow(['boot_time','horizontal_speed','tangential_accel'])
        writer.writerows(np.column_stack([times,measured,estimated_accel]))
    result=dict(exploratory=True,primary_metrics_unchanged=True,run=str(root),member=args.member,
        window_boot_time=[float(times[0]),float(times[-1])],bounds=[member.a_t_min,member.a_t_max],
        source='SIM2 onboard simulation truth: SIM_Aircraft.cpp WriteStreaming fields VN/VE; same boot clock',
        procedure='unique-sample finite difference without smoothing; compare each source on its own time grid',
        estimator=stats(times,estimated_accel),simulator_truth=stats(truth[:,0],accel),
        binary_sha256={str(path):file_hash(path) for path in bins})
    if member.yaw_rate_max is not None and len(attitude_truth)>=3:
        def yaw_stats(t,yaw):
            rate=signal_derivative(np.unwrap(yaw),t)
            w=np.diff(t,append=t[-1])
            return dict(maximum_abs_rad_s=float(max(abs(rate))),
                violation_fraction=float(np.dot(abs(rate)>member.yaw_rate_max+1e-3,w)/(t[-1]-t[0])))
        tt=np.asarray(attitude_truth)
        result['yaw_rate']=dict(bound_rad_s=member.yaw_rate_max,
            estimator=yaw_stats(np.array([float(r['attitude_t']) for r in rows]),
                                np.array([float(r['yaw']) for r in rows])),
            simulator_truth=yaw_stats(tt[:,0],tt[:,1]),
            source='SIM.Yaw in degrees, converted to radians and unwrapped; NED/ENU sign irrelevant to absolute bound')
        with (out/'simulator_yaw.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['boot_time','yaw_ned_rad'])
            writer.writerows(tt)
    write_json(out/'diagnostic.json',result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
