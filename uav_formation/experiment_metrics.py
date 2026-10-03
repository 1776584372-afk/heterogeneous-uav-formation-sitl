"""Recompute flight metrics from raw telemetry.csv and frozen member limits."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from .experiment_config import load_config
from .experiment_artifacts import write_json


def compute_metrics(directory):
    directory=Path(directory)
    cfg,members=load_config(directory/'config.json')
    with (directory/'telemetry.csv').open() as stream:
        rows=[r for r in csv.DictReader(stream) if r['phase']=='track']
    if not rows: raise ValueError('no tracking telemetry; no metrics can be claimed')
    by_member={}
    synchronized={}
    for r in rows:
        synchronized.setdefault(float(r['t']),{})[int(r['member'])]=r
    distances=[]; shapes=[]
    for group in synchronized.values():
        if len(group)!=len(members): continue
        errors=[]
        for i in range(len(members)):
            for j in range(i+1,len(members)):
                a,b=group[i],group[j]
                d=math.hypot(float(a['x'])-float(b['x']),float(a['y'])-float(b['y']))
                distances.append(d); errors.append((d-abs(members[i].r_y-members[j].r_y))**2)
        if errors: shapes.append(float(np.mean(errors)))
    for m in members:
        rr=[r for r in rows if int(r['member'])==m.member_id]
        if len(rr)<3: raise ValueError('insufficient data for member %d'%m.member_id)
        def col(name): return np.array([float(r[name]) for r in rr])
        t=col('t'); weights=np.diff(t,append=t[-1]); duration=t[-1]-t[0]
        if duration<=0 or np.any(np.diff(t)<=0): raise ValueError('timestamps not increasing')
        vx,vy=col('vx'),col('vy'); speed=np.hypot(vx,vy)
        at=np.gradient(speed,t)
        ax,ay=np.gradient(vx,t),np.gradient(vy,t)
        alat=np.abs(vx*ay-vy*ax)/np.maximum(speed,.1)
        yawrate=np.gradient(np.unwrap(col('yaw')),t)
        roll=col('roll'); rollrate=np.gradient(np.unwrap(roll),t)
        stale=(col('pose_age')>.5)|(col('velocity_age')>.5)|(col('state_age')>3)
        actual_speed=col('airspeed') if m.platform=='fixedwing' else speed
        if m.platform=='fixedwing': stale |= col('airspeed_age')>.5
        checks=dict(speed_min=actual_speed<m.v_min-1e-3,speed_max=actual_speed>m.v_max+1e-3,
                    lateral_accel=alat>m.a_lat_max+1e-3,tangential_min=at<m.a_t_min-1e-3,
                    tangential_max=at>m.a_t_max+1e-3)
        if m.yaw_rate_max is not None: checks['heading_rate']=np.abs(yawrate)>m.yaw_rate_max+1e-3
        if m.bank_max is not None: checks['bank']=np.abs(roll)>m.bank_max+1e-3
        if m.bank_rate_max is not None: checks['bank_rate']=np.abs(rollrate)>m.bank_rate_max+1e-3
        squared=(col('x')-col('ref_x'))**2+(col('y')-col('ref_y'))**2
        by_member[m.member_id]=dict(planar_tracking_rmse=float(np.sqrt(np.sum(squared*weights)/duration)),
            max_tracking_error=float(np.sqrt(np.max(squared))),
            violations_time_fraction={name:float(np.sum(mask*weights)/duration) for name,mask in checks.items()},
            stale_time_fraction=float(np.sum(stale*weights)/duration),max_sample_gap=float(np.max(np.diff(t))),
            max_speed=float(np.max(actual_speed)),max_bank=float(np.max(np.abs(roll))),
            max_bank_rate=float(np.max(np.abs(rollrate))),mpc_failures=int(rr[-1]['mpc_fail_count']))
    status=json.loads((directory/'flight_status.json').read_text()) if (directory/'flight_status.json').exists() else {}
    result=dict(flight_completed=bool(status.get('completed',False)),
        tracking_duration=max(synchronized)-min(synchronized),members=by_member,
        shape_distance_rmse=float(np.sqrt(np.mean(shapes))) if shapes else None,
        minimum_separation=min(distances) if distances else None,
        selected_radius=float(rows[-1]['radius']),
        valid_telemetry=all(v['stale_time_fraction']==0 and v['max_sample_gap']<1 for v in by_member.values()),
        derivative_method='numpy.gradient at telemetry receive times; unwrapped yaw/roll; planar velocity',
        evaluation_window='track phase only, excluding takeoff/formation/settle/landing')
    write_json(directory/'metrics.json',result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('directory')
    args=p.parse_args(); print(json.dumps(compute_metrics(args.directory),indent=2))


if __name__=='__main__': main()
