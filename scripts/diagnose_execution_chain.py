#!/usr/bin/env python3
"""Align existing flight references, sent commands and responses without smoothing."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from scripts.analyze_backend_dynamics import distribution
from uav_formation.experiment_config import load_config
from uav_formation.paper_metrics import signal_derivative


def command_heading_derivative(rows):
    """Commands use the flight-loop sample clock, not the held attitude clock.

    This is a sampled diagnostic, not a packet-send timestamp or a continuous
    acceleration/heading-rate certificate.
    """
    times = np.array([float(row['t']) for row in rows])
    headings = np.unwrap([float(row['command_yaw']) for row in rows])
    return signal_derivative(headings, times)


def geometry_phase_masks(kappa, kappa_prime):
    """Each track row belongs to exactly one geometry phase."""
    transition = abs(np.asarray(kappa_prime)) > 1e-8
    straight = (abs(np.asarray(kappa)) < 1e-8) & ~transition
    return dict(straight=straight, transition=transition,
                constant_turn=~(straight | transition))


def measured_airspeed_metrics(rows, lower, upper):
    """Keep airspeed separate; never substitute groundspeed or a missing clock."""
    if not all(row.get('airspeed') not in (None, '')
               and row.get('airspeed_t') not in (None, '') for row in rows):
        return {}, {'unavailable': 'missing airspeed or associated reception clock'}
    times = np.array([float(row['airspeed_t']) for row in rows])
    speeds = np.array([float(row['airspeed']) for row in rows])
    summary = dict(speed=distribution(speeds, times, lower, upper),
                   clock='VFR_HUD has no source time; latest ATTITUDE time at reception')
    data = dict(airspeed_t=times, actual_airspeed=speeds)
    if len(np.unique(times)) >= 3:
        data['actual_airspeed_derivative'] = signal_derivative(speeds, times)
        summary['derivative'] = distribution(data['actual_airspeed_derivative'], times)
    else:
        summary['derivative'] = {'unavailable': 'fewer than three distinct associated timestamps'}
    return data, summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run');parser.add_argument('--output',required=True)
    args=parser.parse_args()
    root=Path(args.run).resolve();out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=False)
    cfg,members=load_config(root/'config.json')
    with (root/'flight.csv').open() as stream:
        flight=list(csv.DictReader(stream))
    result=dict(schema_version=2,run=str(root),reference_and_limits_unchanged=True,
        caution='observational alignment, not causal identification; unique timestamp gradients, no smoothing',
        signal_clocks=dict(reference='flight.csv t',recorded_commands='flight.csv t, not packet-send timestamps',
            actual_groundspeed='velocity_t',actual_yaw_rate='attitude_t',
            actual_airspeed='airspeed_t, latest ATTITUDE clock associated at VFR_HUD reception'),
        sent_field_scope='legacy sent_* names refer to recorded command samples, not independently captured outgoing packets',
        airspeed_scope='command_speed is target airspeed for fixedwing; actual_accel is horizontal groundspeed derivative',
        formal_qualification=False,
        members={},source_sha256={})
    for member in members:
        rows=[r for r in flight if r['phase']=='track' and int(r['member'])==member.member_id]
        if len(rows)<3:
            raise ValueError('member %d has fewer than three track rows'%member.member_id)
        def col(key):return np.array([float(r[key]) for r in rows])
        t,ta,tv=col('t'),col('attitude_t'),col('velocity_t')
        actual_speed=np.hypot(col('vx'),col('vy'))
        at=signal_derivative(actual_speed,tv)
        yaw_rate=signal_derivative(np.unwrap(col('yaw')),ta)
        ref_at=signal_derivative(col('ref_speed'),t)
        cmd_at=signal_derivative(col('command_speed'),t)
        data=dict(t=t,attitude_t=ta,velocity_t=tv,reference_speed=col('ref_speed'),
            command_speed=col('command_speed'),actual_groundspeed=actual_speed,
            reference_accel=ref_at,command_accel=cmd_at,actual_accel=at,
            reference_yaw_rate=col('ref_speed')*col('ref_member_curvature'),
            actual_yaw_rate=yaw_rate,reference_curvature=col('kappa'),
            reference_curvature_derivative=col('kappa_prime'),
            roll=col('roll'),pitch=col('pitch'),altitude=col('alt'))
        detail=dict(tangential_accel=distribution(at,tv,member.a_t_min,member.a_t_max),
            reference_accel=distribution(ref_at,t,member.a_t_min,member.a_t_max),
            sent_speed_derivative=distribution(cmd_at,t,member.a_t_min,member.a_t_max))
        detail['platform']=member.platform
        detail['recorded_command_speed_kind']='airspeed_target' if member.platform=='fixedwing' else 'horizontal_velocity_magnitude'
        if member.platform=='multirotor':
            if all(r.get('optimizer_yaw_rate') not in (None,'') for r in rows):
                data['optimizer_held_yaw_rate']=col('optimizer_yaw_rate')
                detail['optimizer_held_yaw_rate']=distribution(data['optimizer_held_yaw_rate'],t,-member.yaw_rate_max,member.yaw_rate_max)
            else:
                detail['optimizer_held_yaw_rate']={'unavailable':'optimizer_yaw_rate not recorded; command is not a substitute'}
            if cfg.get('copter_yaw_rate_feedforward'):
                data['sent_yaw_rate_feedforward']=col('command_yaw_rate')
                detail['sent_yaw_rate_feedforward']=distribution(data['sent_yaw_rate_feedforward'],t,-member.yaw_rate_max,member.yaw_rate_max)
            data['sent_heading_derivative']=command_heading_derivative(rows)
            for name,clock in (('actual_yaw_rate',ta),('reference_yaw_rate',t),('sent_heading_derivative',t)):
                detail[name]=distribution(data[name],clock,-member.yaw_rate_max,member.yaw_rate_max)
            peak=int(np.argmax(abs(yaw_rate)))
        else:
            airspeed_data,detail['measured_airspeed']=measured_airspeed_metrics(rows,member.v_min,member.v_max)
            data.update(airspeed_data)
            raw=root/('vehicle_%d'%member.member_id)/'telemetry.csv'
            with raw.open() as stream:telemetry=list(csv.DictReader(stream))
            result['source_sha256'][str(raw.relative_to(root))]=hashlib.sha256(raw.read_bytes()).hexdigest()
            for key in ('throttle','target_bank','p','q'):
                pairs=[(float(r['sim_s']),float(r[key])) for r in telemetry if r[key]]
                if pairs:
                    pts=np.asarray(pairs)
                    data[key]=np.interp(ta,pts[:,0],pts[:,1])
            peak=int(np.argmax(abs(at)))
        detail['peak_response']={key:float(values[peak]) for key,values in data.items()}
        detail['phase_acceleration']={}
        for label,mask in geometry_phase_masks(col('kappa'),col('kappa_prime')).items():
            if np.any(mask):
                # Report extrema only; disjoint windows cannot share interval weights.
                detail['phase_acceleration'][label]=dict(minimum=float(min(at[mask])),maximum=float(max(at[mask])),
                                                        samples=int(sum(mask)))
        with (out/('member_%d.csv'%member.member_id)).open('x',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(data.keys());writer.writerows(zip(*data.values()))
        result['members'][member.member_id]=detail
    for name in ('config.json','flight.csv','feedback_solves.jsonl','fixedwing_commands.jsonl','flight_status.json',
                 'config_snapshot/capabilities_verified.yaml'):
        source=root/name
        if source.is_file():result['source_sha256'][name]=hashlib.sha256(source.read_bytes()).hexdigest()
    result['audit_source_sha256']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
        (Path(__file__).resolve(),Path(__file__).resolve().parent/'analyze_backend_dynamics.py',
         Path(__file__).resolve().parents[1]/'uav_formation/paper_metrics.py')}
    (out/'diagnosis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['members'],indent=2))


if __name__=='__main__':main()
