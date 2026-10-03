"""Recompute fleet metrics from recorded reference and measured telemetry only."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from .experiment_config import load_config
from .experiment_artifacts import write_json


def signal_derivative(values, timestamps):
    """Differentiate unique source samples, then map back without smoothing.

    Re-reading a held MAVLink value must not create a fictitious zero derivative
    followed by a spike. Duplicate timestamps retain the last received value.
    """
    if np.any(np.diff(timestamps)<0):
        raise ValueError('source timestamps went backwards')
    unique,index=np.unique(timestamps[::-1],return_index=True)
    last=len(timestamps)-1-index
    if len(unique)<3:
        raise ValueError('fewer than three distinct source samples')
    derivative=np.gradient(values[last],unique)
    return derivative[np.searchsorted(unique,timestamps)]


def compute(directory):
    directory=Path(directory)
    cfg,members=load_config(directory/'config.json')
    status=json.loads((directory/'flight_status.json').read_text())
    capability_path=directory/'config_snapshot/capabilities_verified.yaml'
    if not capability_path.exists() and cfg.get('capability_source'):
        capability_path=Path(cfg['capability_source'])
    capabilities=yaml.safe_load(capability_path.read_text()) if capability_path.is_file() else {}
    with (directory/'flight.csv').open() as stream:
        rows=[r for r in csv.DictReader(stream) if r['phase']=='track']
    result=dict(completed=status.get('completed',False),failure=status.get('error'),
        seed=cfg.get('seed'),method=cfg.get('method'),scene=cfg['name'],members={},
        definition=dict(window='all track-phase samples, including failed partial runs; join/exit excluded and retained raw',
            weighting='left-endpoint durations, terminal sample zero weight',
            derivatives='numpy.gradient on unique source timestamps; last value at duplicate timestamp; no smoothing; legacy files use common time',
            tangential_acceleration='derivative of horizontal ground-speed magnitude for both platforms, matching path kinematics; measured airspeed is used for wing speed bounds only',
            airspeed_clock='VFR_HUD has no onboard time field; associated latest ATTITUDE boot timestamp at reception (20 Hz quantization)',
            constraint_tolerance=1e-3,roll_frame='NED, positive right; compare absolute bank',
            shape='RMS of all pairwise distance residuals, then time weighted',
            missing='null; never substitute a success or zero error'))
    if not rows:
        result.update(valid_telemetry=False,tracking_duration=0.,minimum_separation=None,shape_rmse=None)
        write_json(directory/'metrics.json',result)
        return result
    times=np.array(sorted({float(r['t']) for r in rows}))
    if len(times)<3:
        raise ValueError('fewer than three tracking timestamps')
    w=np.diff(times,append=times[-1]); duration=times[-1]-times[0]
    if duration<=0:
        raise ValueError('nonpositive duration')
    groups={t:{} for t in times}
    for r in rows:
        groups[float(r['t'])][int(r['member'])]=r
    if any(len(group)!=len(members) for group in groups.values()):
        raise ValueError('incomplete synchronized member rows')
    shape=[]; separations=[]
    for group in groups.values():
        squared=[]
        for i in range(len(members)):
            for j in range(i+1,len(members)):
                a,b=group[i],group[j]
                actual=np.hypot(float(a['x'])-float(b['x']),float(a['y'])-float(b['y']))
                desired=abs(members[i].r_y-members[j].r_y)
                squared.append((actual-desired)**2); separations.append(float(actual))
        shape.append(float(np.mean(squared)) if squared else 0.)
    for member in members:
        rr=[groups[t][member.member_id] for t in times]
        def col(name):
            value=np.array([float(r[name]) for r in rr])
            if not np.all(np.isfinite(value)):
                raise ValueError('nonfinite '+name)
            return value
        speed=col('airspeed') if member.platform=='fixedwing' else np.hypot(col('vx'),col('vy'))
        velocity_time=col('velocity_t') if 'velocity_t' in rr[0] else times
        attitude_time=col('attitude_t') if 'attitude_t' in rr[0] else times
        vx,vy=col('vx'),col('vy')
        at=signal_derivative(np.hypot(vx,vy),velocity_time)
        an=np.abs(vx*signal_derivative(vy,velocity_time)-vy*signal_derivative(vx,velocity_time))/np.maximum(np.hypot(vx,vy),.1)
        heading_rate=signal_derivative(np.unwrap(col('yaw')),attitude_time)
        margins=dict(speed_lower=speed-member.v_min,speed_upper=member.v_max-speed,
                     lateral_accel=member.a_lat_max-an,tangential_lower=at-member.a_t_min,
                     tangential_upper=member.a_t_max-at)
        if member.bank_max is not None:
            margins['bank']=member.bank_max-np.abs(col('roll'))
        if member.yaw_rate_max is not None:
            margins['yaw_rate']=member.yaw_rate_max-np.abs(heading_rate)
        direction_ranges={}
        directional_limits=(capabilities.get('multirotor',{}).get('limits',{})
                            if capabilities else {})
        if member.platform=='multirotor' and directional_limits:
            heading=col('yaw')
            forward=vx*np.cos(heading)+vy*np.sin(heading)
            left=-vx*np.sin(heading)+vy*np.cos(heading)
            for name,value in (('forward',forward),('backward',-forward),('left',left),('right',-left)):
                bound=directional_limits[name+'_speed']['value']
                margins['horizontal_heading_'+name]=bound-np.maximum(value,0.)
                direction_ranges[name]=float(max(np.maximum(value,0.)))
        error=np.hypot(col('x')-col('ref_x'),col('y')-col('ref_y'))
        result['members'][member.member_id]=dict(
            tracking_rmse=float(np.sqrt(np.dot(error**2,w)/duration)),tracking_max=float(max(error)),
            speed_min=float(min(speed)),speed_max=float(max(speed)),bank_abs_max_deg=float(np.degrees(max(abs(col('roll'))))),
            horizontal_heading_direction_max_m_s=direction_ranges,
            altitude_error_max=float(max(abs(col('alt')-cfg['altitude']))) if 'alt' in rr[0] else None,
            minimum_margins={name:float(min(value)) for name,value in margins.items()},
            violation_time_fraction={name:float(np.dot(value < -1e-3,w)/duration) for name,value in margins.items()},
            controller_seconds_mean=float(np.mean(col('controller_seconds'))),
            controller_seconds_max=float(max(col('controller_seconds'))),
            mpc_failures=int(col('mpc_failures')[-1]),max_clock_drift=float(max(abs(col('clock_drift')))))
    result.update(valid_telemetry=bool(max(np.diff(times))<=1.),tracking_duration=float(duration),
        maximum_sample_gap=float(max(np.diff(times))),minimum_separation=min(separations) if separations else None,
        shape_rmse=float(np.sqrt(np.dot(shape,w)/duration)) if separations else None,
        selected_radius=float(rows[0]['radius']),
        minimum_interval_width=min(float(r['upper'])-float(r['lower']) for r in rows),
        planning_seconds=status.get('planning_seconds'))
    write_json(directory/'metrics.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    args=parser.parse_args()
    print(json.dumps(compute(args.directory),indent=2))
