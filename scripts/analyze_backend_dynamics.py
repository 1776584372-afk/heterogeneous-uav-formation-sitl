#!/usr/bin/env python3
"""Read-only execution diagnostics; no smoothing or capability extrapolation."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from uav_formation.paper_metrics import signal_derivative


def distribution(values, times, lower=None, upper=None):
    values, times = np.asarray(values, dtype=float), np.asarray(times, dtype=float)
    if (values.ndim != 1 or times.shape != values.shape or len(values) < 3
            or not np.all(np.isfinite(values)) or not np.all(np.isfinite(times))
            or np.any(np.diff(times) < 0)):
        raise ValueError('finite ordered samples required')
    if (any(bound is not None and not np.isfinite(bound) for bound in (lower, upper))
            or (lower is not None and upper is not None and lower > upper)):
        raise ValueError('finite ordered bounds required')
    duration = times[-1] - times[0]
    if duration <= 0:
        raise ValueError('positive sample duration required')
    weights = np.diff(times, append=times[-1])
    result = dict(minimum=float(min(values)), maximum=float(max(values)),
                  mean=float(np.dot(values, weights)/duration), samples=len(values))
    for name, bound, violation in [('lower', lower, values < (lower or 0)-.001),
                                   ('upper', upper, values > (upper or 0)+.001)]:
        if bound is not None:
            result[name+'_bound'] = bound
            result[name+'_violation_fraction'] = float(np.dot(violation, weights)/duration)
            result[name+'_violation_samples'] = int(np.count_nonzero(violation))
    return result


def telemetry_metrics(rows, platform):
    def col(key):
        return np.array([float(r[key]) for r in rows])
    t, tv = col('sim_s'), col('velocity_sim_s')
    speed = np.hypot(col('ve'), col('vn'))
    dv_e, dv_n = signal_derivative(col('ve'), tv), signal_derivative(col('vn'), tv)
    lateral = abs(col('ve')*dv_n-col('vn')*dv_e)/np.maximum(speed, .1)
    at = signal_derivative(speed, tv)
    yaw = signal_derivative(np.unwrap(col('yaw')), t)
    bank_rate = signal_derivative(np.unwrap(col('roll')), t)
    wing = platform == 'plane'
    result = dict(groundspeed=distribution(speed,tv,None if wing else 0.,None if wing else 20.),
        tangential_accel=distribution(at,tv,-.2 if wing else -.4,.2 if wing else .4),
        lateral_accel=distribution(lateral,tv,0.,2.5 if wing else 3.),
        yaw_rate=distribution(yaw,t,None if wing else -.25,None if wing else .25),
        bank_deg=distribution(np.degrees(col('roll')),t,-27. if wing else None,27. if wing else None),
        roll_euler_rate_rad_s=distribution(bank_rate,t),
        body_roll_rate_rad_s=distribution(col('p'),t),
        pitch_deg=distribution(np.degrees(col('pitch')),t),
        throttle=distribution(col('throttle'),t),
        altitude=distribution(col('alt'),t),
        heading_forward=distribution(col('heading_forward'),tv),
        heading_left=distribution(col('heading_left'),tv),
        body_forward=distribution(col('forward'),tv),body_left=distribution(col('left'),tv))
    if wing:
        ta=col('airspeed_sim_s')
        result['airspeed']=distribution(col('airspeed'),ta,12.,24.)
        result['airspeed_derivative']=distribution(signal_derivative(col('airspeed'),ta),ta)
    result['bank_rate_capability_bound']=None
    result['bank_rate_evidence']='observed response only; no certified maximum'
    return result


def analyze(root):
    root=Path(root)
    manifest=json.loads((root/'manifest.json').read_text())
    with (root/'telemetry.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    phases=sorted({r['phase'] for r in rows if r['phase'].startswith(('step_','turn_'))})
    result=dict(schema_version=2,platform=manifest['vehicle'],model=manifest['model'],
        source=str(root.resolve()),envelope_certified=False,
        frozen_limits_changed=False, formal_qualification=False,
        procedure='unique source timestamp gradient, no smoothing; complete step/turn including transients',
        scope='step/turn phases only; excludes takeoff, warm-up, path entry and exit',
        frame='horizontal heading FLU and 3D body FLU reported separately',phases={})
    for phase in phases:
        phase_rows=[r for r in rows if r['phase']==phase]
        try:
            result['phases'][phase]=telemetry_metrics(phase_rows,manifest['vehicle'])
        except ValueError as exc:
            result['phases'][phase]={'unavailable':str(exc)}
    result['steady_windows']=[]
    report_path=root/'result.json'
    if report_path.exists():
        report=json.loads(report_path.read_text())
        for step in report.get('steps',[]):
            lo,hi=step['window']
            window=[r for r in rows if lo<=float(r['sim_s'])<=hi]
            entry=dict(speed=step['speed'],tracking_status=step['status'],window=[lo,hi],
                       bank=step.get('bank'),sign=step.get('sign'))
            try:
                entry['metrics']=telemetry_metrics(window,manifest['vehicle'])
            except ValueError as exc:
                entry['unavailable']=str(exc)
            result['steady_windows'].append(entry)
    source=root/'telemetry.csv'
    result['telemetry_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run');p.add_argument('--output',required=True)
    args=p.parse_args()
    result=analyze(args.run)
    with Path(args.output).open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(args.output)


if __name__=='__main__':
    main()
