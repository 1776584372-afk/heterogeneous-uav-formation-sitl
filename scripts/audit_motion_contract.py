#!/usr/bin/env python3
"""Separate measured yaw/course and diagnose sampled PPVA compatibility."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from scripts.audit_outgoing_packets import read_journal
from scripts.extract_onboard_control import file_hash
from uav_formation.paper_metrics import signal_derivative


def unique_samples(times, values):
    times, values = np.asarray(times, float), np.asarray(values, float)
    if (times.ndim != 1 or len(times) != len(values) or len(times) < 3
            or not np.all(np.isfinite(times)) or not np.all(np.isfinite(values))
            or np.any(np.diff(times) < 0)):
        raise ValueError('finite ordered source samples required')
    unique, reverse = np.unique(times[::-1], return_index=True)
    if len(unique) < 3:
        raise ValueError('at least three distinct source timestamps required')
    return unique, values[len(times)-1-reverse]


def motion_series(velocity_time, velocity, attitude_time, yaw, min_speed=.5):
    tv, vv = unique_samples(velocity_time, velocity)
    ta, yy = unique_samples(attitude_time, yaw)
    if vv.shape != (len(tv), 2) or yy.shape != ta.shape:
        raise ValueError('ENU horizontal vectors and scalar yaw required')
    keep = (tv >= ta[0]) & (tv <= ta[-1])
    tv, vv = tv[keep], vv[keep]
    if len(tv) < 3 or np.max(np.diff(ta)) > .25 or np.max(np.diff(tv)) > .25:
        raise ValueError('insufficient overlap or source gaps exceed 0.25 s')
    speed = np.linalg.norm(vv, axis=1)
    if np.any(speed <= min_speed):
        raise ValueError('course undefined below diagnostic minimum speed')
    yaw_aligned = np.interp(tv, ta, np.unwrap(yy))
    course = np.unwrap(np.arctan2(vv[:, 1], vv[:, 0]))
    beta = np.arctan2(np.sin(course-yaw_aligned), np.cos(course-yaw_aligned))
    lateral = -vv[:, 0]*np.sin(yaw_aligned)+vv[:, 1]*np.cos(yaw_aligned)
    return dict(boot_s=tv, speed_m_s=speed, yaw_enu_rad=yaw_aligned, course_enu_rad=course,
        beta_rad=beta, horizontal_heading_left_m_s=lateral,
        course_rate_rad_s=signal_derivative(course, tv),
        yaw_rate_rad_s=np.interp(tv, ta, signal_derivative(np.unwrap(yy), ta)))


def stats(values):
    values = np.asarray(values, float)
    if len(values) == 0:
        return dict(unavailable='no samples')
    if not np.all(np.isfinite(values)):
        raise ValueError('nonfinite diagnostic')
    return dict(minimum=float(values.min()), maximum=float(values.max()),
        maximum_abs=float(abs(values).max()), rms=float(np.sqrt(np.mean(values**2))), samples=len(values))


def ppva_residual(times, positions, velocities, accelerations):
    times, positions, velocities, accelerations = [np.asarray(v, float) for v in
        (times, positions, velocities, accelerations)]
    if (times.ndim != 1 or len(times) < 3
            or any(v.shape != (len(times), 2) for v in (positions, velocities, accelerations))
            or not all(np.all(np.isfinite(v)) for v in (times, positions, velocities, accelerations))
            or np.any(np.diff(times) <= 0)):
        raise ValueError('finite vectors on a strictly increasing command clock required')
    dt = np.diff(times)[:, None]
    p_defect = np.diff(positions, axis=0)-.5*(velocities[:-1]+velocities[1:])*dt
    v_defect = np.diff(velocities, axis=0)-.5*(accelerations[:-1]+accelerations[1:])*dt
    return dict(position_defect_m=np.linalg.norm(p_defect, axis=1),
        position_defect_over_dt_m_s=np.linalg.norm(p_defect/dt, axis=1),
        velocity_defect_m_s=np.linalg.norm(v_defect, axis=1),
        velocity_defect_over_dt_m_s2=np.linalg.norm(v_defect/dt, axis=1))


def flight_audit(root, output):
    with (root/'flight.csv').open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['phase'] == 'track' and int(r['member']) == 0]
    def col(k): return [float(r[k]) for r in rows]
    series = motion_series(col('velocity_t'), np.array([col('vx'), col('vy')]).T,
                           col('attitude_t'), col('yaw'))
    with (output/'motion.csv').open('x', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(series); writer.writerows(zip(*series.values()))
    summary = {k: stats(v) for k, v in series.items() if k != 'boot_s'}
    summary['beta_deg'] = stats(np.degrees(series['beta_rad']))
    packets, journal = read_journal(root/'vehicle_0/outgoing_packets.jsonl')
    selected = []
    for packet in packets:
        if packet.get('kind') != 'local_target' or packet['phase'] != 'track' or not packet['complete_local_write']:
            continue
        f = packet['fields']
        if f['coordinate_frame'] != 1 or f['type_mask'] & (1+2+8+16+64+128):
            raise ValueError('PPVA field missing from a task packet')
        clocks = packet['context']['clocks']
        stamp = max(clocks['ATTITUDE']['boot_s'], clocks['LOCAL_POSITION_NED']['boot_s'])
        selected.append([stamp, f['y'], f['x'], f['vy'], f['vx'], f['afy'], f['afx']])
    selected = np.asarray(selected)
    times, data = unique_samples(selected[:, 0], selected[:, 1:])
    residual = ppva_residual(times, data[:, :2], data[:, 2:4], data[:, 4:6])
    events = [json.loads(line) for line in (root/'feedback_solves.jsonl').read_text().splitlines()]
    solve_times = np.array([e['measured']['stamp'] for e in events
        if e.get('event') == 'SOLVE' and e['member'] == 0 and e['phase'] == 'track'])
    if len(solve_times):
        indices = np.searchsorted(solve_times, times, side='right')-1
        resets = indices[1:] != indices[:-1]
        subsets = [('all', np.ones(len(resets), bool)), ('solve_refresh', resets), ('held', ~resets)]
        label = 'solve_refresh'
    else:
        resets = np.zeros(len(times)-1, bool)
        subsets = [('all', np.ones(len(resets), bool)), ('no_nmpc', np.ones(len(resets), bool))]
        label = 'solve_refresh_unavailable_no_nmpc'
    with (output/'ppva.csv').open('x', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(['boot_context_s', label, *residual])
        writer.writerows(zip(times[1:], resets.astype(int), *residual.values()))
    return dict(motion=summary, ppva={name: {k: stats(v[mask]) for k, v in residual.items()}
        for name, mask in subsets},
        packet_issues=journal['issues'],
        limitation='PPVA trapezoidal consistency defects use latest observed boot clock, not exact controller arrival time; finite sampling and message jitter also contribute. Solve labels are preceding measurement anchors. This is not a continuous derivative bound or causal proof.',
        motion_alignment='ENU course from velocity source clock; unwrapped yaw interpolated within attitude clock coverage, max gap 0.25 s; beta is horizontal drift angle, not aerodynamic sideslip',
        source_sha256={str(p): file_hash(p) for p in [root/'flight.csv', root/'config.json',
            root/'feedback_solves.jsonl', root/'vehicle_0/outgoing_packets.jsonl']})


def calibration_audit(root):
    result = json.loads((root/'result.json').read_text())
    with (root/'telemetry.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    selected = []
    for step in result['steps']:
        if step['speed'] != 12 or step['bank'] != 20:
            continue
        lo, hi = step['window']
        rr = [r for r in rows if lo <= float(r['sim_s']) <= hi]
        times = np.array([float(r['sim_s']) for r in rr])
        series = motion_series(times, [[float(r['ve']), float(r['vn'])] for r in rr],
            times, [np.pi/2-float(r['yaw']) for r in rr])
        selected.append(dict(original_step=step, diagnostic={k: stats(v) for k, v in series.items()
            if k in ('course_rate_rad_s', 'yaw_rate_rad_s', 'beta_rad')},
            clock_limitation='legacy telemetry lacks velocity source timestamps; derivatives use logged sim_s and are diagnostic only'))
    sources = [root/'result.json', root/'telemetry.csv', root/'source_snapshot/calibrate_turns.py']
    return dict(windows=selected, source_sha256={str(p): file_hash(p) for p in sources},
        provenance='Frozen source computes measured_mean_heading_rate from atan2(ve,vn), i.e. ground course; PASS checks speed, height, course error and position, not instantaneous Euler yaw rate.',
        interpretation='0.25 rad/s remains a chosen operation bound; these passed windows do not certify a yaw-rate physical limit or a whole-turn instantaneous guarantee.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    result = dict(formal_qualification=False, acceptance_unchanged=True, runs={})
    for run in args.run:
        run = run.resolve()
        destination = out/run.name; destination.mkdir()
        result['runs'][str(run)] = flight_audit(run, destination)
    result['historical_yaw_basis'] = calibration_audit(args.calibration.resolve())
    result['analysis_sha256'] = {str(Path(__file__).resolve()): file_hash(Path(__file__))}
    with (out/'audit.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(out/'audit.json')


if __name__ == '__main__':
    main()
