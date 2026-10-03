#!/usr/bin/env python3
"""Audit recorded NMPC initial states and controls, independent of flight success.

Supports the direct MAVLink and ROS S1 evidence formats. This audit does not
certify vehicle capability or replace trajectory/actual-velocity metrics.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np


def audit(directory):
    directory = Path(directory)
    direct = (directory/'feedback_solves.jsonl').exists()
    path = directory/('feedback_solves.jsonl' if direct else 'feedback_status.jsonl')
    events = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    solves = [e for e in events if (e.get('event') == 'SOLVE' if direct else e.get('reason') == 'MEASURED')]
    cfg = json.loads((directory/'config.json').read_text())
    result = dict(source=str(path.resolve()), backend='direct MAVLink' if direct else 'ROS PoseStamped',
                  scope='NMPC feedback provenance and input bounds only; not flight acceptance',
                  solves=len(solves), members={}, non_nmpc_members=[],
                  other_events=dict(Counter(e.get('reason', e.get('event', 'UNKNOWN')) for e in events if e not in solves)))
    for member in cfg['members']:
        i = member['member_id']
        if member['platform'] != 'multirotor':
            result['non_nmpc_members'].append(i)
            continue
        rows = [e for e in solves if e['member'] == i]
        if not rows:
            result['members'][i] = dict(solves=0, feedback_check_passed=False)
            continue
        errors = []; bounds = []; ages = []; extrap = []; times = []; runtime = []
        for row in rows:
            measured = row['measured']
            x, y, yaw = row['x0']
            angular = math.atan2(math.sin(yaw-measured['yaw']), math.cos(yaw-measured['yaw']))
            errors.append(max(abs(x-measured['x']), abs(y-measured['y']), abs(angular)))
            v, w = row['control']
            # ROS records formation capture too, where reverse input is permitted.
            lower = 0. if direct else -member['v_max']
            bounds.append(lower-1e-9 <= v <= member['v_max']+1e-9 and abs(w) <= 1.2+1e-9)
            ages.extend((measured['position_age'], measured['yaw_age']))
            extrap.append(measured['extrapolation']); times.append(measured['stamp'])
            runtime.append(row['solve_wall_s'])
        controls = np.array([e['control'] for e in rows])
        intervals = np.diff(times)
        valid = (max(errors) < 1e-9 and all(bounds) and max(ages) <= .5+1e-9
                 and min(ages) >= -.05 and max(extrap) <= .2+1e-9
                 and bool(np.all(intervals > 0)))
        result['members'][i] = dict(solves=len(rows), feedback_check_passed=valid,
            maximum_x0_measurement_difference=float(max(errors)), maximum_source_age_s=float(max(ages)),
            maximum_measured_extrapolation_s=float(max(extrap)),
            control_min=controls.min(axis=0).tolist(), control_max=controls.max(axis=0).tolist(),
            input_bounds_passed=all(bounds), speed_lower_audit=0. if direct else -member['v_max'],
            nominal_period_s=cfg.get('feedback',{}).get('control_period_s',1/3),
            mean_period_s=float(np.mean(intervals)) if len(intervals) else None,
            maximum_period_s=float(max(intervals)) if len(intervals) else None,
            effective_rate_hz=(len(rows)-1)/(times[-1]-times[0]) if len(rows)>1 else None,
            maximum_solve_wall_s=float(max(runtime)),
            solver_status=dict(Counter(e['status' if direct else 'solver'] for e in rows)))
    result['feedback_check_passed'] = bool(result['members']) and all(
        m['feedback_check_passed'] for m in result['members'].values())
    target = directory/'feedback_audit.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    result = audit(args.directory)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['feedback_check_passed'] else 1)
