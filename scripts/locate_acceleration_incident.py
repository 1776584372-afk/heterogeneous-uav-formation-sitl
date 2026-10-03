#!/usr/bin/env python3
"""Locate a truth acceleration minimum against source-clock flight telemetry."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from uav_formation.paper_metrics import signal_derivative


def locate(run, truth, member):
    with truth.open() as stream:
        peak = min(csv.DictReader(stream), key=lambda r: float(r['tangential_accel']))
    stamp = float(peak['boot_time'])
    with (run/'flight.csv').open() as stream:
        rows = [r for r in csv.DictReader(stream)
                if r['phase'] == 'track' and int(r['member']) == member]
    clocks = np.array([float(r['velocity_t']) for r in rows])
    if len(rows) < 3 or not clocks[0] <= stamp <= clocks[-1]:
        raise ValueError('truth peak outside track telemetry coverage')
    index = int(np.argmin(abs(clocks-stamp)))
    row = rows[index]
    delta = float(clocks[index]-stamp)
    if abs(delta) > .25:
        raise ValueError('no sufficiently close velocity source sample')
    command_accel = signal_derivative(
        np.array([float(r['command_speed']) for r in rows]),
        np.array([float(r['t']) for r in rows]))
    k, kp = float(row['kappa']), float(row['kappa_prime'])
    phase = 'transition' if abs(kp)>1e-8 else 'straight' if abs(k)<1e-8 else 'constant_turn'
    cfg = json.loads((run/'config.json').read_text())
    member_cfg = next(m for m in cfg['members'] if m['member_id'] == member)
    ry = member_cfg['r_y']
    reference_accel = (1-ry*k)*float(row['nu_dot'])-ry*kp*float(row['nu'])**2
    return dict(run=str(run.resolve()), member=member, truth_peak=peak,
        geometry_phase=phase, nearest_flight_row=row, velocity_clock_delta_s=delta,
        reference_accel_m_s2=reference_accel,
        recorded_command_speed_derivative_m_s2=float(command_accel[index]),
        limitation='Nearest source-clock association, not simultaneous controller internals or proof of cause. Command derivative uses logged flight-loop time. Original pass/fail and raw samples are unchanged.',
        source_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (truth, run/'flight.csv', run/'config.json', Path(__file__),
                      Path(__file__).resolve().parents[1]/'uav_formation/paper_metrics.py')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--truth', type=Path, required=True)
    parser.add_argument('--member', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = locate(args.run, args.truth, args.member)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(args.output)


if __name__ == '__main__':
    main()
