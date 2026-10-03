#!/usr/bin/env python3
"""Analyse native DataFlash signals with the preregistered v2 processing methods."""
from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pymavlink import mavutil

from uav_formation.measurement_v2 import ProcessingSpec, derivative_sensitivity, describe


DERIVATIVE_SIGNALS = {
    'sim2_ground_speed_m_s': ('SIM2', lambda row: math.hypot(row['VE'], row['VN'])),
    'sim2_airspeed_m_s': ('SIM2', lambda row: row['As']),
    'arsp_equivalent_airspeed_m_s': ('ARSP', lambda row: row['Airspeed']),
    'sim_euler_yaw_rad': ('SIM', lambda row: math.radians(row['Yaw'])),
    'att_euler_yaw_rad': ('ATT', lambda row: math.radians(row['Yaw'])),
    'sim_euler_roll_rad': ('SIM', lambda row: math.radians(row['Roll'])),
    'att_euler_roll_rad': ('ATT', lambda row: math.radians(row['Roll'])),
}

DIRECT_SIGNALS = {
    'imu_body_roll_rate_rad_s': ('IMU', 'GyrX'),
    'imu_body_pitch_rate_rad_s': ('IMU', 'GyrY'),
    'imu_body_yaw_rate_rad_s': ('IMU', 'GyrZ'),
    'imu_body_accel_x_m_s2': ('IMU', 'AccX'),
    'imu_body_accel_y_m_s2': ('IMU', 'AccY'),
    'imu_body_accel_z_m_s2': ('IMU', 'AccZ'),
}

CONTROL_FIELDS = {
    'copter_psce_desired_accel_east_m_s2': ('PSCE', 'DAE'),
    'copter_psce_target_accel_east_m_s2': ('PSCE', 'TAE'),
    'copter_psce_lean_equivalent_accel_east_m_s2': ('PSCE', 'AE'),
    'copter_pscn_desired_accel_north_m_s2': ('PSCN', 'DAN'),
    'copter_pscn_target_accel_north_m_s2': ('PSCN', 'TAN'),
    'copter_pscn_lean_equivalent_accel_north_m_s2': ('PSCN', 'AN'),
    'fixedwing_tecs_target_airspeed_m_s': ('TECS', 'spdem'),
    'fixedwing_tecs_airspeed_m_s': ('TECS', 'sp'),
    'fixedwing_tecs_throttle_fraction': ('TECS', 'th'),
    'fixedwing_tecs_pitch_rad': ('TECS', 'ph'),
    'fixedwing_ofg_airspeed_m_s': ('OFG', 'Arsp'),
    'fixedwing_ctun_throttle_percent': ('CTUN', 'ThO'),
}


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def unwrap_if_angle(name, values):
    values = np.asarray(values, float)
    return np.unwrap(values) if 'euler_' in name else values


def load_bin(path):
    wanted = sorted({item[0] for item in DERIVATIVE_SIGNALS.values()}
                    | {item[0] for item in DIRECT_SIGNALS.values()}
                    | {item[0] for item in CONTROL_FIELDS.values()}
                    | {'RCOU', 'CTUN', 'TECS', 'OFG', 'RATE', 'MOTB'})
    rows = {name: [] for name in wanted}
    reader = mavutil.mavlink_connection(str(path))
    try:
        while True:
            message = reader.recv_match(type=wanted)
            if message is None:
                break
            name = message.get_type()
            row = message.to_dict()
            if 'TimeUS' not in row:
                continue
            if name in ('IMU', 'ARSP') and int(row.get('I', 0)) != 0:
                continue
            row['boot_s'] = float(row['TimeUS']) / 1e6
            rows[name].append(row)
    finally:
        reader.close()
    return rows


def load_phases(path):
    if path is None:
        return None
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    pairs = []
    for row in rows:
        if row.get('sim_s') not in (None, '') and row.get('phase'):
            pairs.append((float(row['sim_s']), row['phase']))
    pairs.sort()
    if not pairs:
        raise ValueError('telemetry has no phase clock')
    times = [pair[0] for pair in pairs]
    phases = [pair[1] for pair in pairs]
    return times, phases


def load_phase_events(path):
    if path is None:
        return None
    pairs = []
    with Path(path).open() as stream:
        for line in stream:
            event = json.loads(line)
            if (event.get('event') == 'phase_transition'
                    and event.get('boot_s') is not None and event.get('phase')):
                pairs.append((float(event['boot_s']), str(event['phase'])))
    pairs.sort()
    if not pairs:
        raise ValueError('event journal has no phase transitions')
    return [item[0] for item in pairs], [item[1] for item in pairs]


def merged_phase_map(telemetry, events):
    event_map = load_phase_events(events)
    if telemetry is None:
        return event_map
    telemetry_map = load_phases(telemetry)
    event_labels = set(event_map[1])
    additions = []
    added_labels = set()
    previous = None
    for stamp, label in zip(*telemetry_map):
        if (label != previous and label not in event_labels
                and label not in added_labels):
            additions.append((stamp, label))
            added_labels.add(label)
        previous = label
    # Native phase-transition events override telemetry rows sharing the same
    # source timestamp. Telemetry only supplies labels absent from the journal.
    combined = sorted([*additions, *zip(*event_map)], key=lambda item: item[0])
    return [item[0] for item in combined], [item[1] for item in combined]


def phases_at(times, phase_map):
    if phase_map is None:
        return np.asarray(['all'] * len(times), object)
    clock, labels = phase_map
    output = []
    for value in times:
        index = bisect_right(clock, value) - 1
        output.append(labels[index] if index >= 0 else 'unassigned')
    return np.asarray(output, object)


def event_summary(times, values, valid, limit, merge_gap_s=.10):
    times = np.asarray(times, float)
    values = np.asarray(values, float)
    valid = np.asarray(valid, bool) & np.isfinite(values)
    exceed = valid & (np.abs(values) > limit)
    events = []
    active = None
    previous = None
    for time, flag in zip(times, exceed):
        if not flag:
            continue
        if active is None or previous is None or time - previous > merge_gap_s:
            if active is not None:
                events.append(active)
            active = [float(time), float(time)]
        else:
            active[1] = float(time)
        previous = time
    if active is not None:
        events.append(active)
    durations = [end - start for start, end in events]
    if np.count_nonzero(valid) > 1:
        valid_time = times[valid]
        span = valid_time[-1] - valid_time[0]
        weights = np.diff(times, append=times[-1])
        duration = float(np.sum(weights[exceed]))
    else:
        span, duration = 0.0, 0.0
    return {
        'limit_abs': float(limit), 'event_count': len(events),
        'total_duration_s': duration,
        'time_fraction': duration / span if span > 0 else None,
        'longest_contiguous_event_s': max(durations, default=0.0),
        'events': [{'start_s': start, 'end_s': end, 'duration_s': end - start}
                   for start, end in events],
    }


def phase_boundary_validity(times, phase, exclusion_s=.25):
    times = np.asarray(times, float)
    phase = np.asarray(phase, object)
    valid = np.ones(len(times), bool)
    changes = np.flatnonzero(phase[1:] != phase[:-1]) + 1
    for index in changes:
        boundary = .5 * (times[index - 1] + times[index])
        valid[np.abs(times - boundary) <= exclusion_s + 1e-12] = False
    return valid


def method_summary(result, phase_map, diagnostic_limit=None):
    phase = phases_at(result['time'], phase_map)
    valid = (np.asarray(result['valid'], bool) & np.isfinite(result['value'])
             & phase_boundary_validity(result['time'], phase))
    summary = {'all': describe(np.asarray(result['value'])[valid])}
    summary['phases'] = {}
    for label in sorted(set(phase[valid])):
        selected = valid & (phase == label)
        summary['phases'][label] = describe(np.asarray(result['value'])[selected])
    if diagnostic_limit is not None:
        summary['legacy_diagnostic'] = event_summary(
            result['time'], result['value'], valid, diagnostic_limit)
    return summary


def _phase_values(rows, field, phase_map):
    values, phases = [], []
    for row in rows:
        try:
            value = float(row[field])
            stamp = float(row['boot_s'])
        except (KeyError, TypeError, ValueError):
            continue
        values.append(value)
        phases.append(str(phases_at([stamp], phase_map)[0]))
    return np.asarray(values, float), np.asarray(phases, object)


def _pwm_phase_summary(records, channels, high, low=None):
    by_phase = {}
    for row in records:
        try:
            phase = row['_phase']
            values = [float(row[channel]) for channel in channels]
        except (KeyError, TypeError, ValueError):
            continue
        active = [value for value in values if value > 0]
        if phase in ('unassigned', 'all') or not active:
            continue
        by_phase.setdefault(phase, []).extend(active)
    output = {}
    for phase, values in sorted(by_phase.items()):
        array = np.asarray(values, float)
        item = {'samples': len(values), 'minimum': float(np.min(array)),
                'maximum': float(np.max(array)),
                'high_fraction': float(np.mean(array >= high))}
        if low is not None:
            item['low_fraction'] = float(np.mean(array <= low))
        output[phase] = item
    return output


def control_summary(rows, phase_map, platform):
    """Report controller and actuator headroom without identifying capability."""
    output = {'signals': {}, 'actuator': {}}
    for name, (kind, field) in CONTROL_FIELDS.items():
        values, phases = _phase_values(rows.get(kind, []), field, phase_map)
        if len(values) == 0:
            continue
        output['signals'][name] = {
            'all': describe(values),
            'phases': {label: describe(values[phases == label])
                       for label in sorted(set(phases))},
        }
    rcou = []
    for source in rows.get('RCOU', []):
        row = dict(source)
        row['_phase'] = str(phases_at([float(row['boot_s'])], phase_map)[0])
        rcou.append(row)
    if platform == 'copter':
        output['actuator']['motor_pwm'] = {
            'thresholds': {'high_pwm': 1950},
            'phases': _pwm_phase_summary(rcou, ('C1', 'C2', 'C3', 'C4'), 1950),
        }
    else:
        output['actuator']['throttle_pwm'] = {
            'thresholds': {'high_pwm': 1950, 'low_pwm': 1050},
            'phases': _pwm_phase_summary(rcou, ('C3',), 1950, 1050),
        }
        output['actuator']['surface_pwm'] = {
            'thresholds': {'endpoint_high_pwm': 1950, 'endpoint_low_pwm': 1050},
            'phases': _pwm_phase_summary(rcou, ('C1', 'C2', 'C4'), 1950, 1050),
        }
    output['scope'] = ('diagnostic thresholds from acceptance_protocol_v2; phase labels use '
                       'the preceding telemetry clock; no actuator statistic alone certifies capability')
    output['source_semantics'] = {
        'PSCE.AE_PSCN.AN': ('lean-angle-equivalent horizontal acceleration from '
                            'AC_PosControl::lean_angles_to_accel_NE_mss; not a kinematic '
                            'derivative of ground velocity'),
        'PSCE.TAE_PSCN.TAN': 'position-controller target acceleration',
        'PSCE.DAE_PSCN.DAN': 'external/desired acceleration entering position control',
        'TECS.spdem_sp': 'TECS true-airspeed demand and state; separate from SIM2 equivalent airspeed',
        'OFG.Arsp': 'external equivalent-airspeed command accepted by the Plane guided interface',
    }
    return output


def analyse(path, telemetry, output, diagnostic_ground_limit=None, events=None):
    path = Path(path).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    phase_map = merged_phase_map(telemetry, events) if events is not None else load_phases(telemetry)
    rows = load_bin(path)
    report = {
        'schema_version': 1, 'source': str(path),
        'scope': 'SITL engineering evidence; not a real-aircraft physical envelope',
        'processing': ProcessingSpec().__dict__, 'derivatives': {}, 'direct_signals': {},
        'record_counts': {name: len(value) for name, value in rows.items()},
        'diagnostic_legacy_ground_acceleration_limit_m_s2': diagnostic_ground_limit,
        'phase_clock': ('phase_transition events with native boot_s; telemetry only supplements '
                        'lifecycle labels absent from the event journal'
                        if events is not None else 'preceding telemetry phase on ATTITUDE boot clock'),
    }
    for name, (kind, getter) in DERIVATIVE_SIGNALS.items():
        source = rows[kind]
        if len(source) < 3:
            report['derivatives'][name] = {'unavailable': f'fewer than three {kind} samples'}
            continue
        times = np.asarray([row['boot_s'] for row in source])
        values = unwrap_if_angle(name, [getter(row) for row in source])
        try:
            methods = derivative_sensitivity(times, values)
        except ValueError as exc:
            report['derivatives'][name] = {'unavailable': str(exc)}
            continue
        report['derivatives'][name] = {}
        for method, result in methods.items():
            limit = (diagnostic_ground_limit if name == 'sim2_ground_speed_m_s' else None)
            report['derivatives'][name][method] = method_summary(result, phase_map, limit)
        primary = methods['resampled_savitzky_golay']
        with (output / f'{name}_primary.csv').open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['boot_s', 'derivative', 'valid', 'phase'])
            writer.writerows(zip(primary['time'], primary['value'], primary['valid'].astype(int),
                                 phases_at(primary['time'], phase_map)))
    for name, (kind, field) in DIRECT_SIGNALS.items():
        source = rows[kind]
        if not source:
            report['direct_signals'][name] = {'unavailable': f'no {kind} records'}
            continue
        times = np.asarray([row['boot_s'] for row in source])
        values = np.asarray([row[field] for row in source], float)
        phase = phases_at(times, phase_map)
        report['direct_signals'][name] = {'all': describe(values), 'phases': {
            label: describe(values[phase == label]) for label in sorted(set(phase))}}
    platform = 'copter' if rows.get('PSCE') else 'plane'
    report['controller_diagnostics'] = control_summary(rows, phase_map, platform)
    report['source_sha256'] = {str(path): file_hash(path)}
    if telemetry is not None:
        report['source_sha256'][str(Path(telemetry).resolve())] = file_hash(telemetry)
    if events is not None:
        report['source_sha256'][str(Path(events).resolve())] = file_hash(events)
    report['analysis_sha256'] = file_hash(Path(__file__))
    (output / 'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bin_file', type=Path)
    parser.add_argument('--telemetry', type=Path)
    parser.add_argument('--events', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--diagnostic-legacy-ground-limit', type=float)
    args = parser.parse_args()
    if args.diagnostic_legacy_ground_limit is not None and args.diagnostic_legacy_ground_limit <= 0:
        parser.error('diagnostic limit must be positive')
    report = analyse(args.bin_file, args.telemetry, args.output,
                     args.diagnostic_legacy_ground_limit, args.events)
    print(json.dumps({'record_counts': report['record_counts'],
                      'output': str(args.output)}, indent=2))


if __name__ == '__main__':
    main()
