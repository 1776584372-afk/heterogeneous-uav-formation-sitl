#!/usr/bin/env python3
"""Independently audit one v3 system or formal fleet run."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from scripts.analyze_v2_bin import analyse
from scripts.audit_v3_command_timing import (read_packets, raw_source_summary,
                                              timing_summary)
from uav_formation.experiment_artifacts import write_json
from uav_formation.experiment_config import load_config


ROOT = Path(__file__).resolve().parents[1]


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def analysis_output(vehicle):
    """Select an immutable output directory for this analysis implementation."""
    analysis_hash = file_hash(ROOT / 'scripts' / 'analyze_v2_bin.py')
    base = vehicle / ('v3_formal_analysis_' + analysis_hash[:12])
    candidate = base
    revision = 1
    while candidate.exists():
        report_path = candidate / 'analysis.json'
        if report_path.is_file():
            report = json.loads(report_path.read_text())
            if report.get('analysis_sha256') == analysis_hash:
                return candidate, report
        revision += 1
        candidate = vehicle / (base.name + '_revision_%d' % revision)
    return candidate, None


def wrap(values):
    return np.arctan2(np.sin(values), np.cos(values))


def load_reference(directory, member_id):
    with (directory / 'member_reference.csv').open() as stream:
        rows = [row for row in csv.DictReader(stream)
                if int(row['member']) == member_id]
    if len(rows) < 2:
        raise ValueError('member reference is incomplete')
    return {
        key: np.asarray([float(row[key]) for row in rows])
        for key in ('t', 'x', 'y', 'course', 'speed')
    }


def aligned_tracking(directory, cfg, members, thresholds):
    with (directory / 'flight.csv').open() as stream:
        rows = [row for row in csv.DictReader(stream) if row['phase'] == 'track']
    if not rows:
        raise ValueError('no track-phase flight rows')
    origin = json.loads((directory / 'world_origin.json').read_text())['origin']
    output = {}
    for member in members:
        selected = [row for row in rows if int(row['member']) == member.member_id]
        if len(selected) < 3:
            raise ValueError('insufficient member tracking rows')
        reference = load_reference(directory, member.member_id)
        time = np.asarray([float(row['t']) + float(row['clock_drift']) for row in selected])
        ref_time = reference['t']
        rx = np.interp(time, ref_time, reference['x']) + origin[0]
        ry = np.interp(time, ref_time, reference['y']) + origin[1]
        course = np.interp(time, ref_time, np.unwrap(reference['course']))
        speed_ref = np.interp(time, ref_time, reference['speed'])
        x = np.asarray([float(row['x']) for row in selected])
        y = np.asarray([float(row['y']) for row in selected])
        vx = np.asarray([float(row['vx']) for row in selected])
        vy = np.asarray([float(row['vy']) for row in selected])
        actual_speed = (np.asarray([float(row['airspeed']) for row in selected])
                        if member.platform == 'fixedwing' else np.hypot(vx, vy))
        actual_course = np.arctan2(vy, vx)
        position_error = np.hypot(x - rx, y - ry)
        speed_error = np.abs(actual_speed - speed_ref)
        course_error = np.abs(np.degrees(wrap(actual_course - course)))
        altitude_error = np.abs(np.asarray([float(row['alt']) for row in selected]) - cfg['altitude'])
        roll = np.abs(np.degrees(np.asarray([float(row['roll']) for row in selected])))
        failures = []
        checks = {
            'position_error_m': (float(np.max(position_error)), thresholds['maximum_position_error_m']),
            'speed_error_m_s': (float(np.max(speed_error)), thresholds['maximum_speed_error_m_s']),
            'course_error_deg': (float(np.max(course_error)), thresholds['maximum_course_error_deg']),
            'altitude_error_m': (float(np.max(altitude_error)), thresholds['maximum_altitude_error_m']),
        }
        if member.platform == 'fixedwing':
            checks['roll_abs_deg'] = (float(np.max(roll)), thresholds['maximum_fixedwing_roll_deg'])
        for name, (value, limit) in checks.items():
            if value > limit + 1e-12:
                failures.append(name)
        output[str(member.member_id)] = {
            'platform': member.platform,
            'samples': len(selected),
            'reference_alignment': 'fleet track time + recorded per-vehicle clock_drift',
            'position_rmse_m': float(np.sqrt(np.mean(position_error ** 2))),
            'checks': {name: {'value': value, 'limit': limit, 'passed': value <= limit + 1e-12}
                       for name, (value, limit) in checks.items()},
            'mpc_failures': max(int(float(row['mpc_failures'])) for row in selected),
            'failures': failures,
        }
    return output


def latest_bin(vehicle):
    candidates = sorted((vehicle / 'logs').glob('*.BIN'))
    if not candidates:
        raise FileNotFoundError('no DataFlash BIN in ' + str(vehicle))
    return max(candidates, key=lambda path: path.stat().st_size)


def dynamic_audit(vehicle, platform, protocol):
    output, report = analysis_output(vehicle)
    analysis_path = output / 'analysis.json'
    if report is None:
        report = analyse(latest_bin(vehicle), vehicle / 'telemetry.csv', output,
                         events=vehicle / 'events.jsonl')
    signal = 'sim2_airspeed_m_s' if platform == 'fixedwing' else 'sim2_ground_speed_m_s'
    primary = report['derivatives'][signal]['resampled_savitzky_golay']['phases'].get('track')
    if not primary or primary.get('samples', 0) < 3:
        raise ValueError('no valid track-phase primary dynamic signal')
    protocol_platform = 'multicopter' if platform == 'multirotor' else platform
    limits = protocol['dynamic_thresholds'][protocol_platform]
    checks = {
        'p99_abs_m_s2': {'value': primary['p99_abs'], 'limit': limits['p99_abs_m_s2']},
        'rms_m_s2': {'value': primary['rms'], 'limit': limits['rms_m_s2']},
    }
    for item in checks.values():
        item['passed'] = item['value'] <= item['limit'] + 1e-12
    return {'source': signal, 'processing': 'resampled_savitzky_golay',
            'checks': checks, 'analysis': str(analysis_path.relative_to(vehicle.parent))}


def audit(directory, allow_partial=False):
    directory = Path(directory).resolve()
    cfg, members = load_config(directory / 'config.json')
    protocol_path = ROOT / cfg['frozen_contract']['formal_protocol']['path']
    protocol = yaml.safe_load(protocol_path.read_text())
    status = json.loads((directory / 'flight_status.json').read_text())
    tracking = aligned_tracking(directory, cfg, members, protocol['tracking_thresholds'])
    vehicles = {}
    invalid = []
    acceptance_failures = []
    for member in members:
        vehicle = directory / ('vehicle_%d' % member.member_id)
        packets, journal = read_packets(vehicle)
        source_evidence = raw_source_summary(vehicle / 'raw.tlog', .11)
        timing = timing_summary(packets, journal, .11, ('track',), source_evidence)
        write_json(vehicle / 'v3_formal_timing_audit.json', timing)
        if not timing['passed']:
            invalid.append('member_%d_command_timing' % member.member_id)
        dynamic = dynamic_audit(vehicle, member.platform, protocol)
        if not all(item['passed'] for item in dynamic['checks'].values()):
            acceptance_failures.append('member_%d_dynamic' % member.member_id)
        if tracking[str(member.member_id)]['failures']:
            acceptance_failures.append('member_%d_tracking' % member.member_id)
        if protocol['run_eligibility']['require_zero_mpc_failures'] and member.platform == 'multirotor':
            if tracking[str(member.member_id)]['mpc_failures'] != 0:
                acceptance_failures.append('member_%d_mpc_failures' % member.member_id)
        gcs_path = vehicle / 'gcs_forwarding.json'
        gcs = json.loads(gcs_path.read_text()) if gcs_path.is_file() else {'errors': None}
        vehicles[str(member.member_id)] = {'timing': timing, 'dynamic': dynamic,
                                           'tracking': tracking[str(member.member_id)], 'gcs': gcs}
    complete = bool(status.get('completed') and status.get('exit_completed'))
    full_route = bool(status.get('full_route_completed'))
    if invalid:
        classification = 'INVALID_EVIDENCE'
    elif acceptance_failures:
        classification = 'ACCEPTANCE_FAILED'
    elif complete and full_route:
        classification = 'SYSTEM_QUALIFIED'
    elif allow_partial and complete:
        classification = 'SYSTEM_PRECHECK_QUALIFIED'
    else:
        classification = 'RUN_FAILED'
    report = {
        'schema_version': 1,
        'classification': classification,
        'formal_qualified': classification == 'SYSTEM_QUALIFIED',
        'system_precheck_qualified': classification in ('SYSTEM_QUALIFIED', 'SYSTEM_PRECHECK_QUALIFIED'),
        'allow_partial': allow_partial,
        'lifecycle': status,
        'invalid_reasons': invalid,
        'acceptance_failures': acceptance_failures,
        'vehicles': vehicles,
        'protocol': str(protocol_path),
        'scope': 'current frozen v3 SITL work domain; not real-aircraft capability',
    }
    write_json(directory / 'v3_formal_acceptance.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--allow-partial', action='store_true')
    args = parser.parse_args()
    report = audit(args.directory, args.allow_partial)
    print(json.dumps({'classification': report['classification'],
                      'invalid_reasons': report['invalid_reasons'],
                      'acceptance_failures': report['acceptance_failures']}, indent=2))
    return 0 if report['system_precheck_qualified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
