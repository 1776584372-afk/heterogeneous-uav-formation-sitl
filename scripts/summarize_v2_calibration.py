#!/usr/bin/env python3
"""Summarize v2 calibration repeats without selecting capability thresholds."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics


QUALIFICATION_PREFIXES = ('accel_', 'decel_', 'settle_', 'steady_', 'turn_')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numeric_summary(values):
    values = [float(value) for value in values]
    if not values:
        return None
    return {
        'count': len(values),
        'mean': statistics.mean(values),
        'sample_standard_deviation': statistics.stdev(values) if len(values) > 1 else 0.0,
        'median': statistics.median(values),
        'minimum': min(values),
        'maximum': max(values),
    }


def platform_of(protocol, analysis):
    if protocol.get('platform') in ('copter', 'plane'):
        return protocol['platform']
    controller = str(protocol.get('controller', '')).lower()
    if 'arducopter' in controller:
        return 'copter'
    if 'arduplane' in controller:
        return 'plane'
    if analysis['record_counts'].get('PSCE', 0):
        return 'copter'
    raise ValueError('platform cannot be identified from protocol or analysis')


def qualifying_phases(mapping):
    return {name: value for name, value in mapping.items()
            if name.startswith(QUALIFICATION_PREFIXES)}


def actuator_fraction(controller, phase):
    actuator = controller.get('actuator', {})
    fractions = []
    for group in actuator.values():
        item = group.get('phases', {}).get(phase, {})
        fractions.extend(value for name, value in item.items()
                         if name.endswith('_fraction'))
    return max(fractions, default=None)


def load_run(path, analysis_name='v2_analysis'):
    path = Path(path).resolve()
    result_path = path / 'result.json'
    if analysis_name == 'auto':
        analysis_name = ('v2_analysis_hybrid_clock'
                         if (path / 'v2_analysis_hybrid_clock' / 'analysis.json').is_file()
                         else 'v2_analysis')
    analysis_path = path / analysis_name / 'analysis.json'
    if not result_path.is_file() or not analysis_path.is_file():
        raise ValueError(f'incomplete run: {path}')
    result = json.loads(result_path.read_text())
    analysis = json.loads(analysis_path.read_text())
    protocol = result.get('protocol', {})
    platform = platform_of(protocol, analysis)
    signal = ('sim2_ground_speed_m_s' if platform == 'copter'
              else 'sim2_airspeed_m_s')
    primary = analysis['derivatives'][signal]['resampled_savitzky_golay']['phases']
    controller = analysis.get('controller_diagnostics', {})
    records = []
    for phase, metric in qualifying_phases(primary).items():
        records.append({
            'run': str(path), 'protocol_id': protocol.get('protocol_id', path.name),
            'repeat_id': protocol.get('repeat_id'), 'platform': platform,
            'phase': phase, 'samples': metric['samples'],
            'minimum': metric['minimum'], 'maximum': metric['maximum'],
            'maximum_abs': metric['maximum_abs'], 'p95_abs': metric['p95_abs'],
            'p99_abs': metric['p99_abs'], 'rms': metric['rms'],
            'maximum_actuator_endpoint_fraction': actuator_fraction(controller, phase),
            'run_completed': result.get('status') == 'COMPLETED',
            'tracking_passed': bool(result.get('all_tested_points_passed')),
            'exit_completed': bool(result.get('exit_completed')),
        })
    return {
        'path': str(path), 'protocol': protocol, 'platform': platform,
        'analysis_name': analysis_name,
        'result_sha256': sha256(result_path), 'analysis_sha256': sha256(analysis_path),
        'records': records,
    }


def summarize(paths, analysis_name='v2_analysis'):
    runs = [load_run(path, analysis_name) for path in paths]
    grouped = {}
    for run in runs:
        for record in run['records']:
            grouped.setdefault((record['protocol_id'], record['phase']), []).append(record)
    aggregates = []
    for (protocol_id, phase), records in sorted(grouped.items()):
        aggregates.append({
            'protocol_id': protocol_id, 'phase': phase,
            'platform': records[0]['platform'], 'repeats': len(records),
            'tracking_pass_count': sum(record['tracking_passed'] for record in records),
            'complete_exit_count': sum(record['exit_completed'] for record in records),
            'minimum': numeric_summary(record['minimum'] for record in records),
            'maximum': numeric_summary(record['maximum'] for record in records),
            'maximum_abs': numeric_summary(record['maximum_abs'] for record in records),
            'p95_abs': numeric_summary(record['p95_abs'] for record in records),
            'p99_abs': numeric_summary(record['p99_abs'] for record in records),
            'rms': numeric_summary(record['rms'] for record in records),
            'maximum_actuator_endpoint_fraction': numeric_summary(
                record['maximum_actuator_endpoint_fraction'] for record in records
                if record['maximum_actuator_endpoint_fraction'] is not None),
        })
    return {
        'schema_version': 1,
        'scope': ('A/B evidence summary only; no capability, planning, or acceptance '
                  'threshold is selected by this script'),
        'analysis_name': analysis_name, 'runs': runs, 'aggregates': aggregates,
        'script_sha256': sha256(Path(__file__)),
    }


def write_outputs(report, output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    records = [record for run in report['runs'] for record in run['records']]
    if records:
        with (output / 'per_run_phase.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader(); writer.writerows(records)
    lines = ['# v2 Calibration Repeat Summary', '', report['scope'], '',
             '| Protocol | Phase | Repeats | Pass | Min worst | Max worst | P99 abs worst | RMS worst |',
             '|---|---|---:|---:|---:|---:|---:|---:|']
    for item in report['aggregates']:
        lines.append('| {protocol_id} | {phase} | {repeats} | {tracking_pass_count} | '
                     '{minimum[minimum]:.6f} | {maximum[maximum]:.6f} | '
                     '{p99_abs[maximum]:.6f} | {rms[maximum]:.6f} |'.format(**item))
    (output / 'summary.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--analysis-name', default='v2_analysis')
    args = parser.parse_args()
    report = summarize(args.runs, args.analysis_name)
    write_outputs(report, args.output)
    print(json.dumps({'runs': len(report['runs']),
                      'phase_groups': len(report['aggregates']),
                      'output': str(args.output)}, indent=2))


if __name__ == '__main__':
    main()
