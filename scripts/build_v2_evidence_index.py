#!/usr/bin/env python3
"""Build a hash-verified index of all retained v2 A-group evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


QUALIFICATION_PREFIXES = ('accel_', 'decel_', 'settle_', 'steady_', 'turn_')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def platform_of(protocol, analysis):
    value = protocol.get('platform')
    if value in ('copter', 'plane'):
        return value
    controller = str(protocol.get('controller', '')).lower()
    if 'arducopter' in controller:
        return 'copter'
    if 'arduplane' in controller:
        return 'plane'
    counts = analysis.get('record_counts', {})
    if counts.get('PSCE', 0):
        return 'copter'
    if counts.get('TECS', 0):
        return 'plane'
    raise ValueError('platform cannot be identified')


def saturation_summary(analysis):
    maximum = 0.0
    phases = {}
    for group_name, group in analysis.get('controller_diagnostics', {}).get('actuator', {}).items():
        for phase, values in group.get('phases', {}).items():
            if not phase.startswith(QUALIFICATION_PREFIXES):
                continue
            fractions = [float(value) for key, value in values.items()
                         if key.endswith('_fraction')]
            if not fractions:
                continue
            value = max(fractions)
            maximum = max(maximum, value)
            phases[f'{group_name}/{phase}'] = value
    return {'maximum_endpoint_fraction': maximum, 'by_phase': phases,
            'interpretation': 'diagnostic configured endpoints; not physical thrust/surface margin'}


def index_run(run):
    result_path = run / 'result.json'
    analysis_path = run / 'v2_analysis' / 'analysis.json'
    archive_manifest_path = run / 'raw_archive_manifest.json'
    if not (result_path.is_file() and analysis_path.is_file()
            and archive_manifest_path.is_file()):
        raise ValueError(f'incomplete retained evidence: {run}')
    result = json.loads(result_path.read_text())
    analysis = json.loads(analysis_path.read_text())
    archive_manifest = json.loads(archive_manifest_path.read_text())
    archive_path = run / archive_manifest['archive']
    actual_archive_hash = sha256(archive_path)
    if actual_archive_hash != archive_manifest['archive_sha256']:
        raise RuntimeError(f'archive hash mismatch: {archive_path}')
    protocol = result.get('protocol', {})
    return {
        'run': str(run), 'platform': platform_of(protocol, analysis),
        'protocol_id': protocol.get('protocol_id'),
        'repeat_id': protocol.get('repeat_id'),
        'status': result.get('status'),
        'all_tested_points_passed': bool(result.get('all_tested_points_passed')),
        'exit_completed': bool(result.get('exit_completed')),
        'saturation': saturation_summary(analysis),
        'sha256': {
            'result.json': sha256(result_path),
            'v2_analysis/analysis.json': sha256(analysis_path),
            'raw_archive_manifest.json': sha256(archive_manifest_path),
            archive_manifest['archive']: actual_archive_hash,
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config-root', type=Path, default=Path('config/v2'))
    parser.add_argument('--docs-root', type=Path, default=Path('docs/v2'))
    parser.add_argument('--preflight', type=Path)
    args = parser.parse_args()
    root = args.root.resolve(); output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runs = []
    for group in sorted(root.glob('group_A_*')):
        if not group.is_dir():
            continue
        for run in sorted(group.iterdir()):
            if run.is_dir() and (run / 'result.json').is_file():
                runs.append(index_run(run))
    support = {}
    for base in (args.config_root.resolve(), args.docs_root.resolve()):
        for path in sorted(base.glob('*')):
            if path.is_file():
                support[str(path)] = sha256(path)
    if args.preflight is not None:
        for path in sorted(args.preflight.resolve().rglob('*')):
            if path.is_file():
                support[str(path)] = sha256(path)
    report = {
        'schema_version': 1,
        'scope': 'all archived group_A evidence; passing and failed runs retained',
        'root': str(root), 'runs': runs, 'supporting_file_sha256': support,
        'counts': {
            'runs': len(runs),
            'passed': sum(item['all_tested_points_passed'] for item in runs),
            'failed_or_not_qualified': sum(not item['all_tested_points_passed'] for item in runs),
        },
        'script_sha256': sha256(Path(__file__)),
    }
    (output / 'evidence_index.json').write_text(
        json.dumps(report, indent=2, allow_nan=False) + '\n')
    lines = ['# v2 Evidence Index', '', report['scope'], '',
             f"Runs: {report['counts']['runs']}; passed: {report['counts']['passed']}; "
             f"failed/not qualified: {report['counts']['failed_or_not_qualified']}.", '',
             '| Run | Platform | Protocol | Repeat | Passed | Max endpoint fraction | Archive SHA256 |',
             '|---|---|---|---:|---:|---:|---|']
    for item in runs:
        lines.append('| {run} | {platform} | {protocol_id} | {repeat_id} | {passed} | '
                     '{fraction:.6f} | `{archive}` |'.format(
                         run=Path(item['run']).name, platform=item['platform'],
                         protocol_id=item['protocol_id'], repeat_id=item['repeat_id'],
                         passed='yes' if item['all_tested_points_passed'] else 'no',
                         fraction=item['saturation']['maximum_endpoint_fraction'],
                         archive=item['sha256']['raw_evidence.tar.gz']))
    (output / 'evidence_index.md').write_text('\n'.join(lines) + '\n')
    print(json.dumps(report['counts'], indent=2))


if __name__ == '__main__':
    main()
