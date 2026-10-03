#!/usr/bin/env python3
"""Summarize every screening attempt without promoting points to capabilities."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from scripts.analyze_backend_dynamics import analyze


REQUIRED_BOUNDS = {
    'copter': {'groundspeed': (0., 20.), 'tangential_accel': (-.4, .4),
               'lateral_accel': (0., 3.), 'yaw_rate': (-.25, .25)},
    'plane': {'airspeed': (12., 24.), 'tangential_accel': (-.2, .2),
              'lateral_accel': (0., 2.5), 'bank_deg': (-27., 27.)},
}


def bounds_pass(metrics, platform=None):
    if platform is not None:
        if platform not in REQUIRED_BOUNDS:
            return False
        for name, (lower, upper) in REQUIRED_BOUNDS[platform].items():
            metric = metrics.get(name, {})
            if not isinstance(metric, dict) or (metric.get('lower_bound'), metric.get('upper_bound')) != (lower, upper):
                return False
    checked = 0
    for metric in metrics.values():
        if not isinstance(metric, dict):
            continue
        for side, extremum in [('lower', 'minimum'), ('upper', 'maximum')]:
            if side+'_bound' not in metric and side+'_violation_fraction' not in metric:
                continue
            values = [metric.get(key) for key in (side+'_bound', side+'_violation_fraction', extremum)]
            if any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in values):
                return False
            bound, fraction, peak = values
            if fraction != 0 or (peak < bound-.001 if side == 'lower' else peak > bound+.001):
                return False
            checked += 1
    return checked > 0


def expected_points(protocol):
    speeds = protocol.get('speeds', [])
    if 'banks' in protocol:
        return len(speeds)*len(protocol['banks'])*len(protocol.get('turn_signs', []))
    return len(speeds)


def summarize(root):
    attempts=[]
    frozen_checks=[]
    for batch_file in sorted(root.glob('*/batch.json')):
        batch=json.loads(batch_file.read_text())
        frozen_checks.append(batch.get('frozen_unchanged',False))
        for job in batch['jobs']:
            folder=batch_file.parent/job['name']
            result=job.get('result',{})
            item=dict(stage=batch['stage'],candidate=job['name'],run=str(folder.resolve()),
                returncode=job.get('returncode'),error=job.get('error',result.get('error')),
                tracking_points_passed=sum(s['status']=='PASS' for s in result.get('steps',[])),
                tracking_points_total=len(result.get('steps',[])),
                all_requested_tracking_points_passed=result.get('all_tested_points_passed',False),
                exit_completed=result.get('exit_completed',False),
                steady_windows=[],complete_phase_dynamic_pass=False,steady_dynamic_pass=False)
            expected = expected_points(result.get('protocol', {}))
            item['requested_points'] = expected
            item['measurement_scope'] = 'step/turn only; takeoff, warm-up, path entry and exit are not dynamic-certified'
            item['complete_tracking_record'] = bool(expected and expected == item['tracking_points_total']
                and expected == item['tracking_points_passed'] and item['all_requested_tracking_points_passed'])
            try:
                # Recompute from raw telemetry; retain every historical dynamics.json unchanged.
                dynamics=analyze(folder)
                platform=dynamics['platform']
                item['complete_phase_dynamic_pass']=bool(expected and len(dynamics['phases'])==expected) and all(
                    bounds_pass(m, platform) for m in dynamics['phases'].values())
                item['steady_dynamic_pass']=bool(expected and len(dynamics['steady_windows'])==expected) and all(
                    bounds_pass(w.get('metrics',{}), platform) for w in dynamics['steady_windows'])
                item['evaluated_phases'] = dynamics['phases']
                item['telemetry_sha256'] = dynamics['telemetry_sha256']
                for window in dynamics['steady_windows']:
                    metrics=window.get('metrics',{})
                    item['steady_windows'].append(dict(speed=window['speed'],sign=window.get('sign'),
                        tracking_status=window['tracking_status'],metrics=metrics,
                        dynamic_pass=bounds_pass(metrics, platform)))
            except (OSError, ValueError, KeyError, TypeError) as exc:
                item['analysis_error'] = str(exc)
            item['screening_pass']=bool(item['complete_tracking_record'] and job.get('returncode')==0
                and result.get('status')=='COMPLETED' and not item['error'] and not item.get('analysis_error')
                and item['complete_phase_dynamic_pass'] and item['steady_dynamic_pass'] and item['exit_completed'])
            attempts.append(item)
    return dict(schema_version=2,development_only=True,formal_qualification=False,envelope_certified=False,
        old_frozen_limits_unchanged=bool(frozen_checks) and all(frozen_checks),
        interpretation='v1 comparison limits, not newly certified physical limits',
        evaluation='raw telemetry recomputed; peaks and duration violations both checked; incomplete records fail closed',
        bank_rate_capability='unknown; observed roll rates do not certify an allowable maximum',
        analysis_sources_sha256={str(path.resolve()):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (Path(__file__), Path(__file__).with_name('analyze_backend_dynamics.py'))},
        attempts=attempts)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root')
    args=parser.parse_args();root=Path(args.root).resolve()
    result=summarize(root)
    (root/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# Platform Screening','',
        'Development only. No new capability envelope or formal paper qualification.',
        'A completed/steady tracking test is not a full dynamic-constraint pass.','',
        '| Batch | Stage | Candidate | Tracking points | Steady dynamics | Entire step/turn dynamics | Exit |',
        '|---|---|---|---:|---|---|---|']
    for item in result['attempts']:
        lines.append('| '+Path(item['run']).parent.name+' | {stage} | {candidate} | {tracking_points_passed}/{tracking_points_total} | {steady_dynamic_pass} | {complete_phase_dynamic_pass} | {exit_completed} |'.format(**item))
    lines+=['','Full commands and predeclared protocols: each batch.json and per-run protocol.json.',
        'Current summary recomputes raw telemetry, including peak-only endpoint violations. Historical dynamics.json files remain unchanged.',
        'Dynamic scope: step/turn phases only. Takeoff, path entry and exit are not dynamic-certified; Exit means the lifecycle action completed.',
        'Raw evidence: telemetry.csv, events.jsonl, raw.tlog, logs/*.BIN; per-run source_snapshot and manifest.json.',
        'All failed attempts remain in this index. Original frozen capability file is not replaced.','']
    (root/'README.md').write_text('\n'.join(lines))
    hashes={}
    for pattern in ('*/batch.json','*/*/protocol.json','*/*/result.json','*/*/telemetry.csv',
                    '*/*/events.jsonl','*/*/raw.tlog','*/*/logs/*.BIN','*/*/defaults.parm','*/*/manifest.json',
                    '*/*/*truth*.json','*/screening_source.py','*/*/source_snapshot/*.py',
                    '*/*/source_snapshot/*.parm','*/*/source_snapshot/*.json'):
        for path in sorted(root.glob(pattern)):
            digest=hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
            hashes[str(path.relative_to(root))]=digest.hexdigest()
    (root/'evidence_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print('\n'.join(lines))


if __name__=='__main__':main()
