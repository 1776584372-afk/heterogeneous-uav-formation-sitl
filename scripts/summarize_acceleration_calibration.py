#!/usr/bin/env python3
"""Summarize bounded acceleration calibration runs without changing limits."""
import argparse
import json
from pathlib import Path


def summarize_run(path):
    path=Path(path)
    result=json.loads((path/'result.json').read_text())
    protocol=result['protocol']
    manifest=json.loads((path/'manifest.json').read_text())
    platform=manifest['vehicle']
    if platform not in ('copter','plane'):
        raise ValueError('unsupported vehicle in manifest: %s' % platform)
    truth_path=path/'truth_phases.json'
    if not truth_path.is_file():
        return dict(run=str(path.resolve()),status='INCOMPLETE',reason='truth_phases.json missing')
    truth=json.loads(truth_path.read_text())
    phase_rows=[]
    for phase,entry in truth.get('phases',{}).items():
        sim=entry.get('simulator_truth',{})
        telemetry=entry.get('telemetry',{})
        phase_rows.append(dict(phase=phase,window=entry.get('window'),
            truth_min=sim.get('minimum'),truth_max=sim.get('maximum'),
            telemetry_min=telemetry.get('minimum'),telemetry_max=telemetry.get('maximum'),
            truth_upper_violation_fraction=sim.get('upper_violation_fraction'),
            truth_lower_violation_fraction=sim.get('lower_violation_fraction')))
    if not phase_rows:
        return dict(run=str(path.resolve()),status='INCOMPLETE',reason='no step phases')
    limits={'copter':.4,'plane':.2}
    limit=limits[platform]
    return dict(run=str(path.resolve()),status='OBSERVATION_ONLY',platform=platform,
        model=manifest['model'],
        reference_ramp_m_s2=protocol.get('reference_ramp_m_s2'),
        frozen_comparison_limit_m_s2=limit,
        result_status=result.get('status'),all_tested_points_passed=result.get('all_tested_points_passed'),
        phases=phase_rows,
        interpretation='Observed response only; this report does not promote a sampled peak to a physical capability limit')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    reports=[summarize_run(path) for path in args.runs]
    output=args.output.resolve(); output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(schema_version=1,runs=reports),indent=2,allow_nan=False)+'\n')
    lines=['# Acceleration Calibration Follow-up','',
        'Observation-only follow-up. Frozen comparison values remain unchanged.', '',
        '| Run | Platform | Reference ramp | SIM2 min | SIM2 max |', '|---|---|---:|---:|---:|']
    for report in reports:
        phases=report.get('phases',[])
        sim=[p for p in phases if p['truth_min'] is not None and p['truth_max'] is not None]
        lo=min((p['truth_min'] for p in sim),default=None)
        hi=max((p['truth_max'] for p in sim),default=None)
        lines.append('| %s | %s | %s | %s | %s |' % (
            Path(report['run']).name,report.get('platform','unknown'),
            report.get('reference_ramp_m_s2','unknown'),lo,hi))
    lines += ['', 'No value in this report is a new frozen capability or formal qualification.', '']
    output.with_suffix('.md').write_text('\n'.join(lines))
    print(output)


if __name__=='__main__':
    main()
