#!/usr/bin/env python3
"""Separate lifecycle completion, tracking qualification and measured constraints."""
import argparse
import json
from pathlib import Path

from uav_formation.paper_metrics import compute
from uav_formation.experiment_artifacts import write_json


def audit(directory):
    directory=Path(directory)
    cfg=json.loads((directory/'config.json').read_text())
    status=json.loads((directory/'flight_status.json').read_text())
    criteria=cfg['acceptance']
    manifest_path=directory/'run_manifest.json'
    run_kind=json.loads(manifest_path.read_text()).get('kind') if manifest_path.is_file() else None
    result=dict(criteria=criteria,lifecycle=status,
        full_route_completed=bool(status.get('full_route_completed') and status.get('completed')),
        tracking_qualified=False,measured_constraints_passed=False,formal_qualified=False,
        run_kind=run_kind)
    try:
        metrics=compute(directory)
        if not metrics.get('valid_telemetry'):
            raise ValueError('insufficient or incomplete tracking telemetry')
        result['tracking_qualified']=bool(result['full_route_completed'] and status.get('exit_completed') and all(
            m['tracking_rmse']<=criteria['tracking_rmse_m']
            and m['tracking_max']<=criteria['tracking_max_m']
            and m['altitude_error_max']<=criteria['altitude_error_max_m'] for m in metrics['members'].values()))
        violations={i:{k:v for k,v in m['violation_time_fraction'].items()
                       if v>criteria['maximum_constraint_violation_time_fraction']}
                    for i,m in metrics['members'].items()}
        result['violations']=violations
        result['measured_constraints_passed']=not any(violations.values())
        result['formal_qualified']=bool(result['tracking_qualified'] and result['measured_constraints_passed']
                                       and cfg.get('capability_evidence')=='measured'
                                       and not cfg.get('copter_reference_only') and run_kind=='paper_fleet')
        if cfg.get('copter_reference_only'):
            result['qualification_note']='direct-reference diagnostic bypasses NMPC; cannot qualify the paper NMPC branch'
        result['members']={i:{k:m[k] for k in ('tracking_rmse','tracking_max','speed_min','speed_max',
            'bank_abs_max_deg','altitude_error_max')} for i,m in metrics['members'].items()}
    except (ValueError,KeyError) as exc:
        result.update(tracking_qualified=False, measured_constraints_passed=False,
                      formal_qualified=False)
        result['audit_error']=str(exc)
    write_json(directory/'acceptance.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    print(json.dumps(audit(args.directory),indent=2))
