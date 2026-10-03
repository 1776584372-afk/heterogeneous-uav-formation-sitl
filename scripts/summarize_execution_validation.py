#!/usr/bin/env python3
"""Recompute every attempt in one development directory, preserving raw evidence."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from scripts.audit_tracking_run import audit
from scripts.audit_measured_feedback import audit as audit_feedback
from uav_formation.experiment_artifacts import write_json


def summarize(root):
    root=Path(root).resolve()
    results={}; rows=[]; hashes={}
    for status_path in sorted(root.glob('*/flight_status.json')):
        directory=status_path.parent
        result=audit(directory)
        cfg=json.loads((directory/'config.json').read_text())
        if any(m['platform']=='multirotor' for m in cfg['members']):
            result['feedback_audit']=audit_feedback(directory)
        results[directory.name]=result
        for member in cfg['members']:
            i=str(member['member_id'])
            stats=result.get('members',{}).get(i,{})
            rows.append(dict(run=directory.name,member=i,platform=member['platform'],
                full_route=result['full_route_completed'],tracking=result['tracking_qualified'],
                actual_constraints=result['measured_constraints_passed'],formal=result['formal_qualified'],
                rmse_m=stats.get('tracking_rmse'),maximum_error_m=stats.get('tracking_max'),
                violations=json.dumps(result.get('violations',{}).get(i,{}),sort_keys=True),
                audit_error=result.get('audit_error'),
                trajectory=str(directory/'flight.csv'),configuration=str(directory/'config.json')))
        # Large binary logs are hashed incrementally and remain at their original paths.
        sources=[directory/'config.json',directory/'flight.csv',directory/'flight_status.json',
                 directory/'feedback_solves.jsonl',directory/'fixedwing_commands.jsonl',
                 *directory.glob('vehicle_*/logs/*.BIN'),*directory.glob('vehicle_*/raw.tlog')]
        for path in sources:
            if path.is_file():
                digest=hashlib.sha256()
                with path.open('rb') as stream:
                    for block in iter(lambda:stream.read(1024*1024),b''): digest.update(block)
                hashes[str(path.relative_to(root))]=digest.hexdigest()
    summary=dict(scope='development attempts, not formal repeated evidence',runs=results,
        counts=dict(attempts=len(results),full_routes=sum(r['full_route_completed'] for r in results.values()),
                    tracking_passed=sum(r['tracking_qualified'] for r in results.values()),
                    measured_constraints_passed=sum(r['measured_constraints_passed'] for r in results.values()),
                    formal_passed=sum(r['formal_qualified'] for r in results.values())))
    write_json(root/'summary.json',summary)
    write_json(root/'raw_evidence_sha256.json',hashes)
    with (root/'summary.csv').open('w',newline='') as stream:
        if rows:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    print(json.dumps(summary['counts'],indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    summarize(parser.parse_args().directory)
