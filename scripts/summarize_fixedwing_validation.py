#!/usr/bin/env python3
"""Export development evidence without changing raw logs or capability inputs."""
import argparse
import csv
import difflib
import hashlib
import json
from pathlib import Path
import shutil

from uav_formation.capability_model import team_interval
from uav_formation.experiment_config import load_config
from uav_formation.experiment_artifacts import ROOT, write_json


CHANGED = (
    'uav_formation/paper_tracking.py', 'uav_formation/fixedwing_adapter.py',
    'uav_formation/sitl_vehicle.py', 'scripts/run_paper_sitl.py',
    'scripts/paper_batch.py', 'scripts/compare_reference.py',
    'scripts/plot_paper_run.py', 'scripts/audit_measured_feedback.py',
)
ADDED = (
    'scripts/prepare_fixedwing_validation.py', 'scripts/audit_fixedwing_validation.py',
    'scripts/audit_tracking_run.py', 'scripts/summarize_fixedwing_validation.py',
    'tests/test_plane_course.py', 'tests/test_validation_gates.py',
    'docs/固定翼制导与S1S2S3开发验收_20260925.md',
)


def csv_table(path, rows):
    if rows:
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def summarize(root):
    runs = {}
    rows = []
    for status_path in sorted(root.glob('*/flight_status.json')):
        directory = status_path.parent
        status = json.loads(status_path.read_text())
        acceptance_path = directory/'acceptance.json'
        metrics_path = directory/'metrics.json'
        acceptance = json.loads(acceptance_path.read_text()) if acceptance_path.exists() else {}
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
        cfg = json.loads((directory/'config.json').read_text())
        runs[directory.name] = dict(status=status, acceptance=acceptance,
            metrics=metrics, config=str((directory/'config.json').resolve()),
            interpretation='single development attempt, not formal repeated evidence')
        for member in cfg['members']:
            m = metrics.get('members', {}).get(str(member['member_id']), {})
            rows.append(dict(run=directory.name, member=member['member_id'],
                platform=member['platform'], slot_m=member['r_y'],
                completed=status.get('completed', False),
                full_route=status.get('full_route_completed', False),
                tracking_qualified=acceptance.get('tracking_qualified', False),
                measured_constraints_passed=acceptance.get('measured_constraints_passed', False),
                formal_qualified=acceptance.get('formal_qualified', False),
                tracking_duration_s=metrics.get('tracking_duration'),
                radius_m=metrics.get('selected_radius'),
                rmse_m=m.get('tracking_rmse'), max_error_m=m.get('tracking_max'),
                speed_min_m_s=m.get('speed_min'), speed_max_m_s=m.get('speed_max'),
                bank_max_deg=m.get('bank_abs_max_deg'),
                height_error_max_m=m.get('altitude_error_max'),
                solver_failures=m.get('mpc_failures') if member['platform']=='multirotor' else None,
                violations=json.dumps(m.get('violation_time_fraction', {}), sort_keys=True),
                raw_trajectory=str((directory/'flight.csv').resolve()),
                frozen_config=str((directory/'config.json').resolve())))
    write_json(root/'flight_summary.json', runs)
    csv_table(root/'flight_summary.csv', rows)

    slots = []
    for turn, sign in (('left', 1), ('right', -1)):
        for side in ('negative', 'positive'):
            cfg, members = load_config(root/'tasks_v2'/('s3_%s_wing_%s.yaml' % (turn, side)))
            wing = next(m for m in members if m.platform=='fixedwing')
            for radius in (80., 160.):
                kappa = sign/radius
                lower, upper, lower_owner, upper_owner = team_interval(members, kappa)
                for m in members:
                    q = 1-m.r_y*kappa
                    slots.append(dict(first_turn=turn, wing_side='inner' if wing.r_y*sign>0 else 'outer',
                        radius_m=radius, member=m.member_id, platform=m.platform, slot_m=m.r_y,
                        kappa_per_m=kappa, q=q, nominal_progress_m_s=14.,
                        nominal_member_speed_m_s=14.*q,
                        lower_progress_m_s=lower, upper_progress_m_s=upper,
                        static_nonempty=lower<=upper, nominal_progress_feasible=lower<=14.<=upper,
                        lower_owner=lower_owner[0], lower_limit=int(lower_owner[1]),
                        upper_owner=upper_owner[0], upper_limit=int(upper_owner[1]),
                        scope='constant-curvature sample only; full-route dynamics assessed separately'))
    csv_table(root/'slot_comparison.csv', slots)

    snapshot = root/'delivery_source'
    patches = []
    sources = []
    for relative in CHANGED+ADDED:
        source = ROOT/relative
        before = root/'before'/source.name
        if relative in CHANGED and not before.is_file():
            raise FileNotFoundError('missing task baseline: '+str(before))
        old = before.read_text().splitlines(keepends=True) if relative in CHANGED else []
        new = source.read_text().splitlines(keepends=True)
        patches.extend(difflib.unified_diff(old, new,
            fromfile='before/'+relative if relative in CHANGED else '/dev/null',
            tofile='after/'+relative))
        destination = snapshot/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        sources.append(dict(path=relative, sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    (root/'source_changes.patch').write_text(''.join(patches))
    write_json(root/'delivery_source_sha256.json', sources)
    shutil.copy2(ROOT/'docs/固定翼制导与S1S2S3开发验收_20260925.md', root/'交付说明.md')
    print(json.dumps(dict(attempts=len(runs), member_rows=len(rows),
        full_route=sum(bool(r['status'].get('full_route_completed')) for r in runs.values()),
        tracking_qualified=sum(bool(r['acceptance'].get('tracking_qualified')) for r in runs.values()),
        formal_qualified=sum(bool(r['acceptance'].get('formal_qualified')) for r in runs.values())), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    summarize(parser.parse_args().directory)
