#!/usr/bin/env python3
"""Reproduce the bounded 2026-09-27 execution contrasts from saved evidence."""
import argparse
import json
from pathlib import Path
import shutil

from pymavlink import mavutil

from scripts.audit_outgoing_packets import read_journal
from scripts.extract_onboard_control import file_hash


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    base, root = args.baseline.resolve(), args.directory.resolve()
    hashes = {}

    def read(path):
        hashes[str(path)] = file_hash(path)
        return json.loads(path.read_text())

    def span(metric):
        return {k: metric[k] for k in ('minimum', 'maximum', 'lower_violation_fraction',
                'upper_violation_fraction') if k in metric}

    def rotor(folder, run_name):
        run = folder/run_name
        acceptance = read(run/'acceptance.json')
        feedback = read(run/'feedback_audit.json')
        dynamics = read(folder/'rotor_dynamics/diagnosis.json')['members']['0']
        truth = read(folder/'rotor_truth/diagnostic.json')
        return dict(run=str(run), lifecycle=acceptance['lifecycle'],
            tracking_qualified=acceptance['tracking_qualified'],
            measured_constraints_passed=acceptance['measured_constraints_passed'],
            position=acceptance['members']['0'], feedback=feedback,
            acceleration_telemetry=span(dynamics['tangential_accel']),
            acceleration_truth=span(truth['simulator_truth']),
            yaw_rate_telemetry=span(dynamics['actual_yaw_rate']),
            yaw_rate_truth=truth['yaw_rate']['simulator_truth'])

    def plane(folder, run_name):
        result = read(folder/run_name/'result.json')
        dynamics = read(folder/'plane_truth.json')
        return dict(run=str(folder/run_name), status=result['status'],
            steady_pass=result['all_tested_points_passed'], exit_completed=result['exit_completed'],
            protocol=result['protocol'], phases={k: dict(telemetry=span(v['telemetry']),
                simulator_truth=span(v['simulator_truth'])) for k, v in dynamics['phases'].items()})

    manifests = {}
    for label, path in [('rotor_baseline', base/'rotor_tangent_01/vehicle_0'),
                        ('rotor_angle_only', root/'rotor_angle_only_01/vehicle_0'),
                        ('plane_baseline', base/'plane_slew_01'),
                        ('plane_quintic', root/'plane_quintic_01')]:
        manifest = read(path/'manifest.json')
        snapshot_mismatches = []
        current_changes = []
        for source, expected in manifest['source_sha256'].items():
            snapshot = path/'source_snapshot'/Path(source).name
            if not snapshot.is_file() or file_hash(snapshot) != expected:
                snapshot_mismatches.append(source)
            if not Path(source).is_file() or file_hash(Path(source)) != expected:
                current_changes.append(source)
        manifests[label] = dict(model=manifest['model'], binary_sha256=manifest['binary_sha256'],
            ardupilot_commit=manifest['ardupilot_commit'], defaults_sha256=file_hash(path/'defaults.parm'),
            snapshot_mismatches=snapshot_mismatches, current_changes_since_run=current_changes,
            model_source_hashes={k: v for k, v in manifest['source_sha256'].items()
                if k.endswith(('.parm', '.json'))})

    protocol = read(root/'protocol.json')
    capability = Path(protocol['capability_file']).resolve()
    hashes[str(capability)] = file_hash(capability)
    conditions = {}
    for platform, trial in [('rotor', 'angle_only'), ('plane', 'quintic')]:
        before, after = manifests[platform+'_baseline'], manifests[platform+'_'+trial]
        conditions[platform] = {k: before[k] == after[k] for k in
            ('model', 'binary_sha256', 'ardupilot_commit', 'defaults_sha256', 'model_source_hashes')}

    packets, journal = read_journal(root/'plane_quintic_01/outgoing_packets.jsonl')
    initial = []
    with (root/'plane_quintic_01/outgoing_packets.jsonl').open() as stream:
        for _ in range(2):
            event = json.loads(next(stream))
            initial.append(dict(send_index=event['send_index'], magic=event['packet_hex'][:2]))
    dependency = Path(mavutil.__file__).resolve()
    source_dir = root/'diagnostic_source_snapshot'
    source_dir.mkdir(exist_ok=True)
    repository = Path(__file__).resolve().parents[1]
    sources = [dependency, *[repository/p for p in (
        'scripts/summarize_execution_isolation.py', 'scripts/summarize_onboard_probe.py',
        'scripts/extract_onboard_control.py', 'scripts/run_paper_sitl.py',
        'tests/test_onboard_probe_summary.py', 'tests/test_copter_yaw_policy.py')],
        Path.home()/'ardupilot/libraries/AC_AttitudeControl/AC_AttitudeControl.cpp',
        Path.home()/'ardupilot/ArduPlane/GCS_MAVLink_Plane.cpp',
        Path.home()/'ardupilot/ArduPlane/Plane.h']
    for source in sources:
        shutil.copy2(source, source_dir/source.name)
        hashes[str(source)] = file_hash(source)

    result = dict(formal_qualification=False, physical_limits_changed=False,
        contrasts=dict(rotor_baseline=rotor(base, 'rotor_tangent_01'),
            rotor_angle_only=rotor(root, 'rotor_angle_only_01'),
            plane_baseline=plane(base, 'plane_slew_01'), plane_quintic=plane(root, 'plane_quintic_01')),
        matched_model_conditions=conditions, manifests=manifests,
        frozen_capability_hash_matches=hashes[str(capability)] == protocol['capability_sha256'],
        plane_packet_audit=read(root/'plane_socket/audit.json'),
        plane_initial_protocol_transition=dict(packets=[{k: p[k] for k in
            ('send_index', 'seq', 'type', 'phase')} for p in packets[:2]], magic=initial,
            original_issues=journal['issues'],
            explanation='mavutil.auto_mavlink_version rebuilds the encoder when v1 switches to v2; its sequence restarts. Raw strict audit remains failed; no transport loss inferred from this startup event.',
            limitation='local socket writes and OFG receiver states do not prove every packet was applied'),
        plane_onboard=read(root/'plane_onboard_summary_v2.json'),
        baseline_plane_onboard=read(root/'baseline_plane_onboard_summary_v2.json'),
        rotor_onboard=read(root/'rotor_onboard_summary.json'),
        baseline_rotor_onboard=read(root/'baseline_rotor_onboard_summary.json'),
        conclusions=['Neither proposed change passes unchanged dynamic limits in this single trial.',
            'Reject as default repairs; keep both raw failures. No significance claim from one pair.',
            'Do not equate quaternion z-twist with Euler yaw or OFG slew-location sentinel with command-186 altitude.'],
        input_sha256=hashes)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(matched_model_conditions=conditions,
        frozen_capability_hash_matches=result['frozen_capability_hash_matches'],
        snapshots_valid=all(not m['snapshot_mismatches'] for m in manifests.values())), indent=2))


if __name__ == '__main__':
    main()
