import csv
import json

from uav_formation.capability_model import MemberSpec
from scripts.audit_v3_formal_run import aligned_tracking, analysis_output
from scripts.analyze_v2_bin import merged_phase_map


def test_tracking_uses_recorded_vehicle_clock_drift(tmp_path):
    with (tmp_path / 'member_reference.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['t', 'member', 'x', 'y', 'course', 'speed'])
        writer.writeheader()
        for t in (0., 1., 2.):
            writer.writerow(dict(t=t, member=0, x=10*t, y=0., course=0., speed=10.))
    (tmp_path / 'world_origin.json').write_text(json.dumps({'origin': [0., 0.]}))
    fields = ['t', 'phase', 'member', 'clock_drift', 'x', 'y', 'vx', 'vy',
              'airspeed', 'alt', 'roll', 'mpc_failures']
    with (tmp_path / 'flight.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        # At fleet t=0.9 the member clock is at reference t=1.0.
        writer.writerow(dict(t=.9, phase='track', member=0, clock_drift=.1,
                             x=10., y=0., vx=10., vy=0., airspeed=10., alt=60.,
                             roll=0., mpc_failures=0))
        writer.writerow(dict(t=1.0, phase='track', member=0, clock_drift=.1,
                             x=11., y=0., vx=10., vy=0., airspeed=10., alt=60.,
                             roll=0., mpc_failures=0))
        writer.writerow(dict(t=1.1, phase='track', member=0, clock_drift=.1,
                             x=12., y=0., vx=10., vy=0., airspeed=10., alt=60.,
                             roll=0., mpc_failures=0))
    member = MemberSpec(0, 'multirotor', 0., 0., 20., 3.)
    thresholds = dict(maximum_position_error_m=1e-9, maximum_speed_error_m_s=1e-9,
                      maximum_course_error_deg=1e-9, maximum_altitude_error_m=1e-9,
                      maximum_fixedwing_roll_deg=27.)
    result = aligned_tracking(tmp_path, {'altitude': 60.}, (member,), thresholds)
    assert result['0']['position_rmse_m'] < 1e-12
    assert result['0']['failures'] == []


def test_native_exit_event_overrides_duplicate_track_telemetry(tmp_path):
    telemetry = tmp_path / 'telemetry.csv'
    with telemetry.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['sim_s', 'phase'])
        writer.writeheader()
        writer.writerows([
            {'sim_s': 1., 'phase': 'join'},
            {'sim_s': 2., 'phase': 'track'},
            {'sim_s': 10., 'phase': 'exit'},
            {'sim_s': 10., 'phase': 'track'},
        ])
    events = tmp_path / 'events.jsonl'
    events.write_text(json.dumps({'event': 'phase_transition', 'boot_s': 10.,
                                  'phase': 'exit'}) + '\n')
    times, phases = merged_phase_map(telemetry, events)
    assert list(zip(times, phases))[-1] == (10., 'exit')


def test_analysis_output_is_versioned_and_never_reuses_wrong_hash(tmp_path):
    output, report = analysis_output(tmp_path)
    assert report is None
    output.mkdir()
    (output / 'analysis.json').write_text(json.dumps({'analysis_sha256': 'old'}))

    revised, report = analysis_output(tmp_path)

    assert report is None
    assert revised != output
    assert revised.name.endswith('_revision_2')
