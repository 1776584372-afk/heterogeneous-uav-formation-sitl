import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from scripts.run_paper_sitl import (main, reference_copter_setpoint,
    rotor_development_gate, send_copter_setpoint)
from uav_formation.capability_model import MemberSpec
from uav_formation.sitl_vehicle import SitlVehicle


ROOT = Path(__file__).resolve().parents[1]


def wire_command(angle_only=False):
    stream = io.BytesIO()
    vehicle = SitlVehicle.__new__(SitlVehicle)
    vehicle.link = SimpleNamespace(mav=mavlink.MAVLink(stream))
    vehicle.target = {}
    vehicle.home_offset = (100., 200.)
    vehicle.altitude = 60.
    logged_rate = send_copter_setpoint(vehicle, (112., 225., .4), (12., 3.),
        (.1, .2), -.0875, yaw_angle_only=angle_only)
    packet = mavlink.MAVLink(None).parse_buffer(stream.getvalue())[0].to_dict()
    return packet, vehicle.target, logged_rate


def test_angle_only_changes_only_rate_value_and_mask_on_wire():
    original, original_log, rate = wire_command()
    angle_only, angle_log, ignored_rate = wire_command(True)
    assert original['type_mask'] == 0
    assert angle_only['type_mask'] == 2048
    assert original['yaw_rate'] == pytest.approx(-.0875)
    assert angle_only['yaw_rate'] == 0.
    assert angle_only['yaw'] == pytest.approx(math.pi/2-.4)
    assert original_log['sent_yaw_rate'] == pytest.approx(-.0875)
    assert rate == pytest.approx(.0875)
    assert angle_log['sent_yaw_rate'] is None and ignored_rate is None
    for key in original.keys()-{'type_mask', 'yaw_rate'}:
        assert angle_only[key] == original[key]
    assert angle_only['x'] == 25. and angle_only['y'] == 12.


class FrozenBeforeFlight(Exception):
    pass


@pytest.mark.parametrize('angle_only', [False, True])
def test_frozen_config_matches_actual_yaw_rate_policy(tmp_path, angle_only):
    args = ['run_paper_sitl.py', '--config', str(ROOT/'config/experiments/backend_single_rotor_turn.json'),
            '--output', str(tmp_path/'unused'), '--development', '--copter-yaw-rate',
            '--copter-synchronous', '--copter-tangent-yaw']
    if angle_only:
        args.append('--copter-yaw-angle-only')
    with patch('sys.argv', args), patch('scripts.run_paper_sitl.freeze_run', side_effect=FrozenBeforeFlight) as freeze:
        with pytest.raises(FrozenBeforeFlight):
            main()
    cfg = freeze.call_args.args[1]
    assert cfg['copter_yaw_angle_only'] is angle_only
    assert cfg['copter_yaw_rate_feedforward'] is not angle_only
    assert cfg['copter_synchronous_setpoint'] is True
    assert not (tmp_path/'unused').exists()


@pytest.mark.parametrize('flags', [[], ['--development'], ['--copter-tangent-yaw'],
    ['--development', '--copter-tangent-yaw', '--copter-yaw-rate']])
def test_angle_only_invalid_combinations_stop_before_flight(tmp_path, flags):
    args = ['run_paper_sitl.py', '--config', str(ROOT/'config/experiments/backend_single_rotor_turn.json'),
            '--output', str(tmp_path/'unused'), '--copter-yaw-angle-only', *flags]
    with patch('sys.argv', args), patch('scripts.run_paper_sitl.freeze_run') as freeze:
        with pytest.raises(SystemExit) as exc:
            main()
    assert exc.value.code == 2
    freeze.assert_not_called()


def test_gate_rejects_angle_only_evidence_for_other_backend(tmp_path):
    paths = []
    for speed in (12., 14.):
        path = tmp_path/str(speed)
        path.mkdir()
        paths.append(path)
        (path/'acceptance.json').write_text(json.dumps(dict(tracking_qualified=True,
            full_route_completed=True, measured_constraints_passed=False)))
        (path/'config.json').write_text(json.dumps(dict(members=[dict(platform='multirotor')],
            waypoints=[[0, 0], [900, 0]], planner=dict(nominal_speed=speed),
            copter_yaw_rate_feedforward=False, copter_yaw_angle_only=True,
            copter_synchronous_setpoint=True, copter_tangent_yaw=True)))
    cfg = dict(planner=dict(nominal_speed=14.), required_straight_margin=1.)
    members = [MemberSpec(0, 'fixedwing', 0., 12., 24., 2.5, bank_max=.4)]
    kwargs = dict(synchronous=True, tangent_yaw=True)
    with pytest.raises(ValueError, match='mismatch'):
        rotor_development_gate(paths, cfg, members, False, **kwargs)
    result = rotor_development_gate(paths, cfg, members, False, yaw_angle_only=True, **kwargs)
    assert result['sampled_speeds'] == [12., 14.]
    for path in paths:
        source=json.loads((path/'config.json').read_text())
        source['copter_reference_only']=True
        (path/'config.json').write_text(json.dumps(source))
    with pytest.raises(ValueError,match='mismatch'):
        rotor_development_gate(paths,cfg,members,False,yaw_angle_only=True,**kwargs)


def test_correction_setting_is_part_of_mixed_fleet_gate(tmp_path):
    paths=[]
    for speed in (12.,14.):
        path=tmp_path/str(speed);path.mkdir();paths.append(path)
        (path/'acceptance.json').write_text(json.dumps(dict(tracking_qualified=True,
            full_route_completed=True,measured_constraints_passed=True)))
        (path/'config.json').write_text(json.dumps(dict(members=[dict(platform='multirotor')],
            waypoints=[[0,0],[900,0]],planner=dict(nominal_speed=speed),
            copter_yaw_rate_feedforward=True,copter_consistent_correction=True)))
    cfg=dict(planner=dict(nominal_speed=14.),required_straight_margin=1.)
    members=[MemberSpec(0,'fixedwing',0.,12.,24.,2.5,bank_max=.4)]
    with pytest.raises(ValueError,match='mismatch'):
        rotor_development_gate(paths,cfg,members,True)
    assert rotor_development_gate(paths,cfg,members,True,consistent_correction=True)['sampled_speeds']==[12.,14.]


def test_direct_reference_produces_consistent_turn_kinematics():
    target, velocity, acceleration, yaw_rate_ned = reference_copter_setpoint(
        (10., 20., math.pi/2, 14., 1/160., .2))
    assert target == pytest.approx((10., 20., math.pi/2))
    assert velocity == pytest.approx((0., 14.))
    assert acceleration == pytest.approx((-14*14/160., .2))
    assert yaw_rate_ned == pytest.approx(-14/160.)


def test_direct_reference_freezes_as_no_nmpc(tmp_path):
    args = ['run_paper_sitl.py', '--config', str(ROOT/'config/experiments/backend_single_rotor_turn.json'),
        '--output', str(tmp_path/'unused'), '--development', '--copter-yaw-rate',
        '--copter-synchronous', '--copter-analytic-gradient', '--copter-tangent-yaw',
        '--copter-reference-only']
    with patch('sys.argv', args), patch('scripts.run_paper_sitl.freeze_run', side_effect=FrozenBeforeFlight) as freeze:
        with pytest.raises(FrozenBeforeFlight):
            main()
    cfg = freeze.call_args.args[1]
    assert cfg['copter_reference_only'] is True
    assert 'no NMPC' in cfg['execution_backend']
