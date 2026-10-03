"""Validated experiment input shared by offline planning and ROS execution."""
from pathlib import Path
from dataclasses import replace
import hashlib
import json
import math
import yaml
from .capability_model import G, MemberSpec
from .reference_coordinator import ReferenceCoordinator


ROOT = Path(__file__).resolve().parents[1]


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _contract_path(entry):
    path = Path(entry['path'])
    return path if path.is_absolute() else ROOT / path


def validate_frozen_contract(cfg, members):
    """Fail closed when a formal task no longer matches its frozen v3 evidence."""
    contract = cfg.get('frozen_contract')
    if contract is None:
        return
    required = {'capability', 'measurement', 'acceptance', 'validation', 'formal_protocol'}
    if not isinstance(contract, dict) or set(contract) != required:
        raise ValueError('frozen_contract must name capability, measurement, acceptance, validation and formal_protocol')
    paths = {}
    for name in sorted(required):
        entry = contract[name]
        if not isinstance(entry, dict) or set(entry) != {'path', 'sha256'}:
            raise ValueError('invalid frozen contract entry: ' + name)
        path = _contract_path(entry)
        if not path.is_file() or _sha256(path) != entry['sha256']:
            raise ValueError('frozen contract hash mismatch: ' + name)
        paths[name] = path
    capability = yaml.safe_load(paths['capability'].read_text())
    validation = json.loads(paths['validation'].read_text())
    if (capability.get('status') != 'FROZEN'
            or not capability.get('formal_experiments_unlocked', False)):
        raise ValueError('capability contract has not unlocked formal experiments')
    if (not validation.get('validation_passed', False)
            or any(validation.get('valid_counts', {}).get(p, 0) < 5
                   for p in ('fixedwing', 'multicopter'))):
        raise ValueError('independent validation contract did not pass')
    frozen = capability['unchanged_candidate']
    if cfg.get('wind_m_s') != 0 or cfg.get('altitude') != frozen['altitude_m']:
        raise ValueError('formal task environment differs from frozen v3 work domain')
    speed_lo, speed_hi = frozen['common_speed_interval_m_s']
    bank = math.radians(frozen['equivalent_bank_max_deg'])
    lateral = G * math.tan(bank)
    for member in members:
        expected_at = (frozen['multicopter_reference_tangential_acceleration_m_s2']
                       if member.platform == 'multirotor'
                       else frozen['fixedwing_reference_tangential_acceleration_m_s2'])
        if (member.v_min != speed_lo or member.v_max != speed_hi
                or member.a_t_min != expected_at[0] or member.a_t_max != expected_at[1]
                or member.a_lat_max > lateral + 1e-6):
            raise ValueError('member %d exceeds frozen v3 planning limits' % member.member_id)
        if member.platform == 'fixedwing':
            minimum_radius = capability['fixed_wing']['planning_limit']['minimum_reference_turn_radius_m']
            if (member.bank_max != bank or member.turn_radius_min is None
                    or member.turn_radius_min < minimum_radius):
                raise ValueError('fixed-wing member differs from frozen v3 turn limits')
    planner = cfg['planner']
    for name in ('nominal_speed', 'start_speed', 'end_speed'):
        value = planner.get(name)
        if value is not None and not speed_lo <= value <= speed_hi:
            raise ValueError('%s lies outside frozen common speed interval' % name)
    if any(member.platform == 'fixedwing' for member in members):
        expected = capability['fixed_wing']['planning_limit']['guidance']
        actual = cfg.get('fixedwing_guidance', {})
        for key in ('mode', 'lookahead_m', 'cross_track_gain',
                    'course_to_bank_gain_rad_per_rad'):
            config_key = {'mode': 'guidance_mode', 'lookahead_m': 'lookahead',
                          'course_to_bank_gain_rad_per_rad': 'course_to_bank_gain'}.get(key, key)
            if actual.get(config_key) != expected[key]:
                raise ValueError('fixed-wing guidance differs from frozen v3: ' + config_key)
        if actual.get('progress_gain') != .05:
            raise ValueError('fixed-wing progress gain differs from v3 A/B controller')
        controller = cfg.get('fixedwing_controller_limits', {})
        expected_controller = {
            'v_min': 12., 'v_max': 24., 'a_lat_max': 2.5,
            'bank_max_deg': 27., 'a_t_min': -.2, 'a_t_max': .2,
            'source': 'v3_A_B_calibration_controller_member',
        }
        if controller != expected_controller:
            raise ValueError('fixed-wing controller limits differ from v3 A/B controller')
    if any(member.platform == 'multirotor' for member in members):
        expected_copter = {
            'mode': 'measured_feedback_nmpc_consistent_correction',
            'tangent_yaw': True,
            'yaw_rate_feedforward': True,
            'synchronous_setpoint': True,
            'analytic_gradient': True,
            'correction': {'max_offset_m': 1., 'max_rate_m_s': .2,
                           'max_accel_m_s2': .1, 'response_s_inv': .5},
        }
        if cfg.get('multicopter_execution') != expected_copter:
            raise ValueError('multicopter execution differs from preregistered v3 formal controller')


def fixedwing_controller_member(cfg, member):
    """Return tracking-controller authority separately from planning limits."""
    limits = cfg.get('fixedwing_controller_limits')
    if limits is None:
        return member
    if member.platform != 'fixedwing':
        raise ValueError('fixed-wing controller limits require a fixed-wing member')
    return replace(member, v_min=limits['v_min'], v_max=limits['v_max'],
                   a_lat_max=limits['a_lat_max'],
                   bank_max=math.radians(limits['bank_max_deg']),
                   a_t_min=limits['a_t_min'], a_t_max=limits['a_t_max'])


def load_config(path):
    with Path(path).open(encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or cfg.get('schema_version') != 1:
        raise ValueError('schema_version: 1 required')
    if cfg.get('wind_m_s', 0) != 0:
        raise ValueError('current reference planner supports no-wind experiments only')
    members = []
    for raw in cfg['members']:
        raw = dict(raw)
        if raw.pop('r_x', 0) != 0 or raw.pop('r_z', 0) != 0:
            raise ValueError('research mode requires r_x=r_z=0')
        members.append(MemberSpec(**raw))
    if [m.member_id for m in members] != list(range(len(members))):
        raise ValueError('execution requires consecutive ordered IDs starting at zero')
    if len({m.r_y for m in members}) != len(members):
        raise ValueError('members must occupy distinct lateral slots')
    pts = cfg['waypoints']
    if len(pts) < 2 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in pts):
        raise ValueError('finite planar waypoints required')
    if cfg['endpoint'] not in ('hover', 'flythrough'):
        raise ValueError('endpoint must be hover or flythrough')
    if any(m.platform == 'fixedwing' for m in members) and cfg['endpoint'] != 'flythrough':
        raise ValueError('fixedwing cannot use hover endpoint')
    if not math.isfinite(cfg['altitude']) or cfg['altitude'] <= 0:
        raise ValueError('positive altitude required')
    validate_frozen_contract(cfg, members)
    return cfg, tuple(members)


def planning_members(cfg, members):
    """Reserve tracking authority without changing frozen physical capabilities."""
    factors=cfg.get('reference_reserve', {})
    allowed={'tangential_fraction','lateral_fraction'}
    if not isinstance(factors,dict) or set(factors)-allowed:
        raise ValueError('unknown reference reserve setting')
    for value in factors.values():
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or not 0<value<=1:
            raise ValueError('reference reserve fractions must lie in (0,1]')
    at=factors.get('tangential_fraction',1.)
    lat=factors.get('lateral_fraction',1.)
    return tuple(replace(m,a_t_min=m.a_t_min*at,a_t_max=m.a_t_max*at,
                         a_lat_max=m.a_lat_max*lat) for m in members)


def coordinator_from_config(cfg, members, nominal_speed=None, nominal_only=False):
    planner = dict(cfg['planner'])
    if nominal_only:
        planner['radii'] = [planner['nominal_radius']]
    if nominal_speed is not None:
        planner['nominal_speed'] = nominal_speed
    planner.setdefault('start_speed', 0. if cfg['endpoint'] == 'hover' else None)
    planner.setdefault('end_speed', 0. if cfg['endpoint'] == 'hover' else None)
    return ReferenceCoordinator(planning_members(cfg,members), **planner)
