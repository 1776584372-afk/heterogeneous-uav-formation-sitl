#!/usr/bin/env python3
"""SITL speed-ramp calibration on a fixed-curvature path."""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time

from scripts.calibrate_sitl import quintic_speed
from uav_formation.capability_model import MemberSpec
from uav_formation.paper_tracking import PlaneCourseTracker
from uav_formation.sitl_vehicle import AP, SitlVehicle


class TimingEvidenceError(RuntimeError):
    """The host missed the preregistered command-update interval."""


def advance_fixed_curvature(east, north, course, speed, signed_curvature, dt):
    if not all(math.isfinite(value) for value in
               (east, north, course, speed, signed_curvature, dt)):
        raise ValueError('finite kinematics required')
    if speed <= 0 or dt < 0 or abs(signed_curvature) < 1e-9:
        raise ValueError('positive speed, nonnegative dt, and nonzero curvature required')
    next_course = course + signed_curvature * speed * dt
    east += (math.cos(course) - math.cos(next_course)) / signed_curvature
    north += (math.sin(next_course) - math.sin(course)) / signed_curvature
    return east, north, next_course


def segment_schedule(speeds, rate, settle_s=30., hold_s=10.):
    if len(speeds) != 3 or speeds[0] != speeds[2] or speeds[1] == speeds[0]:
        raise ValueError('speeds must be start, peak, start')
    duration = 1.875 * abs(speeds[1] - speeds[0]) / rate
    return [
        ('settle', settle_s, speeds[0], speeds[0]),
        ('accel', duration, speeds[0], speeds[1]),
        ('high', hold_s, speeds[1], speeds[1]),
        ('decel', duration, speeds[1], speeds[2]),
        ('final', hold_s, speeds[2], speeds[2]),
    ]


def scheduled_speed(segments, elapsed, rate):
    offset = 0.
    for name, duration, start, end in segments:
        if elapsed < offset + duration or name == segments[-1][0]:
            local = max(0., min(duration, elapsed - offset))
            if start == end:
                return name, start, 0.
            speed, acceleration = quintic_speed(start, end, local, rate)
            return name, speed, acceleration
        offset += duration
    raise AssertionError('unreachable schedule')


def phase_name(stage, speed0, bank, sign):
    return 'turn_coupled_%s_%g_%g_%d' % (stage, speed0, bank, sign)


def retransmit_reference(vehicle, transmission):
    if transmission is None:
        return False
    if transmission[0] == 'plane':
        command = transmission[1]
        vehicle.plane_reference(
            command.airspeed, command.bearing_deg, command.lateral_accel_limit,
            lateral_mode='course', altitude=command.altitude)
        vehicle.check_plane_commands()
    else:
        values = transmission[1]
        vehicle.copter_reference(*values[:6], yaw_rate=values[6])
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle', choices=['plane', 'copter'], required=True)
    parser.add_argument('--speeds', required=True)
    parser.add_argument('--ramp-rate', type=float, required=True)
    parser.add_argument('--bank', type=float, required=True)
    parser.add_argument('--turn-signs', default='left,right')
    parser.add_argument('--lookahead', type=float, default=40.)
    parser.add_argument('--guidance-mode', choices=['preview', 'frenet', 'curvature_ff'], default='preview')
    parser.add_argument('--cross-track-gain', type=float, default=1.)
    parser.add_argument('--course-to-bank-gain', type=float, default=math.radians(50.),
                        help='ArduPlane GUIDED course-error to bank gain, rad/rad')
    parser.add_argument('--output', required=True)
    parser.add_argument('--port', type=int, default=39900)
    parser.add_argument('--speedup', type=float, default=4.)
    parser.add_argument('--maximum-command-gap', type=float)
    parser.add_argument('--dataset-group', choices=['PILOT', 'A', 'B'], default='PILOT')
    parser.add_argument('--repeat-id', type=int, default=0)
    parser.add_argument('--protocol-id', default='v2-coupled-turn')
    args = parser.parse_args()
    speeds = [float(value) for value in args.speeds.split(',')]
    sign_names = [value.strip() for value in args.turn_signs.split(',') if value.strip()]
    if (len(speeds) != 3 or speeds[0] != speeds[2] or speeds[1] <= speeds[0]
            or args.ramp_rate <= 0 or not 0 < args.bank <= 20
            or not math.isfinite(args.cross_track_gain) or args.cross_track_gain <= 0
            or not math.isfinite(args.course_to_bank_gain) or args.course_to_bank_gain <= 0
            or (args.maximum_command_gap is not None
                and (not math.isfinite(args.maximum_command_gap)
                     or args.maximum_command_gap <= 0))
            or sign_names not in (['left'], ['right'], ['left', 'right'])):
        parser.error('invalid coupled-turn work point')
    # Compass course increases clockwise: negative curvature is a left turn.
    signs = [-1 if value == 'left' else 1 for value in sign_names]
    segments = segment_schedule(speeds, args.ramp_rate)
    total_duration = sum(segment[1] for segment in segments)
    curvature = 9.80665 * math.tan(math.radians(args.bank)) / speeds[0] ** 2
    maximum_lateral_accel = speeds[1] ** 2 * curvature
    if maximum_lateral_accel > 2.5 + 1e-9:
        parser.error('coupled point exceeds frozen lateral-acceleration work domain')
    model = 'plane' if args.vehicle == 'plane' else 'x:freestyle.json'
    defaults = None if args.vehicle == 'plane' else AP / 'Tools/autotest/models/freestyle.parm'
    vehicle = SitlVehicle(args.vehicle, model, args.output, args.port,
                          speedup=args.speedup, defaults=defaults)
    protocol = {
        'protocol_version': 1, 'kind': 'fixed_curvature_speed_ramp',
        'platform': args.vehicle, 'dataset_group': args.dataset_group,
        'repeat_id': args.repeat_id, 'protocol_id': args.protocol_id,
        'altitude_m': 60., 'wind_m_s': 0., 'speeds': speeds,
        'reference_tangential_acceleration_m_s2': args.ramp_rate,
        'equivalent_bank_at_start_speed_deg': args.bank,
        'fixed_curvature_m_inv': curvature, 'turn_signs': sign_names,
        'maximum_reference_lateral_acceleration_m_s2': maximum_lateral_accel,
        'segments': [{'name': name, 'duration_s': duration, 'start_speed': start,
                      'end_speed': end} for name, duration, start, end in segments],
        'speedup': args.speedup, 'lookahead_m': args.lookahead,
        'maximum_command_gap_s': args.maximum_command_gap,
        'guidance_mode': args.guidance_mode, 'cross_track_gain': args.cross_track_gain,
        'course_to_bank_gain_rad_per_rad': args.course_to_bank_gain,
        'qualification_phases': ['turn_coupled_accel_*', 'turn_coupled_high_*',
                                 'turn_coupled_decel_*', 'turn_coupled_final_*'],
        'excluded_phases': ['takeoff', 'path_entry_*', 'turn_coupled_settle_*', 'exit'],
        'tracking_gates': {'speed_error_m_s': 1., 'altitude_error_m': 2.,
                           'course_error_deg': 5., 'position_error_m': 4.,
                           'bank_abs_deg': 27.},
        'scope': 'SITL coupled work-domain evidence; not a real-aircraft envelope',
    }
    (vehicle.directory / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    report = {'status': 'FAILED', 'protocol': protocol, 'points': []}
    try:
        vehicle.start(); vehicle.takeoff()
        print('AIRBORNE', args.vehicle, flush=True)
        for turn_name, sign in zip(sign_names, signs):
            if args.vehicle == 'plane':
                vehicle.set_phase('path_entry_%g_%g_%d' % (speeds[0], args.bank, sign))
                row = vehicle.row(); entry_start = row['sim_s']; settled = None
                entry_course = math.atan2(row['ve'], row['vn'])
                while True:
                    vehicle.plane_reference(speeds[0], math.degrees(entry_course), 2.5,
                                            lateral_mode='course')
                    vehicle.pump(.05 / vehicle.speedup); vehicle.check_plane_commands()
                    row = vehicle.row(); elapsed = row['sim_s'] - entry_start
                    error = math.atan2(math.sin(math.atan2(row['ve'], row['vn']) - entry_course),
                                       math.cos(math.atan2(row['ve'], row['vn']) - entry_course))
                    valid = (abs(row['airspeed'] - speeds[0]) < .3
                             and abs(row['alt'] - 60.) < 2.
                             and abs(error) < math.radians(3.))
                    if valid and settled is None:
                        settled = row['sim_s']
                    if not valid:
                        settled = None
                    if elapsed >= 25. and settled is not None and row['sim_s'] - settled >= 5.:
                        break
                    if elapsed > 90.:
                        raise RuntimeError('path entry did not settle')
            row = vehicle.row(); begin = previous = row['sim_s']
            previous_command_wall = None
            course = math.atan2(row['ve'], row['vn'])
            east, north = row['east'], row['north']
            first = len(vehicle.samples); current_stage = None
            aligned_metrics = []
            last_transmission = None
            tracker = None
            if args.vehicle == 'plane':
                tracker = PlaneCourseTracker(
                    MemberSpec(0, 'fixedwing', 0., 12., 24., 2.5,
                               bank_max=math.radians(27), a_t_min=-.2, a_t_max=.2),
                    lookahead=args.lookahead, progress_gain=.05,
                    guidance_mode=args.guidance_mode,
                    cross_track_gain=args.cross_track_gain,
                    course_to_bank_gain=args.course_to_bank_gain,
                    initial_command_airspeed=vehicle.target.get('ref_speed'))
            while vehicle.row()['sim_s'] - begin < total_duration:
                loop_row = vehicle.row()
                loop_wall = time.monotonic()
                now = loop_row['sim_s']; dt = max(0., now - previous); previous = now
                elapsed = now - begin
                stage, speed, acceleration = scheduled_speed(segments, elapsed, args.ramp_rate)
                if stage != current_stage:
                    vehicle.set_phase(phase_name(stage, speeds[0], args.bank, sign))
                    current_stage = stage
                if (args.maximum_command_gap is not None
                        and previous_command_wall is not None
                        and loop_wall - previous_command_wall > args.maximum_command_gap):
                    raise TimingEvidenceError(
                        'command wall-clock gap %.6f s exceeds %.6f s at %.6f s' %
                        (loop_wall - previous_command_wall, args.maximum_command_gap, now))
                if dt <= 0:
                    if retransmit_reference(vehicle, last_transmission):
                        previous_command_wall = time.monotonic()
                    vehicle.pump(.05 / vehicle.speedup)
                    continue
                if args.maximum_command_gap is not None and dt > args.maximum_command_gap:
                    raise TimingEvidenceError(
                        'command source-clock gap %.6f s exceeds %.6f s at %.6f s' %
                        (dt, args.maximum_command_gap, now))
                signed_curvature = sign * curvature
                east, north, course = advance_fixed_curvature(
                    east, north, course, speed, signed_curvature, dt)
                omega = signed_curvature * speed
                if stage != 'settle':
                    actual_speed = (loop_row['airspeed'] if args.vehicle == 'plane'
                                    else math.hypot(loop_row['vn'], loop_row['ve']))
                    actual_course = math.atan2(loop_row['ve'], loop_row['vn'])
                    aligned_metrics.append({
                        'boot_s': now,
                        'stage': stage,
                        'actual_speed_m_s': actual_speed,
                        'speed_error_m_s': abs(actual_speed - speed),
                        'altitude_error_m': abs(loop_row['alt'] - 60.),
                        'course_error_deg': abs(math.degrees(math.atan2(
                            math.sin(actual_course - course),
                            math.cos(actual_course - course)))),
                        'position_error_m': math.hypot(loop_row['east'] - east,
                                                       loop_row['north'] - north),
                        'bank_abs_deg': abs(math.degrees(loop_row['roll'])),
                    })
                if args.vehicle == 'plane':
                    row = loop_row
                    command = tracker.update(
                        (east, north, math.pi / 2 - course, speed, -signed_curvature, 60.),
                        (row['east'], row['north']), row['airspeed'], dt)
                    vehicle.plane_reference(command.airspeed, command.bearing_deg,
                                            command.lateral_accel_limit,
                                            lateral_mode='course', altitude=command.altitude)
                    last_transmission = ('plane', command)
                    vehicle.target.update(ref_east=east, ref_north=north, ref_accel=acceleration)
                    vehicle.check_plane_commands()
                else:
                    ve, vn = speed * math.sin(course), speed * math.cos(course)
                    ae = acceleration * math.sin(course) + speed * omega * math.cos(course)
                    an = acceleration * math.cos(course) - speed * omega * math.sin(course)
                    vehicle.copter_reference(east, north, ve, vn, course,
                                             accel=(ae, an), yaw_rate=omega)
                    last_transmission = (
                        'copter', (east, north, ve, vn, course, (ae, an), omega))
                    vehicle.target['ref_accel'] = acceleration
                vehicle.target['ref_path_course'] = math.degrees(course)
                previous_command_wall = time.monotonic()
                vehicle.pump(.05 / vehicle.speedup)
                row = vehicle.row()
                if (row['mode'] != 'GUIDED' or not row['armed'] or row['alt'] < 30.
                        or abs(math.degrees(row['roll'])) > 65.
                        or max(row['attitude_age'], row['velocity_age'], row['airspeed_age']) > .8):
                    raise RuntimeError('flight protection or stale telemetry')
            prefixes = ('turn_coupled_accel_', 'turn_coupled_high_',
                        'turn_coupled_decel_', 'turn_coupled_final_')
            rows = [row for row in vehicle.samples[first:]
                    if row['phase'].startswith(prefixes)]
            if len(rows) < 20:
                raise RuntimeError('insufficient coupled qualification telemetry')
            if len(aligned_metrics) < 20:
                raise RuntimeError('insufficient source-clock-aligned qualification samples')
            errors = {
                name: max(sample[name] for sample in aligned_metrics)
                for name in protocol['tracking_gates']
            }
            failures = [name for name, value in errors.items()
                        if value > protocol['tracking_gates'][name]]
            point = {'turn': turn_name, 'sign': sign, 'errors': errors, 'failures': failures,
                     'status': 'FAIL' if failures else 'PASS',
                     'mean_actual_speed_m_s': statistics.mean(
                         sample['actual_speed_m_s'] for sample in aligned_metrics),
                     'metric_alignment': 'reference and measured state at identical vehicle boot time',
                     'aligned_samples': len(aligned_metrics),
                     'window': [rows[0]['sim_s'], rows[-1]['sim_s']]}
            report['points'].append(point)
            (vehicle.directory / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(point), flush=True)
        report['status'] = 'COMPLETED'
        report['all_tested_points_passed'] = (
            len(report['points']) == len(signs)
            and all(point['status'] == 'PASS' for point in report['points']))
        report['envelope_certified'] = False
    except Exception as exc:
        report['error'] = repr(exc)
        report['failure_classification'] = (
            'INVALID_EVIDENCE' if isinstance(exc, TimingEvidenceError) else 'RUN_FAILED')
        print('FAILED', repr(exc), flush=True)
    finally:
        try:
            if vehicle.link:
                vehicle.finish(); report['exit_completed'] = True
        except Exception as exc:
            report['exit_completed'] = False; report['exit_error'] = repr(exc)
        vehicle.close()
        (vehicle.directory / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report.get('all_tested_points_passed') and report.get('exit_completed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
