#!/usr/bin/env python3
"""Bounded, evidence-preserving selection preview using the existing S1 chain.

This is not a full capability calibration. No aircraft parameters are relaxed.
Use the experiment_env.sh wrapper. A fresh output directory is mandatory.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shlex
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
AP = Path.home() / 'ardupilot'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def command(directory, args, filename, cwd=ROOT):
    with (directory / 'commands.txt').open('a') as stream:
        stream.write('cwd=' + str(cwd) + '\n' + shlex.join(map(str, args)) + '\n')
    with (directory / filename).open('w') as stream:
        result = subprocess.run(list(map(str, args)), cwd=cwd, stdout=stream,
                                stderr=subprocess.STDOUT)
    return result.returncode


def initialize(directory):
    directory.mkdir(parents=True, exist_ok=False)
    commands = [
        ('git_status.txt', ['git', 'status', '--short']),
        ('source_revision.txt', ['git', 'rev-parse', 'HEAD']),
        ('working_changes.patch', ['git', 'diff', '--', 'uav_formation', 'scripts', 'config']),
        ('files.txt', ['rg', '--files', 'uav_formation', 'scripts', 'config', 'tests']),
        ('reference_links.txt', ['rg', '-n', 'Illustrative|v_min|v_max|bank_max|a_lat_max|fixedwing_reference|COMMAND_INT', 'config', 'scripts', 'uav_formation']),
        ('ardupilot_revision.txt', ['git', '-C', str(AP), 'rev-parse', 'HEAD']),
        ('ardupilot_status.txt', ['git', '-C', str(AP), 'status', '--short']),
        ('model_files.txt', ['rg', '--files', str(AP / 'Tools/autotest/models')]),
    ]
    for filename, args in commands:
        command(directory, args, filename)
    snapshots = directory / 'source_snapshot'
    snapshots.mkdir()
    sources = list((ROOT / 'uav_formation').glob('*.py'))
    sources += list((ROOT / 'scripts').glob('*.py'))
    sources += list((ROOT / 'config/experiments').glob('*'))
    sources += [ROOT / 'config/copter_battery.parm', ROOT / 'scripts/experiment_env.sh']
    for name in ('SIM_Frame.cpp', 'SIM_Multicopter.cpp', 'SIM_Motor.cpp', 'SIM_Plane.cpp', 'SIM_Plane.h'):
        sources.append(AP / 'libraries/SITL' / name)
    sources += [AP / 'Tools/autotest/models/plane.parm', AP / 'Tools/autotest/models/freestyle.json',
                AP / 'Tools/autotest/models/freestyle.parm']
    hashes = {}
    for source in sources:
        relative = source.relative_to(ROOT) if source.is_relative_to(ROOT) else Path('ardupilot') / source.relative_to(AP)
        target = snapshots / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        hashes[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
    for name in ('arducopter', 'arduplane'):
        source = AP / 'build/sitl/bin' / name
        hashes[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
    save(directory / 'source_sha256.json', hashes)
    protocol = dict(
        kind='single_vehicle_selection_preview_not_formal_calibration',
        created_unix=time.time(), model='+', mode='GUIDED', altitude_m=10., wind_m_s=0.,
        speed_steps_m_s=[2., 4., 6.], unchanged_speed_cap_m_s=6.,
        control_chain='existing DroneMPC -> PositionCommand -> PoseStamped bridge -> MAVROS -> ArduCopter',
        steady_window='first 12 s after 10 s continuously at nominal reference speed',
        nominal_tolerance_m_s=.02, settle_s=10., window_s=12., min_window_s=11.8,
        max_gap_s=.3, max_speed_error_m_s=.5, max_cross_track_m=1.,
        max_planar_tracking_error_m=8., max_altitude_error_m=1., max_yaw_error_deg=5.,
        max_abs_roll_deg=15., speed_uncertainty_m_s=.3, required_team_width_m_s=1.,
        saturation='not scored from derived CSV; raw actuator telemetry retained in bag/BIN, complete calibration pending',
        failure_policy='stop higher speed steps after first failed preview; preserve all runs',
        limits='no extrapolation beyond tested speeds; preview is not a physical maximum or mixed-flight approval')
    save(directory / 'calibration_protocol.json', protocol)
    return protocol


def analyze(run, speed, protocol):
    status_path = run / 'flight_status.json'
    status = json.loads(status_path.read_text()) if status_path.exists() else {}
    result = dict(command_speed_m_s=speed, run=str(run), status='FAIL', reasons=[],
                  flight_completed=status.get('completed', False), landed=status.get('landed', False))
    if not status.get('completed') or not status.get('landed'):
        result['reasons'].append('flight lifecycle incomplete')
    if not (run / 'telemetry.csv').exists():
        result['reasons'].append('no telemetry')
        return result
    with (run / 'telemetry.csv').open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['phase'] == 'track']
    numeric = ('t', 'ref_speed', 'vx', 'vy', 'yaw', 'ref_yaw', 'x', 'y', 'ref_x', 'ref_y',
               'z', 'roll', 'pose_age', 'velocity_age', 'state_age')
    if any(not math.isfinite(float(row[name])) for row in rows for name in numeric):
        result['reasons'].append('nonfinite telemetry')
        return result
    segment = []
    chosen = []
    for row in rows:
        if abs(float(row['ref_speed']) - speed) > protocol['nominal_tolerance_m_s']:
            segment = []
            continue
        if segment and float(row['t']) - float(segment[-1]['t']) > protocol['max_gap_s']:
            segment = []
        segment.append(row)
        elapsed = float(row['t']) - float(segment[0]['t'])
        if elapsed >= protocol['settle_s']:
            chosen.append(row)
        if elapsed >= protocol['settle_s'] + protocol['window_s']:
            break
        # A discontinuity invalidates the window, rather than joining disconnected data.
        if len(segment) == 1:
            chosen = []
    if len(chosen) < 2 or float(chosen[-1]['t']) - float(chosen[0]['t']) < protocol['min_window_s']:
        result['reasons'].append('insufficient contiguous steady reference window')
        return result
    def values(name):
        return [float(r[name]) for r in chosen]
    forward = [float(r['vx']) * math.cos(float(r['yaw'])) + float(r['vy']) * math.sin(float(r['yaw'])) for r in chosen]
    left = [-float(r['vx']) * math.sin(float(r['yaw'])) + float(r['vy']) * math.cos(float(r['yaw'])) for r in chosen]
    metrics = dict(
        max_speed_error_m_s=max(abs(v - speed) for v in forward),
        max_cross_track_m=max(abs(float(r['y']) - float(r['ref_y'])) for r in chosen),
        max_planar_tracking_error_m=max(math.hypot(float(r['x']) - float(r['ref_x']), float(r['y']) - float(r['ref_y'])) for r in chosen),
        max_altitude_error_m=max(abs(z - protocol['altitude_m']) for z in values('z')),
        max_yaw_error_deg=max(abs(math.degrees(math.atan2(math.sin(float(r['yaw'])-float(r['ref_yaw'])), math.cos(float(r['yaw'])-float(r['ref_yaw']))))) for r in chosen),
        max_abs_roll_deg=max(abs(math.degrees(v)) for v in values('roll')))
    for name, value in metrics.items():
        if value > protocol[name]:
            result['reasons'].append(name + ' exceeds frozen threshold')
    if any(r['mode'] != 'GUIDED' or r['armed'] != 'True' for r in chosen):
        result['reasons'].append('not continuously armed GUIDED')
    if any(float(r['pose_age']) > .5 or float(r['velocity_age']) > .5 or float(r['state_age']) > 3 for r in chosen):
        result['reasons'].append('stale telemetry')
    result.update(metrics=metrics, window=[float(chosen[0]['t']), float(chosen[-1]['t'])],
                  mean_forward_m_s=statistics.mean(forward), min_forward_m_s=min(forward),
                  max_forward_m_s=max(forward), max_abs_left_m_s=max(map(abs, left)),
                  conservative_observed_forward_m_s=max(0., min(forward)-protocol['speed_uncertainty_m_s']))
    result['status'] = 'FAIL' if result['reasons'] else 'PASS'
    with (run / 'steady_window.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(chosen[0])+['body_forward','body_left'])
        writer.writeheader()
        for row, vf, vl in zip(chosen, forward, left):
            writer.writerow(dict(row, body_forward=vf, body_left=vl))
    return result


def run_steps(directory, protocol):
    import yaml
    base = yaml.safe_load((ROOT / 'config/experiments/s1.yaml').read_text())
    prior = directory / 'copter_preview.json'
    results = json.loads(prior.read_text()) if prior.exists() else []
    if results and results[-1]['status'] != 'PASS':
        return results
    for index, speed in enumerate(protocol['speed_steps_m_s']):
        if any(r['command_speed_m_s'] == speed for r in results):
            continue
        cfg = json.loads(json.dumps(base))
        length = speed * 38 + speed * speed / 1.5
        cfg.update(name='selection_single_copter_%g' % speed, altitude=protocol['altitude_m'],
                   waypoints=[[0., 0.], [length, 0.]], members=[dict(base['members'][0], r_y=0.)])
        cfg['planner'].update(nominal_speed=speed, radii=[20.], bounds=[-20., -20., length+20., 20.])
        cfg['selection_protocol'] = str(directory / 'calibration_protocol.json')
        config = directory / ('copter_%g_input.json' % speed)
        if config.exists():
            if json.loads(config.read_text()) != cfg:
                raise ValueError('frozen input changed')
        else:
            save(config, cfg)
        label = 'copter_%g' % speed
        attempt = 1
        while (directory / (label+'_console.log')).exists():
            attempt += 1
            label = 'copter_%g_attempt%d' % (speed, attempt)
        run = directory / label
        args = ['bash', 'scripts/experiment_env.sh', 'python3', 'scripts/run_experiment.py',
                '--config', str(config), '--output', str(run), '--domain', '81',
                '--base-port', str(18660+index*30+(attempt-1)*100), '--timeout', '240']
        print('Starting single-copter step', speed, 'm/s', flush=True)
        code = command(directory, args, label+'_console.log')
        result = analyze(run, speed, protocol)
        result['returncode'] = code
        if code != 0:
            result['status'] = 'FAIL'
            result['reasons'].append('runner nonzero exit')
        save(run / 'precheck_result.json' if run.exists() else directory / (label+'_startup_failure.json'), result)
        results.append(result)
        save(directory / 'copter_preview.json', results)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if result['status'] != 'PASS':
            break
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--resume', action='store_true', help='keep frozen protocol and all prior attempts')
    args = parser.parse_args()
    directory = Path(args.output).resolve()
    protocol = json.loads((directory/'calibration_protocol.json').read_text()) if args.resume else initialize(directory)
    with (directory / 'commands.txt').open('a') as stream:
        stream.write('entrypoint: ' + shlex.join(sys.argv) + '\n')
    test_name = 'unit_tests_%d.log' % time.time_ns() if args.resume else 'unit_tests.log'
    code = command(directory, ['bash', 'scripts/experiment_env.sh', 'python3', '-m', 'unittest',
                               'discover', '-s', 'tests', '-v'], test_name)
    if code:
        raise RuntimeError('tests failed; SITL not started')
    results = run_steps(directory, protocol)
    save(directory / 'preview_state.json', dict(copter=results, fixedwing='single_flight_not_ready',
         straight_gate='UNKNOWN', reason='no independent fixedwing flight envelope; do not substitute parameter values'))


if __name__ == '__main__':
    main()
