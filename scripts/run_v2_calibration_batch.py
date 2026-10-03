#!/usr/bin/env python3
"""Run a resumable, preregistered v2 single-vehicle calibration batch."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_plan(path, group):
    plan = yaml.safe_load(Path(path).read_text())
    key = 'group_' + group
    if plan.get('status') != 'PRECALIBRATION_FROZEN' or key not in plan:
        raise ValueError('plan is not a frozen preregistered group')
    return plan, plan[key]


def require_b_freeze(capability_path=ROOT / 'config/v2/capability_v2.yaml',
                     acceptance_path=ROOT / 'config/v2/acceptance_protocol_v2.yaml'):
    capability = yaml.safe_load(Path(capability_path).read_text())
    acceptance = yaml.safe_load(Path(acceptance_path).read_text())
    if capability.get('status') != 'FROZEN' or not capability.get('freeze_allowed'):
        raise RuntimeError('B group is locked until capability_v2 is frozen')
    if acceptance.get('status') != 'FROZEN':
        raise RuntimeError('B group is locked until acceptance_protocol_v2 is frozen')


def expand(plan, group_data, group, only=None):
    repeats = int(plan['common'][group + '_repeats'])
    common_job = {
        'approach_rate': float(plan['common'].get('approach_rate_m_s2', 1.0)),
        'approach_settle': float(plan['common'].get('approach_settle_sim_s', 10.0)),
        'maximum_command_gap': plan['common'].get('maximum_command_gap_s'),
        'fixedwing_guidance': dict(plan['common'].get('fixedwing_guidance', {})),
    }
    jobs = []
    for condition in group_data.get('straight_ramps', []):
        if only and condition['id'] not in only:
            continue
        for repeat in range(repeats):
            jobs.append(dict(common_job, kind='straight_ramp', repeat=repeat, **condition))
    for condition in group_data.get('steady_turns', []):
        if only and condition['id'] not in only:
            continue
        for repeat in range(repeats):
            jobs.append(dict(common_job, kind='steady_turn', repeat=repeat, **condition))
    for condition in group_data.get('coupled_turns', []):
        if only and condition['id'] not in only:
            continue
        for repeat in range(repeats):
            jobs.append(dict(common_job, kind='coupled_turn', repeat=repeat, **condition))
    return jobs


def command_for(job, group, output, port, speedup):
    python = sys.executable
    if job['kind'] == 'straight_ramp':
        command = [python, str(ROOT / 'scripts/calibrate_sitl.py'), '--vehicle',
            'copter' if job['platform'] == 'multicopter' else 'plane',
            '--speeds', ','.join(str(value) for value in job['speeds']),
            '--ramp-rate', str(job['rate']), '--ramp-shape', 'quintic',
            '--approach-rate', str(job.get('approach_rate', 1.0)),
            '--approach-settle', str(job.get('approach_settle', 10.0)),
            '--direction', job['direction'], '--dataset-group', group,
            '--repeat-id', str(job['repeat']), '--protocol-id', job['id'],
            '--speedup', str(speedup), '--port', str(port), '--output', str(output)]
        if job['platform'] == 'multicopter':
            command.extend(['--ppva', '--continuous-position'])
        return command
    if job['kind'] == 'coupled_turn':
        command = [python, str(ROOT / 'scripts/calibrate_coupled_turns.py'), '--vehicle',
            'copter' if job['platform'] == 'multicopter' else 'plane',
            '--speeds', ','.join(str(value) for value in job['speeds']),
            '--ramp-rate', str(job['rate']), '--bank', str(job['bank_deg']),
            '--turn-signs', ','.join(job.get('signs', ('left', 'right'))),
            '--dataset-group', group, '--repeat-id', str(job['repeat']),
            '--protocol-id', job['id'], '--speedup', str(speedup),
            '--output', str(output), '--port', str(port)]
        if job.get('maximum_command_gap') is not None:
            command.extend(['--maximum-command-gap', str(job['maximum_command_gap'])])
        if job['platform'] == 'fixedwing':
            guidance = job.get('fixedwing_guidance', {})
            if guidance:
                command.extend([
                    '--guidance-mode', str(guidance['guidance_mode']),
                    '--lookahead', str(guidance['lookahead']),
                    '--cross-track-gain', str(guidance['cross_track_gain']),
                    '--course-to-bank-gain', str(guidance['course_to_bank_gain']),
                ])
        return command
    command = [python, str(ROOT / 'scripts/calibrate_turns.py'), '--vehicle',
        'copter' if job['platform'] == 'multicopter' else 'plane',
        '--speeds', ','.join(str(value) for value in job['speeds']),
        '--banks', ','.join(str(value) for value in job['banks_deg']),
        '--turn-signs', ','.join(job.get('signs', ('left', 'right'))),
        '--controller', 'course' if job['platform'] == 'fixedwing' else 'bank',
        '--dataset-group', group, '--repeat-id', str(job['repeat']),
        '--protocol-id', job['id'], '--speedup', str(speedup),
        '--output', str(output), '--port', str(port)]
    if job['platform'] == 'multicopter':
        command.extend(['--accel-feedforward', '--yaw-rate-feedforward'])
    else:
        command.extend(['--path-guidance', '--handover-command-speed',
                        '--continue-after-tracking-failure'])
        guidance = job.get('fixedwing_guidance', {})
        if guidance:
            command.extend([
                '--guidance-mode', str(guidance['guidance_mode']),
                '--lookahead', str(guidance['lookahead']),
                '--cross-track-gain', str(guidance['cross_track_gain']),
                '--course-to-bank-gain', str(guidance['course_to_bank_gain']),
            ])
    return command


def write_status(path, status):
    path.write_text(json.dumps(status, indent=2, allow_nan=False) + '\n')


def batch_requires_halt(run_state):
    return run_state in ('INVALID_EVIDENCE', 'FAILED')


def scheduled_command(command, scheduling):
    if not scheduling:
        return list(command), {}
    control_cpu = scheduling.get('control_cpu')
    sitl_cpu = scheduling.get('sitl_cpu')
    aux_cpu = scheduling.get('aux_cpu')
    if (type(control_cpu) is not int or type(sitl_cpu) is not int
            or type(aux_cpu) is not int):
        raise ValueError('host scheduling CPUs must be integers')
    if min(control_cpu, sitl_cpu, aux_cpu) < 0 or len({control_cpu, sitl_cpu, aux_cpu}) != 3:
        raise ValueError('host scheduling requires distinct nonnegative CPUs')
    return ['taskset', '-c', str(control_cpu), *command], {
        'UAV_SITL_CPU': str(sitl_cpu),
        'UAV_AUX_CPU': str(aux_cpu),
    }


def allocated_port(base_port, job_index, stride=10):
    port = base_port + job_index * stride
    if not 1024 <= port <= 65532:
        raise ValueError('allocated SITL base port is outside 1024..65532')
    return port


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=ROOT / 'config/v2/calibration_plan_v2.yaml')
    parser.add_argument('--group', choices=['A', 'B'], required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--port', type=int, default=39600)
    parser.add_argument('--only', action='append')
    parser.add_argument('--max-jobs', type=int)
    parser.add_argument('--capability', type=Path,
                        default=ROOT / 'config/v2/capability_v2.yaml')
    parser.add_argument('--acceptance', type=Path,
                        default=ROOT / 'config/v2/acceptance_protocol_v2.yaml')
    parser.add_argument('--measurement', type=Path,
                        default=ROOT / 'config/v2/measurement_definition_v2.yaml')
    args = parser.parse_args()
    if args.group == 'B':
        require_b_freeze(args.capability, args.acceptance)
    plan, group_data = load_plan(args.plan, args.group)
    jobs = expand(plan, group_data, args.group, set(args.only or []))
    if args.max_jobs is not None:
        if args.max_jobs <= 0:
            parser.error('max-jobs must be positive')
        jobs = jobs[:args.max_jobs]
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    snapshot = root / 'frozen_input'
    if not snapshot.exists():
        snapshot.mkdir()
        for path in (args.plan, args.measurement, args.acceptance, args.capability,
                     ROOT / 'config/v2/analysis_amendment_20260927.yaml'):
            shutil.copy2(path, snapshot / Path(path).name)
    status_path = root / 'batch_status.json'
    status = json.loads(status_path.read_text()) if status_path.exists() else {
        'schema_version': 1, 'group': args.group, 'formal_qualification': False,
        'plan_sha256': sha256(args.plan), 'planned_jobs': len(jobs), 'runs': {}}
    if status['plan_sha256'] != sha256(args.plan):
        raise RuntimeError('plan changed after batch creation')
    for index, job in enumerate(jobs):
        name = f"{job['id']}_rep{job['repeat']:02d}"
        destination = root / name
        existing = status['runs'].get(name, {})
        result_path = destination / 'result.json'
        if existing.get('state') == 'RUNNING' and result_path.is_file():
            result = json.loads(result_path.read_text())
            recovered = (result.get('status') == 'COMPLETED'
                         and result.get('all_tested_points_passed')
                         and result.get('exit_completed'))
            recovered_state = ('COMPLETED' if recovered else
                               'INVALID_EVIDENCE' if
                               result.get('failure_classification') == 'INVALID_EVIDENCE'
                               else 'FAILED')
            existing.update(state=recovered_state,
                            returncode=0 if recovered else 1,
                            recovered_after_batch_interruption=True)
            status['runs'][name] = existing
            write_status(status_path, status)
        if existing.get('state') in ('COMPLETED', 'FAILED', 'INVALID_EVIDENCE'):
            run_state = existing['state']
        else:
            usage = shutil.disk_usage(root)
            if usage.free < 2 * 1024**3:
                status['blocked'] = 'less than 2 GiB free; no new run started'
                write_status(status_path, status)
                raise RuntimeError(status['blocked'])
            assigned_port = allocated_port(
                args.port, index, int(plan['common'].get('port_stride', 10)))
            command = command_for(job, args.group, destination, assigned_port,
                                  float(plan['common']['speedup']))
            command, environment = scheduled_command(
                command, plan['common'].get('host_scheduling'))
            status['runs'][name] = {'state': 'RUNNING', 'job': job, 'command': command,
                                    'environment': environment,
                                    'assigned_base_port': assigned_port}
            write_status(status_path, status)
            log_path = root / (name + '.console.log')
            with log_path.open('w') as stream:
                completed = subprocess.run(command, cwd=ROOT, stdout=stream,
                    stderr=subprocess.STDOUT, env={**os.environ, **environment})
            run_state = 'COMPLETED' if completed.returncode == 0 else 'FAILED'
            if completed.returncode != 0 and result_path.is_file():
                result = json.loads(result_path.read_text())
                if result.get('failure_classification') == 'INVALID_EVIDENCE':
                    run_state = 'INVALID_EVIDENCE'
            status['runs'][name].update(state=run_state, returncode=completed.returncode)
            write_status(status_path, status)
        log_path = root / (name + '.console.log')
        if destination.is_dir() and (destination / 'logs').is_dir():
            bins = sorted((destination / 'logs').glob('*.BIN'))
            if bins and not (destination / 'v2_analysis/analysis.json').is_file():
                analysis = destination / 'v2_analysis'
                analysis_command = [sys.executable, str(ROOT / 'scripts/analyze_v2_bin.py'), str(bins[0]),
                    '--telemetry', str(destination / 'telemetry.csv'),
                    '--events', str(destination / 'events.jsonl'), '--output', str(analysis)]
                with log_path.open('a') as stream:
                    analysed = subprocess.run(analysis_command, cwd=ROOT, stdout=stream,
                                              stderr=subprocess.STDOUT)
                status['runs'][name]['analysis_returncode'] = analysed.returncode
        if (status['runs'][name].get('analysis_returncode') == 0
                or (destination / 'v2_analysis/analysis.json').is_file()):
            archive_command = [sys.executable,
                str(ROOT / 'scripts/archive_v2_calibration_run.py'), str(destination)]
            with log_path.open('a') as stream:
                archived = subprocess.run(archive_command, cwd=ROOT, stdout=stream,
                                          stderr=subprocess.STDOUT)
            status['runs'][name]['archive_returncode'] = archived.returncode
        write_status(status_path, status)
        if batch_requires_halt(run_state):
            status['halted_reason'] = (
                '%s in %s; no later preregistered job was started' %
                (run_state, name))
            write_status(status_path, status)
            break
    status['completed_jobs'] = sum(run['state'] == 'COMPLETED' for run in status['runs'].values())
    status['failed_jobs'] = sum(run['state'] == 'FAILED' for run in status['runs'].values())
    status['invalid_evidence_jobs'] = sum(
        run['state'] == 'INVALID_EVIDENCE' for run in status['runs'].values())
    status['state'] = 'FINISHED' if len(status['runs']) == len(jobs) else 'PARTIAL'
    write_status(status_path, status)
    print(json.dumps({key: status.get(key) for key in
        ('state', 'planned_jobs', 'completed_jobs', 'failed_jobs')}, indent=2))
    return (0 if not status['failed_jobs'] and not status['invalid_evidence_jobs']
            and status['state'] == 'FINISHED' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
