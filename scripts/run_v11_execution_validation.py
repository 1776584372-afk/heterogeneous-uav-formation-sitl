#!/usr/bin/env python3
"""Run the preregistered v11 execution-timing independent validation group."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

from scripts.audit_v3_formal_run import audit
from uav_formation.experiment_artifacts import ROOT, write_json


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_protocol(protocol):
    for section in ('source_sha256', 'task_sha256'):
        for path, expected in protocol[section].items():
            if expected == 'TBD_AFTER_CREATION':
                raise ValueError('protocol source hash not finalized: '+path)
            if sha256(ROOT/path) != expected:
                raise ValueError('frozen hash mismatch: '+path)
    for item in protocol['unchanged_contracts'].values():
        if sha256(ROOT/item['path']) != item['sha256']:
            raise ValueError('frozen contract mismatch: '+item['path'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path,
                        default=ROOT/'config/v5/execution_protocol_v5.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = yaml.safe_load(protocol_path.read_text())
    validate_protocol(protocol)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    state = {'status': 'RUNNING', 'protocol': str(protocol_path),
             'protocol_sha256': sha256(protocol_path), 'runs': []}
    write_json(output/'validation_status.json', state)
    scheduling = protocol['execution']
    isolation = output/'cpu_isolation.json'
    subprocess.run([sys.executable, str(ROOT/'scripts/host_cpu_isolation.py'),
                    'apply', '--state', str(isolation), '--background-cpus',
                    str(scheduling['background_cpu'])], cwd=ROOT, check=True)
    tasks = {'S1': ROOT/'config/experiments/v3/s1.yaml',
             'S2': ROOT/'config/experiments/v3/s2.yaml',
             'S3': ROOT/'config/experiments/v3/s3.yaml'}
    environment = dict(os.environ,
                       UAV_SITL_CPU=str(scheduling['sitl_cpu']),
                       UAV_AUX_CPU=str(scheduling['auxiliary_cpu']),
                       UAV_CONTROL_CPU=str(scheduling['control_cpu']),
                       UAV_STREAM_CPU=str(scheduling['stream_cpu']))
    try:
        for index, item in enumerate(protocol['independent_group_B']['required_runs']):
            run = output/item['id']
            # The runner pins its control thread internally.  Leaving the
            # process affinity unrestricted lets the streamer thread own a
            # separate CPU instead of competing with NMPC on one CPU.
            command = [sys.executable, str(ROOT/'scripts/run_paper_sitl.py'), '--config',
                       str(tasks[item['scenario']]), '--output', str(run),
                       '--base-port', str(29700+40*index), '--seed', str(item['seed']),
                       '--method', 'F', '--speedup', str(scheduling['sitl_speedup'])]
            partial = 'duration_s' in item
            if partial:
                command.extend(['--development', '--track-seconds', str(item['duration_s'])])
            if item['scenario'] == 'S3':
                command.extend(['--rotor-precheck', str(output/'s1_full_seed21')])
            console = output/(item['id']+'.console.log')
            began = time.monotonic()
            with console.open('w') as stream:
                returncode = subprocess.run(command, cwd=ROOT, env=environment,
                                            stdout=stream, stderr=subprocess.STDOUT).returncode
            record = dict(item, returncode=returncode,
                          wall_seconds=time.monotonic()-began, command=command,
                          console=str(console))
            if run.exists() and returncode == 0:
                try:
                    result = audit(run, allow_partial=partial)
                    record.update(classification=result['classification'],
                                  acceptance=str(run/'v3_formal_acceptance.json'),
                                  acceptance_sha256=sha256(run/'v3_formal_acceptance.json'))
                except Exception as exc:
                    record['audit_error'] = repr(exc)
            state['runs'].append(record)
            write_json(output/'validation_status.json', state)
            qualified = record.get('classification') in ('SYSTEM_QUALIFIED',
                                                          'SYSTEM_PRECHECK_QUALIFIED')
            if returncode != 0 or not qualified:
                state.update(status='FAILED', failed_run=item['id'])
                write_json(output/'validation_status.json', state)
                return 1
        state['status'] = 'PASSED'
        write_json(output/'validation_status.json', state)
        return 0
    except KeyboardInterrupt:
        state['status'] = 'INTERRUPTED'
        write_json(output/'validation_status.json', state)
        return 130
    finally:
        subprocess.run([sys.executable, str(ROOT/'scripts/host_cpu_isolation.py'),
                        'restore', '--state', str(isolation)], cwd=ROOT, check=True)


if __name__ == '__main__':
    raise SystemExit(main())
