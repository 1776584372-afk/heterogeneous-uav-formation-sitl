#!/usr/bin/env python3
"""Temporarily confine same-user background processes to experiment CPU 0."""
import argparse
import json
import os
from pathlib import Path


def process_start_ticks(pid):
    fields = Path('/proc/%d/stat' % pid).read_text().split()
    return int(fields[21])


def apply(state_path, background_cpus):
    if state_path.exists():
        raise RuntimeError('CPU isolation state already exists: '+str(state_path))
    uid = os.getuid()
    own_pid = os.getpid()
    records = []
    for entry in sorted(Path('/proc').iterdir(), key=lambda path: path.name):
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            if entry.stat().st_uid != uid or pid == own_pid:
                continue
            original = sorted(os.sched_getaffinity(pid))
            records.append({
                'pid': pid,
                'start_ticks': process_start_ticks(pid),
                'original_affinity': original,
                'command': (entry/'comm').read_text().strip(),
            })
            os.sched_setaffinity(pid, background_cpus)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state_path.write_text(json.dumps({
        'schema_version': 1,
        'background_cpus': sorted(background_cpus),
        'records': records,
    }, indent=2)+'\n')
    return len(records)


def restore(state_path):
    state = json.loads(state_path.read_text())
    restored = 0
    for record in state['records']:
        pid = int(record['pid'])
        try:
            if process_start_ticks(pid) != int(record['start_ticks']):
                continue
            os.sched_setaffinity(pid, set(record['original_affinity']))
            restored += 1
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    state_path.rename(state_path.with_suffix(state_path.suffix+'.restored'))
    return restored


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['apply', 'restore'])
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--background-cpus', default='0')
    args = parser.parse_args()
    cpus = {int(value) for value in args.background_cpus.split(',')}
    count = apply(args.state, cpus) if args.mode == 'apply' else restore(args.state)
    print(json.dumps({'mode': args.mode, 'processes': count, 'state': str(args.state)}))


if __name__ == '__main__':
    raise SystemExit(main())
