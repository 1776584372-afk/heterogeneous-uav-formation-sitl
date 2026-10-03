#!/usr/bin/env python3
"""Audit v3 command-cycle wall and observed source-clock intervals."""
import argparse
import json
from pathlib import Path
import tarfile
import tempfile

from pymavlink import mavutil
from scripts.audit_outgoing_packets import read_journal


def read_packets(root):
    root = Path(root)
    for name in ('command_stream_packets.jsonl', 'outgoing_packets.jsonl'):
        direct = root/name
        if direct.is_file():
            return read_journal(direct)
    archive = root/'raw_evidence.tar.gz'
    with tarfile.open(archive, 'r:gz') as stream:
        source = None
        for name in ('command_stream_packets.jsonl', 'outgoing_packets.jsonl'):
            try:
                source = stream.extractfile(name)
            except KeyError:
                source = None
            if source is not None:
                break
        if source is None:
            raise FileNotFoundError('command timing journal')
        with tempfile.NamedTemporaryFile() as target:
            target.write(source.read()); target.flush()
            return read_journal(target.name)


def raw_source_summary(path, maximum_gap_s):
    """Check source-clock continuity from the raw MAVLink evidence stream."""
    connection = mavutil.mavlink_connection(str(path), dialect='ardupilotmega')
    stamps = []
    while True:
        message = connection.recv_msg()
        if message is None:
            break
        if message.get_type() != 'ATTITUDE':
            continue
        stamp = float(message.time_boot_ms) / 1000.
        if not stamps or stamp > stamps[-1]:
            stamps.append(stamp)
    if len(stamps) < 2:
        raise ValueError('insufficient raw ATTITUDE source-clock evidence')
    gaps = [b-a for a, b in zip(stamps, stamps[1:])]
    violations = [{'previous_source_s': a, 'current_source_s': b, 'gap_s': b-a}
                  for a, b in zip(stamps, stamps[1:])
                  if b-a > maximum_gap_s + 1e-12 or b-a < -1e-12]
    return {
        'path': str(path),
        'message': 'ATTITUDE.time_boot_ms from raw.tlog',
        'samples': len(stamps),
        'maximum_gap_s': max(gaps),
        'minimum_gap_s': min(gaps),
        'violations': violations,
        'passed': not violations,
    }


def timing_summary(packets, journal, maximum_gap_s, phase_prefixes=('turn_coupled_',),
                   source_evidence=None):
    vehicles = {packet['context']['vehicle'] for packet in packets}
    if len(vehicles) != 1:
        raise ValueError('journal must contain exactly one vehicle type')
    vehicle = vehicles.pop()
    cycle_kind = 'course' if vehicle == 'plane' else 'local_target'
    selected = [packet for packet in packets
                if packet.get('kind') == cycle_kind
                and packet['complete_local_write']
                and packet['phase'].startswith(tuple(phase_prefixes))]
    if len(selected) < 2:
        raise ValueError('insufficient command cycles')
    wall_gaps = []
    source_gaps = []
    violations = []
    observed_source_violations = []
    for previous, current in zip(selected, selected[1:]):
        if previous['phase'].rsplit('_', 1)[-1] != current['phase'].rsplit('_', 1)[-1]:
            continue
        wall_gap = current['wall_s'] - previous['wall_s']
        previous_clock = (previous['context'].get('clocks', {})
                          .get('ATTITUDE', {}).get('boot_s'))
        current_clock = (current['context'].get('clocks', {})
                         .get('ATTITUDE', {}).get('boot_s'))
        source_gap = None
        if previous_clock is not None and current_clock is not None:
            source_gap = float(current_clock) - float(previous_clock)
            source_gaps.append(source_gap)
        wall_gaps.append(wall_gap)
        source_bad = (source_gap is None or source_gap > maximum_gap_s + 1e-12
                      or source_gap < -1e-12)
        if source_bad:
            observed_source_violations.append({
                'previous_send_index': previous['send_index'],
                'current_send_index': current['send_index'],
                'phase': current['phase'],
                'wall_gap_s': wall_gap,
                'source_gap_s': source_gap,
            })
        if wall_gap > maximum_gap_s + 1e-12 or (source_evidence is None and source_bad):
            violations.append({
                'previous_send_index': previous['send_index'],
                'current_send_index': current['send_index'],
                'phase': current['phase'],
                'wall_gap_s': wall_gap,
                'source_gap_s': source_gap,
            })
    sequence_cautions = [issue for issue in journal.get('issues', [])
                         if issue.get('reason') == 'MAVLink sequence discontinuity']
    blocking_journal_issues = [issue for issue in journal.get('issues', [])
                               if issue.get('reason') != 'MAVLink sequence discontinuity']
    return {
        'schema_version': 1,
        'vehicle': vehicle,
        'cycle_packet_kind': cycle_kind,
        'cycles': len(selected),
        'maximum_allowed_gap_s': maximum_gap_s,
        'phase_prefixes': list(phase_prefixes),
        'maximum_wall_gap_s': max(wall_gaps),
        'maximum_source_gap_s': max(source_gaps),
        'minimum_source_gap_s': min(source_gaps),
        'violations': violations,
        'observed_source_violations': observed_source_violations,
        'raw_source_evidence': source_evidence,
        'journal_evidence_consistent_original': journal['evidence_consistent'],
        'journal_transport_consistent': not blocking_journal_issues,
        'journal_sequence_cautions': sequence_cautions,
        'blocking_journal_issues': blocking_journal_issues,
        'passed': (not violations and not blocking_journal_issues
                   and (source_evidence is None or source_evidence['passed'])),
        'clock_scope': ('source clock is latest observed ATTITUDE boot time at local socket send; '
                        'it is not firmware command-processing time'),
        'sequence_scope': ('MAVLink sequence resets are reported but do not imply a missing local '
                           'socket write when journal send_index and timestamps remain continuous'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--maximum-gap', type=float, default=.11)
    parser.add_argument('--phase-prefix', action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    packets, journal = read_packets(args.run)
    report = timing_summary(packets, journal, args.maximum_gap,
                            tuple(args.phase_prefix) or ('turn_coupled_',))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
