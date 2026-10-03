#!/usr/bin/env python3
"""Audit local socket acceptance, not aircraft receipt or execution."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink


def decode_event(event):
    if event['schema_version'] != 1 or event['event'] != 'socket_send':
        raise ValueError('unsupported journal schema')
    packet = bytes.fromhex(event['packet_hex'])
    if len(packet) != event['requested_bytes']:
        raise ValueError('packet length disagrees with journal')
    messages = mavlink.MAVLink(None).parse_buffer(packet)
    if not messages or len(messages) != 1 or bytes(messages[0].get_msgbuf()) != packet:
        raise ValueError('expected exactly one complete MAVLink packet')
    msg = messages[0]
    accepted = event['accepted_bytes']
    complete = (event['status'] == 'SOCKET_ACCEPTED_ALL'
                and type(accepted) is int and accepted == len(packet)
                and event['error'] is None)
    result = dict(send_index=event['send_index'], wall_s=event['wall_after_s'],
                  phase=event['context']['phase'], complete_local_write=complete,
                  status=event['status'], type=msg.get_type(), seq=msg.get_seq(),
                  system=msg.get_srcSystem(), component=msg.get_srcComponent(),
                  fields=msg.to_dict(), context=event['context'])
    if msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':
        result['kind'] = 'local_target'
    elif msg.get_type() == 'COMMAND_INT' and msg.command in (43000, 186, 43002):
        result['kind'] = {43000: 'airspeed', 186: 'altitude', 43002: 'course'}[msg.command]
    return result


def read_journal(path):
    decoded, issues, statuses = [], [], Counter()
    previous_index, previous_wall = 0, -math.inf
    sequences = {}
    timing = defaultdict(list)
    with Path(path).open() as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                event = json.loads(line)
                statuses[event.get('status', 'MISSING_STATUS')] += 1
                item = decode_event(event)
                before, after = event['wall_before_s'], event['wall_after_s']
                if not all(math.isfinite(v) for v in (before, after)) or after < before:
                    raise ValueError('invalid socket timestamps')
                if item['send_index'] != previous_index + 1 or before < previous_wall:
                    issues.append(dict(line=line_number, reason='journal index/time discontinuity'))
                previous_index, previous_wall = item['send_index'], after
                key = (item['system'], item['component'])
                if key in sequences and item['seq'] != (sequences[key]+1) % 256:
                    issues.append(dict(line=line_number, reason='MAVLink sequence discontinuity'))
                sequences[key] = item['seq']
                if not item['complete_local_write']:
                    issues.append(dict(line=line_number, reason='incomplete local socket write',
                                       status=item['status']))
                timing[(item['phase'], item.get('kind', item['type']))].append((before, after-before))
                decoded.append(item)
            except (ValueError, TypeError, KeyError, mavlink.MAVError) as exc:
                issues.append(dict(line=line_number, reason=str(exc)))
    if not decoded:
        issues.append(dict(reason='no decodable packets'))
    intervals = {}
    for (phase, kind), samples in timing.items():
        gaps = [b[0]-a[0] for a, b in zip(samples, samples[1:])]
        intervals[phase+'/'+kind] = dict(count=len(samples),
            maximum_gap_wall_s=max(gaps) if gaps else None,
            mean_gap_wall_s=sum(gaps)/len(gaps) if gaps else None,
            maximum_send_duration_wall_s=max(v[1] for v in samples))
    return decoded, dict(status_counts=dict(statuses), issues=issues,
                        intervals=intervals, evidence_consistent=not issues)


def number(row, key):
    raw = row.get(key)
    if raw in (None, ''):
        return None
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError('nonfinite '+key)
    return value


def align_telemetry(decoded, rows, home_offset=None):
    """Use only preceding completed writes; never interpolate a send timestamp."""
    commands = sorted((d for d in decoded if d['complete_local_write'] and 'kind' in d),
                      key=lambda d: d['wall_s'])
    cursor, last, previous_wall = 0, {}, -math.inf
    aligned, mismatches, compared, missing = [], [], Counter(), Counter()
    for index, row in enumerate(rows):
        wall = number(row, 'wall_s')
        if wall is None or wall < previous_wall:
            raise ValueError('telemetry wall clock missing or not ordered')
        previous_wall = wall
        while cursor < len(commands) and commands[cursor]['wall_s'] <= wall:
            command = commands[cursor]
            last[command['kind']] = command
            cursor += 1
        output = {key: row.get(key) for key in ('wall_s', 'sim_s', 'velocity_sim_s',
            'airspeed_sim_s', 'phase', 'mode', 've', 'vn', 'airspeed', 'groundspeed',
            'roll', 'yaw', 'alt', 'throttle', 'ref_speed', 'ref_course')}
        checks = []
        if 'local_target' in last:
            item = last['local_target']; f = item['fields']; mask = f['type_mask']
            output.update(local_send_index=item['send_index'], local_age_wall_s=wall-item['wall_s'],
                socket_north=f['x'], socket_east=f['y'], socket_down=f['z'], socket_mask=mask,
                socket_vn=f['vx'], socket_ve=f['vy'], socket_an=f['afx'], socket_ae=f['afy'],
                socket_yaw=f['yaw'], socket_yaw_rate=f['yaw_rate'], socket_frame=f['coordinate_frame'])
            checks.append(('sent_mask', mask, False))
            if f['coordinate_frame'] != 1:
                mismatches.append(dict(row=index, field='coordinate_frame', expected=1,
                                       packet=f['coordinate_frame']))
            elif home_offset is not None:
                if not mask & 1:
                    checks.append(('ref_north', f['x']+home_offset[1], False))
                if not mask & 2:
                    checks.append(('ref_east', f['y']+home_offset[0], False))
            for field, key, bit in [('sent_vn','vx',8), ('sent_ve','vy',16),
                                  ('sent_an','afx',64), ('sent_ae','afy',128),
                                  ('sent_yaw','yaw',1024), ('sent_yaw_rate','yaw_rate',2048)]:
                if not mask & bit:
                    checks.append((field, f[key], field == 'sent_yaw'))
        for kind, key in [('airspeed','param2'), ('altitude','param1'), ('course','param2')]:
            if kind in last:
                item = last[kind]; value = item['fields'][key]
                output['socket_'+kind] = value
                output[kind+'_send_index'] = item['send_index']
                output[kind+'_age_wall_s'] = wall-item['wall_s']
                if kind == 'airspeed':
                    checks.append(('ref_speed', value, False))
                elif kind == 'course':
                    checks.append(('ref_course', value, 'degrees'))
        for field, value, angular in checks:
            sample = number(row, field)
            if sample is None:
                missing[field] += 1
                continue
            compared[field] += 1
            delta = sample-value
            if angular:
                period = 360. if angular == 'degrees' else 2*math.pi
                delta = (delta+period/2) % period-period/2
            if abs(delta) > 2e-5*max(1., abs(value)):
                mismatches.append(dict(row=index, wall_s=wall, field=field,
                                       recorded=sample, packet=value, delta=delta))
        aligned.append(output)
    return aligned, dict(comparison_counts=dict(compared), mismatches=mismatches,
        missing_expected_fields=dict(missing), coverage_complete=bool(compared) and not missing,
        world_translation_known=home_offset is not None,
        values_match=(not mismatches) if compared else None,
        alignment='latest complete local socket write preceding telemetry wall_s; no interpolation',
        limitation='wall-time age is not flight-controller latency; incomplete stream invalidates receipt inference')


def audit(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    decoded, summary = read_journal(root/'outgoing_packets.jsonl')
    manifest = json.loads((root/'manifest.json').read_text())
    with (root/'telemetry.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    aligned, comparisons = align_telemetry(decoded, rows, manifest.get('home_offset'))
    summary.update(schema_version=1, source=str(root), comparisons=comparisons,
        scope='local TCP acceptance only; not aircraft receipt, ACK, control application or flight qualification',
        clock_caution='wall-time packet gaps must not be used as simulation-time acceleration',
        formal_qualification=False,
        source_sha256={name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                       for name in ('outgoing_packets.jsonl', 'telemetry.csv', 'manifest.json')},
        audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    output.mkdir(parents=True, exist_ok=False)
    with (output/'decoded_packets.jsonl').open('x') as stream:
        for item in decoded:
            stream.write(json.dumps(item, allow_nan=False)+'\n')
    with (output/'aligned.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted({k for r in aligned for k in r}))
        writer.writeheader(); writer.writerows(aligned)
    (output/'audit.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('vehicle_directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = audit(args.vehicle_directory, args.output)
    print(json.dumps({k: result[k] for k in ('status_counts', 'evidence_consistent')}, indent=2))
    print('comparisons', result['comparisons']['comparison_counts'])
    print('mismatches', len(result['comparisons']['mismatches']))
    return 0 if (result['evidence_consistent'] and result['comparisons']['values_match']
                 and result['comparisons']['coverage_complete']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
