import io
import json

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from scripts.audit_outgoing_packets import align_telemetry, decode_event, read_journal


def event(t=1., index=1, complete=True):
    stream = io.BytesIO()
    mav = mavlink.MAVLink(stream)
    mav.seq = (index-1) % 256
    mav.set_position_target_local_ned_send(0, 1, 1, 1, 0,
        20., 10., -60., 0., 12., 0., 0., .1, 0., 1.57, -.2)
    packet = stream.getvalue()
    return dict(schema_version=1, event='socket_send', send_index=index,
        wall_before_s=t-.001, wall_after_s=t, requested_bytes=len(packet),
        accepted_bytes=len(packet) if complete else 2,
        status='SOCKET_ACCEPTED_ALL' if complete else 'SHORT_WRITE', error=None,
        packet_hex=packet.hex(), context={'phase': 'track'})


def test_asof_does_not_use_future_or_partial_packet():
    packets = [decode_event(event()), decode_event(event(2., 2, False)),
               decode_event(event(3., 3))]
    rows = [dict(wall_s=t, sent_ve=12., sent_mask=0) for t in (.5, 1.5, 2.5, 3.5)]
    aligned, result = align_telemetry(packets, rows)
    assert 'local_send_index' not in aligned[0]
    assert [r['local_send_index'] for r in aligned[1:]] == [1, 1, 3]
    assert result['values_match'] is True


def test_mismatch_and_missing_evidence_not_silently_passed():
    packets = [decode_event(event())]
    _, result = align_telemetry(packets, [dict(wall_s=2., sent_ve=15.)])
    assert result['values_match'] is False
    assert result['mismatches'][0]['field'] == 'sent_ve'
    _, result = align_telemetry(packets, [dict(wall_s=2.)])
    assert result['values_match'] is None


def test_bad_crc_and_length_are_rejected():
    record = event()
    record['requested_bytes'] += 1
    with pytest.raises(ValueError):
        decode_event(record)
    record = event()
    packet = bytearray.fromhex(record['packet_hex']); packet[-1] ^= 1
    record['packet_hex'] = packet.hex()
    with pytest.raises(mavlink.MAVError):
        decode_event(record)


def test_journal_gaps_partial_and_truncated_records_fail(tmp_path):
    path = tmp_path/'journal.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in [event(), event(2., 3, False)])+'\n{"truncated":')
    decoded, result = read_journal(path)
    assert len(decoded) == 2
    assert result['evidence_consistent'] is False
    assert len(result['issues']) == 4


def test_angle_wrap_and_ignored_fields():
    packet = decode_event(event())
    packet['fields']['type_mask'] = 2048
    _, result = align_telemetry([packet], [dict(wall_s=2., sent_yaw=1.57+2*3.141592653589793,
        sent_yaw_rate=100., sent_mask=2048)])
    assert result['values_match'] is True
    assert 'sent_yaw_rate' not in result['comparison_counts']


def test_fixedwing_targets_use_separate_fields():
    packets = []
    for index, (code, p1, p2) in enumerate([(43000,0.,14.), (186,60.,3.), (43002,0.,90.)], 1):
        stream = io.BytesIO(); mav = mavlink.MAVLink(stream)
        mav.command_int_send(1, 1, 3, code, 0, 0, p1, p2, 0., 0., 0, 0, 0.)
        record = event(index, index); packet = stream.getvalue()
        record.update(packet_hex=packet.hex(), requested_bytes=len(packet), accepted_bytes=len(packet))
        packets.append(decode_event(record))
    aligned, result = align_telemetry(packets, [dict(wall_s=4., ref_speed=14., ref_course=450., airspeed=13.)])
    assert result['values_match'] is True
    assert aligned[0]['socket_airspeed'] == 14.
    assert aligned[0]['airspeed'] == 13.
    assert aligned[0]['socket_altitude'] == 60.


def test_positions_use_recorded_world_translation():
    packet = decode_event(event())
    _, result = align_telemetry([packet], [dict(wall_s=2., ref_east=110., ref_north=220.)], (100., 200.))
    assert result['values_match'] is True
    assert result['comparison_counts'] == {'ref_north': 1, 'ref_east': 1}
    _, result = align_telemetry([packet], [dict(wall_s=2., ref_east=110., ref_north=220.)])
    assert result['values_match'] is None


def test_partial_comparison_coverage_is_explicit():
    _, result = align_telemetry([decode_event(event())], [dict(wall_s=2., sent_ve=12.)])
    assert result['values_match'] is True
    assert result['coverage_complete'] is False
    assert result['missing_expected_fields']['sent_yaw'] == 1
