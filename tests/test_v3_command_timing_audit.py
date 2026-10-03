import pytest

from scripts.audit_v3_command_timing import timing_summary


def packet(index, wall, boot, vehicle='plane', kind='course', phase='turn_coupled_accel_14_5_-1'):
    return {
        'send_index': index,
        'wall_s': wall,
        'phase': phase,
        'kind': kind,
        'complete_local_write': True,
        'context': {'vehicle': vehicle, 'clocks': {'ATTITUDE': {'boot_s': boot}}},
    }


def test_retransmitted_duplicate_source_stamp_is_valid():
    packets = [packet(1, 1., 10.), packet(2, 1.05, 10.), packet(3, 1.10, 10.1)]
    report = timing_summary(packets, {'evidence_consistent': True}, .11)
    assert report['passed']
    assert report['maximum_wall_gap_s'] == pytest.approx(.05)
    assert report['maximum_source_gap_s'] < .11
    assert report['minimum_source_gap_s'] == 0.


def test_wall_or_source_gap_invalidates_timing_evidence():
    packets = [packet(1, 1., 10.), packet(2, 1.12, 10.12)]
    report = timing_summary(packets, {'evidence_consistent': True}, .11)
    assert not report['passed']
    assert len(report['violations']) == 1


def test_bootstrap_sequence_reset_is_caution_not_missing_write():
    packets = [packet(1, 1., 10.), packet(2, 1.05, 10.05)]
    journal = {'evidence_consistent': False, 'issues': [
        {'line': 2, 'reason': 'MAVLink sequence discontinuity'}]}
    report = timing_summary(packets, journal, .11)
    assert report['passed']
    assert report['journal_sequence_cautions'] == journal['issues']
    assert report['blocking_journal_issues'] == []


def test_formal_track_phase_can_be_audited_explicitly():
    packets = [packet(1, 1., 10., phase='track'),
               packet(2, 1.05, 10.05, phase='track')]
    report = timing_summary(packets, {'evidence_consistent': True}, .11, ('track',))
    assert report['passed']
    assert report['phase_prefixes'] == ['track']
