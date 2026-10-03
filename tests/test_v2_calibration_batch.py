import pytest

from scripts.run_v2_calibration_batch import (
    batch_requires_halt,
    allocated_port,
    command_for,
    expand,
    require_b_freeze,
    scheduled_command,
)


def test_batch_halts_only_for_invalid_timing_evidence():
    assert batch_requires_halt('INVALID_EVIDENCE')
    assert batch_requires_halt('FAILED')
    assert not batch_requires_halt('COMPLETED')


def test_batch_allocates_distinct_ports_per_job():
    assert [allocated_port(39980,index) for index in range(3)] == [39980,39990,40000]
    with pytest.raises(ValueError):
        allocated_port(65530,1)


def test_host_scheduling_separates_control_and_sitl_cpus():
    command, environment = scheduled_command(
        ['python3', 'flight.py'], {'control_cpu': 3, 'sitl_cpu': 2, 'aux_cpu': 1})
    assert command == ['taskset', '-c', '3', 'python3', 'flight.py']
    assert environment == {'UAV_SITL_CPU': '2', 'UAV_AUX_CPU': '1'}
    with pytest.raises(ValueError):
        scheduled_command(
            ['python3'], {'control_cpu': 2, 'sitl_cpu': 2, 'aux_cpu': 1})


def test_expand_keeps_groups_and_repeats_separate():
    plan = {'common': {'A_repeats': 3, 'B_repeats': 5}}
    group = {'straight_ramps': [{'id': 'one', 'platform': 'multicopter',
        'direction': 'forward', 'speeds': [1, 2, 1], 'rate': .1}], 'steady_turns': []}
    assert len(expand(plan, group, 'A')) == 3
    assert len(expand(plan, group, 'B')) == 5


def test_b_group_unlocks_only_after_v2_freeze():
    require_b_freeze()


def test_turn_command_preserves_preregistered_identity_and_conditions(tmp_path):
    job = {'kind': 'steady_turn', 'repeat': 2, 'id': 'fw_turn',
           'platform': 'fixedwing', 'speeds': [14], 'banks_deg': [5],
           'signs': ['right']}
    command = command_for(job, 'A', tmp_path / 'run', 39600, 4.0)
    assert command[command.index('--dataset-group') + 1] == 'A'
    assert command[command.index('--repeat-id') + 1] == '2'
    assert command[command.index('--protocol-id') + 1] == 'fw_turn'
    assert '--continue-after-tracking-failure' in command
    assert command[command.index('--turn-signs') + 1] == 'right'
    assert command[command.index('--speedup') + 1] == '4.0'


def test_coupled_turn_command_preserves_work_point(tmp_path):
    job = {'kind': 'coupled_turn', 'repeat': 1, 'id': 'mc_coupled',
           'platform': 'multicopter', 'speeds': [12, 13, 12], 'rate': .3,
           'bank_deg': 5, 'signs': ['left', 'right']}
    command = command_for(job, 'A', tmp_path / 'run', 39900, 4.0)
    assert command[1].endswith('calibrate_coupled_turns.py')
    assert command[command.index('--speeds') + 1] == '12,13,12'
    assert command[command.index('--ramp-rate') + 1] == '0.3'
    assert command[command.index('--bank') + 1] == '5'
    assert command[command.index('--dataset-group') + 1] == 'A'


def test_coupled_turn_command_carries_preregistered_timing_gate(tmp_path):
    job = {'kind': 'coupled_turn', 'repeat': 0, 'id': 'mc_v3',
           'platform': 'multicopter', 'speeds': [14, 15, 14], 'rate': .3,
           'bank_deg': 5, 'signs': ['left', 'right'],
           'maximum_command_gap': .11}
    command = command_for(job, 'A', tmp_path / 'run', 39900, 2.0)
    assert command[command.index('--maximum-command-gap') + 1] == '0.11'
    assert command[command.index('--speedup') + 1] == '2.0'


def test_fixedwing_coupled_command_carries_frozen_guidance(tmp_path):
    guidance = {'guidance_mode': 'curvature_ff', 'lookahead': 40.,
                'cross_track_gain': 1.,
                'course_to_bank_gain': 0.8726646259971648}
    job = {'kind': 'coupled_turn', 'repeat': 0, 'id': 'fw_coupled',
           'platform': 'fixedwing', 'speeds': [14, 15, 14], 'rate': .05,
           'bank_deg': 5, 'signs': ['left', 'right'],
           'fixedwing_guidance': guidance}
    command = command_for(job, 'A', tmp_path / 'run', 39900, 4.)
    assert command[command.index('--guidance-mode') + 1] == 'curvature_ff'
    assert command[command.index('--lookahead') + 1] == '40.0'
    assert command[command.index('--cross-track-gain') + 1] == '1.0'
    assert float(command[command.index('--course-to-bank-gain') + 1]) == pytest.approx(
        guidance['course_to_bank_gain'])
