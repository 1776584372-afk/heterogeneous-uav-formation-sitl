import numpy as np

from scripts.analyze_v2_bin import (control_summary, event_summary, load_phase_events,
                                    merged_phase_map, phase_boundary_validity)


def test_event_summary_separates_short_and_sustained_events():
    time = np.arange(0, 1.01, 0.1)
    value = np.zeros_like(time)
    value[[2, 5, 6, 7, 8]] = 2
    result = event_summary(time, value, np.ones_like(time, bool), 1, merge_gap_s=0.11)
    assert result['event_count'] == 2
    assert result['longest_contiguous_event_s'] == 0.30000000000000004


def test_invalid_samples_do_not_form_events():
    time = np.arange(5, dtype=float)
    value = np.ones(5) * 2
    result = event_summary(time, value, [False, False, True, False, False], 1)
    assert result['event_count'] == 1
    assert result['longest_contiguous_event_s'] == 0


def test_phase_transition_excludes_both_filter_halves():
    time = np.arange(0, 2.01, .1)
    phase = np.where(time < 1, 'before', 'after')
    valid = phase_boundary_validity(time, phase, .25)
    assert not np.any(valid[(time >= .75) & (time <= 1.15)])
    assert np.all(valid[(time <= .65) | (time >= 1.25)])


def test_controller_summary_separates_plane_throttle_and_surfaces():
    phase_map = ([0.0, 1.0], ['steady', 'steady'])
    rows = {
        'TECS': [{'boot_s': 0.0, 'spdem': 15.0, 'sp': 14.8, 'th': .7, 'ph': .1}],
        'RCOU': [{'boot_s': 0.0, 'C1': 1950, 'C2': 1500,
                  'C3': 2000, 'C4': 1050}],
    }
    report = control_summary(rows, phase_map, 'plane')
    assert report['signals']['fixedwing_tecs_airspeed_m_s']['all']['maximum'] == 14.8
    throttle = report['actuator']['throttle_pwm']['phases']['steady']
    surfaces = report['actuator']['surface_pwm']['phases']['steady']
    assert throttle['high_fraction'] == 1.0
    assert surfaces['high_fraction'] == 1 / 3
    assert surfaces['low_fraction'] == 1 / 3


def test_controller_summary_reports_copter_internal_acceleration():
    phase_map = ([0.0], ['accel'])
    rows = {
        'PSCE': [{'boot_s': 0.0, 'DAE': .2, 'TAE': .3, 'AE': .4}],
        'RCOU': [{'boot_s': 0.0, 'C1': 1000, 'C2': 1200,
                  'C3': 1400, 'C4': 1960}],
    }
    report = control_summary(rows, phase_map, 'copter')
    signal = report['signals']['copter_psce_lean_equivalent_accel_east_m_s2']['all']
    assert signal['maximum'] == .4
    assert report['actuator']['motor_pwm']['phases']['accel']['high_fraction'] == .25


def test_phase_events_use_native_boot_timestamp(tmp_path):
    path = tmp_path / 'events.jsonl'
    path.write_text('{"event":"other","boot_s":1,"phase":"x"}\n'
                    '{"event":"phase_transition","boot_s":2.5,"phase":"accel"}\n')
    assert load_phase_events(path) == ([2.5], ['accel'])


def test_event_clock_wins_and_telemetry_only_supplies_missing_lifecycle(tmp_path):
    events = tmp_path / 'events.jsonl'
    events.write_text('{"event":"phase_transition","boot_s":2.0,"phase":"accel"}\n')
    telemetry = tmp_path / 'telemetry.csv'
    telemetry.write_text('sim_s,phase\n0,takeoff\n2.2,accel\n4,exit\n')
    assert merged_phase_map(telemetry, events) == (
        [0.0, 2.0, 4.0], ['takeoff', 'accel', 'exit'])
