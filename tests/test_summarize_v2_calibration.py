import json

from scripts.summarize_v2_calibration import numeric_summary, platform_of, summarize


def make_run(root, repeat, peak):
    run = root / f'rep{repeat}'; (run / 'v2_analysis').mkdir(parents=True)
    result = {'status': 'COMPLETED', 'all_tested_points_passed': True,
              'exit_completed': True, 'protocol': {'platform': 'copter',
              'protocol_id': 'condition', 'repeat_id': repeat}}
    metric = {'samples': 10, 'minimum': -peak, 'maximum': peak,
              'maximum_abs': peak, 'mean': 0, 'standard_deviation': peak / 2,
              'p95_abs': peak * .8, 'p99_abs': peak * .9, 'rms': peak / 2}
    analysis = {'record_counts': {'PSCE': 1}, 'controller_diagnostics': {},
                'derivatives': {'sim2_ground_speed_m_s': {
                'resampled_savitzky_golay': {'phases': {
                'accel_step': metric, 'takeoff': metric}}}}}
    (run / 'result.json').write_text(json.dumps(result))
    (run / 'v2_analysis' / 'analysis.json').write_text(json.dumps(analysis))
    return run


def test_numeric_summary_uses_sample_standard_deviation():
    summary = numeric_summary([1, 3])
    assert summary['mean'] == 2
    assert summary['sample_standard_deviation'] == 2 ** .5


def test_summary_keeps_repeats_and_excludes_takeoff(tmp_path):
    runs = [make_run(tmp_path, 0, .2), make_run(tmp_path, 1, .4)]
    report = summarize(runs)
    assert len(report['aggregates']) == 1
    item = report['aggregates'][0]
    assert item['repeats'] == 2
    assert item['tracking_pass_count'] == 2
    assert item['maximum_abs']['maximum'] == .4


def test_legacy_protocol_controller_prevents_signal_swap():
    assert platform_of({'controller': 'ArduCopter GUIDED position+velocity feedforward'},
                       {'record_counts': {}}) == 'copter'
    assert platform_of({'controller': 'ArduPlane GUIDED course/airspeed/altitude'},
                       {'record_counts': {}}) == 'plane'


def test_auto_prefers_versioned_hybrid_analysis(tmp_path):
    run = make_run(tmp_path, 0, .2)
    hybrid = run / 'v2_analysis_hybrid_clock'; hybrid.mkdir()
    analysis = json.loads((run / 'v2_analysis' / 'analysis.json').read_text())
    metric = analysis['derivatives']['sim2_ground_speed_m_s']['resampled_savitzky_golay']['phases']['accel_step']
    metric.update(minimum=-.3, maximum=.3, maximum_abs=.3, p95_abs=.24, p99_abs=.27, rms=.15)
    (hybrid / 'analysis.json').write_text(json.dumps(analysis))
    report = summarize([run], analysis_name='auto')
    assert report['runs'][0]['analysis_name'] == 'v2_analysis_hybrid_clock'
    assert report['aggregates'][0]['maximum_abs']['maximum'] == .3
