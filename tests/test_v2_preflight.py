import numpy as np

from scripts.build_v2_preflight import affine_clock, sample_clock


def test_clock_summary_exposes_duplicates_and_gaps():
    result = sample_clock([0, 0.1, 0.1, 0.4])
    assert result['duplicates'] == 1
    assert result['maximum_unique_period_s'] == 0.30000000000000004
    assert result['backward_steps'] == 0


def test_affine_clock_reports_drift_and_residual():
    source = np.arange(100, dtype=float)
    target = 3 + source * 1.00002
    result = affine_clock(source, target)
    assert abs(result['relative_rate_error_ppm'] - 20) < 1e-6
    assert result['residual_max_abs_s'] < 1e-10


def test_affine_clock_accounts_for_sitl_speedup():
    source = np.arange(100, dtype=float)
    target = 10 + source * 2.001
    result = affine_clock(source, target, expected_slope=2.0)
    assert abs(result['relative_rate_error_ppm'] - 500) < 1e-6
