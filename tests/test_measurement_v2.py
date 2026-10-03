import numpy as np
import pytest

from uav_formation.measurement_v2 import (
    ProcessingSpec, derivative_sensitivity, threshold_events, unique_last,
)


def test_unique_samples_keep_last_duplicate():
    time, value = unique_last([0, 1, 1, 2], [0, 10, 11, 20])
    np.testing.assert_allclose(time, [0, 1, 2])
    np.testing.assert_allclose(value, [0, 11, 20])


def test_timestamp_reversal_is_rejected():
    with pytest.raises(ValueError, match="backwards"):
        unique_last([0, 2, 1], [0, 2, 1])


def test_all_derivatives_recover_linear_slope():
    time = np.arange(0, 8, 0.01)
    speed = 12 + 0.3 * time
    methods = derivative_sensitivity(time, speed)
    for result in methods.values():
        valid = result["valid"] & np.isfinite(result["value"])
        np.testing.assert_allclose(result["value"][valid], 0.3, atol=2e-7)


def test_resampling_does_not_bridge_large_gap():
    time = np.r_[np.arange(0, 2, 0.02), np.arange(2.5, 5, 0.02)]
    speed = 10 + time
    methods = derivative_sensitivity(time, speed, ProcessingSpec(maximum_gap_s=0.1))
    primary = methods["resampled_savitzky_golay"]
    in_gap = (primary["time"] > 2.0) & (primary["time"] < 2.5)
    assert not np.any(primary["valid"][in_gap])


def test_threshold_events_are_duration_weighted_and_merge_small_gaps():
    result = threshold_events(
        [0., .02, .04, .06, .08], [0., .5, 0., .6, 0.],
        [True] * 5, upper=.4, merge_gap_s=.03)
    assert result["event_count"] == 1
    assert result["sample_count"] == 2
    assert result["total_duration_s"] == pytest.approx(.04)
    assert result["longest_event_s"] == pytest.approx(.06)


def test_threshold_events_reject_bad_clocks_and_empty_thresholds():
    with pytest.raises(ValueError, match="strictly increasing"):
        threshold_events([0., 0.], [0., 1.], [True, True], upper=.4)
    with pytest.raises(ValueError, match="threshold"):
        threshold_events([0., 1.], [0., 1.], [True, True])
