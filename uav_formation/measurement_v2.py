"""Deterministic v2 preprocessing and derivative sensitivity analysis."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, savgol_filter, sosfiltfilt


@dataclass(frozen=True)
class ProcessingSpec:
    resample_hz: float = 50.0
    maximum_gap_s: float = 0.10
    window_s: float = 0.50
    polynomial_order: int = 3
    regression_window_s: float = 0.50
    butterworth_order: int = 4
    butterworth_cutoff_hz: float = 2.0

    def __post_init__(self):
        values = (self.resample_hz, self.maximum_gap_s, self.window_s,
                  self.regression_window_s, self.butterworth_cutoff_hz)
        if not all(np.isfinite(values)) or min(values) <= 0:
            raise ValueError("processing parameters must be finite and positive")
        if self.polynomial_order < 1 or self.butterworth_order < 1:
            raise ValueError("filter orders must be positive")
        if self.butterworth_cutoff_hz >= self.resample_hz / 2:
            raise ValueError("cutoff must be below Nyquist")


def unique_last(times, values):
    """Return ordered unique source samples, keeping the last duplicate."""
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    if times.ndim != 1 or len(times) != len(values) or len(times) < 3:
        raise ValueError("at least three timestamped samples are required")
    if not np.all(np.isfinite(times)) or not np.all(np.isfinite(values)):
        raise ValueError("samples must be finite")
    if np.any(np.diff(times) < 0):
        raise ValueError("source timestamps went backwards")
    unique, reverse_index = np.unique(times[::-1], return_index=True)
    keep = len(times) - 1 - reverse_index
    if len(unique) < 3:
        raise ValueError("at least three unique timestamps are required")
    return unique, values[keep]


def contiguous_resample(times, values, spec=ProcessingSpec()):
    """Resample without bridging source gaps larger than the frozen limit."""
    times, values = unique_last(times, values)
    step = 1.0 / spec.resample_hz
    grid = np.arange(times[0], times[-1] + step * 0.25, step)
    if len(grid) < 5:
        raise ValueError("insufficient duration for resampling")
    result = np.interp(grid, times, values)
    right = np.searchsorted(times, grid, side="right")
    left = np.maximum(right - 1, 0)
    right = np.minimum(right, len(times) - 1)
    covered = (times[right] - times[left] <= spec.maximum_gap_s + 1e-12)
    covered[(grid < times[0]) | (grid > times[-1])] = False
    return grid, result, covered


def _odd_window(seconds, rate, order, length):
    samples = max(order + 2, int(round(seconds * rate)))
    if samples % 2 == 0:
        samples += 1
    maximum = length if length % 2 else length - 1
    samples = min(samples, maximum)
    if samples <= order:
        raise ValueError("signal is shorter than the requested filter window")
    return samples


def windowed_slope(times, values, window_s):
    times, values = np.asarray(times, float), np.asarray(values, float)
    result = np.empty(len(times))
    half = window_s / 2
    for index, center in enumerate(times):
        selected = np.abs(times - center) <= half + 1e-12
        if np.count_nonzero(selected) < 3:
            result[index] = np.nan
            continue
        x = times[selected] - center
        result[index] = np.dot(x, values[selected] - np.mean(values[selected])) / np.dot(x, x)
    return result


def derivative_sensitivity(times, values, spec=ProcessingSpec()):
    """Compute all preregistered derivatives on native and common grids."""
    native_t, native_v = unique_last(times, values)
    native_first = np.diff(native_v) / np.diff(native_t)
    grid, sampled, covered = contiguous_resample(native_t, native_v, spec)
    dt = 1.0 / spec.resample_hz
    window = _odd_window(spec.window_s, spec.resample_hz,
                         spec.polynomial_order, len(grid))
    sos = butter(spec.butterworth_order, spec.butterworth_cutoff_hz,
                 btype="lowpass", fs=spec.resample_hz, output="sos")
    # Filtfilt padding distorts a nonzero ramp near both ends. Filtering only
    # the residual preserves a physical constant acceleration exactly.
    trend_coefficients = np.polyfit(grid, sampled, 1)
    trend = np.polyval(trend_coefficients, grid)
    buttered = trend + sosfiltfilt(sos, sampled - trend)
    methods = {
        "native_first_order_difference": {
            "time": native_t[1:], "value": native_first,
            "valid": np.diff(native_t) <= spec.maximum_gap_s,
        },
        "native_central_gradient": {
            "time": native_t, "value": np.gradient(native_v, native_t),
            "valid": np.r_[np.diff(native_t)[0] <= spec.maximum_gap_s,
                            (np.diff(native_t)[:-1] <= spec.maximum_gap_s)
                            & (np.diff(native_t)[1:] <= spec.maximum_gap_s),
                            np.diff(native_t)[-1] <= spec.maximum_gap_s],
        },
        "resampled_central_gradient": {
            "time": grid, "value": np.gradient(sampled, dt), "valid": covered,
        },
        "resampled_savitzky_golay": {
            "time": grid,
            "value": savgol_filter(sampled, window, spec.polynomial_order,
                                    deriv=1, delta=dt, mode="interp"),
            "valid": covered.copy(),
        },
        "resampled_windowed_linear_regression": {
            "time": grid, "value": windowed_slope(grid, sampled, spec.regression_window_s),
            "valid": covered.copy(),
        },
        "resampled_butterworth_zero_phase": {
            "time": grid, "value": np.gradient(buttered, dt), "valid": covered.copy(),
        },
    }
    edge = window // 2
    for name in ("resampled_savitzky_golay", "resampled_windowed_linear_regression",
                 "resampled_butterworth_zero_phase"):
        methods[name]["valid"][:edge] = False
        methods[name]["valid"][-edge:] = False
    return methods


def describe(values):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"samples": 0}
    return {
        "samples": int(len(values)),
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "maximum_abs": float(np.max(np.abs(values))),
        "mean": float(np.mean(values)),
        "standard_deviation": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "p99_abs": float(np.percentile(np.abs(values), 99)),
        "rms": float(np.sqrt(np.mean(values ** 2))),
    }


def threshold_events(times, values, valid, *, lower=None, upper=None,
                     merge_gap_s=0.10):
    """Describe duration-weighted excursions without declaring pass/fail.

    Each sample owns the interval up to the next timestamp. This gives a
    nonzero duration to an isolated sample while keeping the calculation
    deterministic on both native and resampled clocks.
    """
    times = np.asarray(times, float)
    values = np.asarray(values, float)
    valid = np.asarray(valid, bool)
    if times.ndim != 1 or len(times) != len(values) or len(times) != len(valid):
        raise ValueError("times, values, and valid must be equal-length vectors")
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError("at least two strictly increasing timestamps are required")
    if lower is None and upper is None:
        raise ValueError("at least one threshold is required")
    if lower is not None and upper is not None and lower >= upper:
        raise ValueError("lower threshold must be below upper threshold")
    finite = valid & np.isfinite(values)
    exceeded = np.zeros(len(times), bool)
    if lower is not None:
        exceeded |= values < lower
    if upper is not None:
        exceeded |= values > upper
    exceeded &= finite
    steps = np.diff(times)
    terminal_step = float(np.median(steps))
    weights = np.r_[steps, terminal_step]
    events = []
    for index in np.flatnonzero(exceeded):
        start = float(times[index])
        end = start + float(weights[index])
        if events and start - events[-1][1] <= merge_gap_s + 1e-12:
            events[-1][1] = max(events[-1][1], end)
        else:
            events.append([start, end])
    valid_duration = float(np.sum(weights[finite]))
    total_duration = float(np.sum(weights[exceeded]))
    return {
        "lower": None if lower is None else float(lower),
        "upper": None if upper is None else float(upper),
        "event_count": len(events),
        "total_duration_s": total_duration,
        "time_fraction": total_duration / valid_duration if valid_duration else None,
        "longest_event_s": max((end - start for start, end in events), default=0.0),
        "sample_count": int(np.count_nonzero(exceeded)),
        "events": [
            {"start_s": start, "end_s": end, "duration_s": end - start}
            for start, end in events
        ],
    }
