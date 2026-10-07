from __future__ import annotations

import pytest

from fleet_evals.faults.ground_truth_builder import crossings_from_snr_trace


def _trace(*snrs, dt=1.0, start=0.0):
    return [(start + i * dt, snr) for i, snr in enumerate(snrs)]


def test_never_crosses():
    trace = _trace(0.0, 0.5, 0.9, 0.5, 0.0)
    result = crossings_from_snr_trace(trace, "motor_temp_c", [3.0], sustained_s=2.0)
    assert result == ()


def test_spike_shorter_than_sustained_gives_none():
    # dt=1, sustained_s=3 -> n_required = 3 consecutive samples; a 2-sample
    # spike above threshold does not qualify.
    trace = _trace(0.0, 3.0, 3.0, 0.0, 0.0)
    result = crossings_from_snr_trace(trace, "ch", [3.0], sustained_s=3.0)
    assert result == ()


def test_short_spike_then_later_sustained_run_reports_later_start():
    # index0..1: 2-sample spike (too short). index3..5: 3-sample sustained run.
    trace = _trace(3.0, 3.0, 0.0, 3.0, 3.0, 3.0)
    result = crossings_from_snr_trace(trace, "ch", [3.0], sustained_s=3.0)
    assert len(result) == 1
    assert result[0].t_sim_s == 3.0
    assert result[0].snr_threshold == 3.0
    assert result[0].channel == "ch"
    assert result[0].sustained_s == 3.0


def test_run_exactly_sustained_long_is_accepted():
    # dt=1, sustained_s=2 -> n_required=2.
    trace = _trace(3.0, 3.0, 0.0)
    result = crossings_from_snr_trace(trace, "ch", [3.0], sustained_s=2.0)
    assert len(result) == 1
    assert result[0].t_sim_s == 0.0


def test_run_one_sample_shorter_is_rejected():
    # dt=1, sustained_s=3 -> n_required=3; only 2 qualifying samples exist.
    trace = _trace(3.0, 3.0, 0.0)
    result = crossings_from_snr_trace(trace, "ch", [3.0], sustained_s=3.0)
    assert result == ()


def test_run_cut_off_by_trace_end_is_rejected():
    # Run starts at the last two samples but the trace ends before it can be
    # confirmed sustained for 3 seconds.
    trace = _trace(0.0, 0.0, 3.0, 3.0)
    result = crossings_from_snr_trace(trace, "ch", [3.0], sustained_s=3.0)
    assert result == ()


def test_sustained_s_zero_accepts_single_qualifying_sample():
    trace = _trace(0.0, 3.0, 0.0)
    result = crossings_from_snr_trace(trace, "ch", [3.0], sustained_s=0.0)
    assert len(result) == 1
    assert result[0].t_sim_s == 1.0


def test_multiple_thresholds_independent():
    trace = _trace(1.0, 1.0, 1.0, 4.0, 4.0, 4.0)
    result = crossings_from_snr_trace(trace, "ch", [1.0, 4.0, 10.0], sustained_s=3.0)
    by_threshold = {c.snr_threshold: c for c in result}
    assert by_threshold[1.0].t_sim_s == 0.0
    assert by_threshold[4.0].t_sim_s == 3.0
    assert 10.0 not in by_threshold


def test_empty_trace_returns_empty():
    assert crossings_from_snr_trace([], "ch", [1.0], sustained_s=1.0) == ()


def test_negative_sustained_s_raises():
    with pytest.raises(ValueError):
        crossings_from_snr_trace(_trace(1.0), "ch", [1.0], sustained_s=-1.0)


def test_non_increasing_timestamps_raise():
    with pytest.raises(ValueError):
        crossings_from_snr_trace([(1.0, 1.0), (1.0, 2.0)], "ch", [1.0], sustained_s=0.0)


def test_negative_snr_raises():
    with pytest.raises(ValueError):
        crossings_from_snr_trace([(0.0, -1.0)], "ch", [1.0], sustained_s=0.0)
