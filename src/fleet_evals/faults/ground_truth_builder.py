"""Turns a per-channel expected-SNR trace into detectability crossings.

Lives here rather than in faults/interface.py because it needs the simulator
to exist to be tested meaningfully (see that module's docstring); it stays
part of the fault layer's contract, not the evaluator's, per ADR-002.
"""

from __future__ import annotations

import math
from typing import Sequence

from fleet_evals.faults.ground_truth import DetectabilityCrossing


def crossings_from_snr_trace(
    trace: Sequence[tuple[float, float]],
    channel: str,
    thresholds: Sequence[float],
    sustained_s: float,
) -> tuple[DetectabilityCrossing, ...]:
    """First sustained run, per threshold, of `FaultInjector.expected_signal_snr`
    samples for one channel -- never observed telemetry; that distinction is
    what keeps ground truth uninterpreted (ADR-002).

    A run's samples must be consecutive (no dip below threshold in between).
    It counts once it reaches `ceil(sustained_s / dt)` samples, `dt` inferred
    from the trace's own (assumed uniform) spacing; the reported `t_sim_s` is
    the run's start, not its confirmation point. A run still accumulating
    when the trace ends does not count -- an unobservable fault produces no
    crossing for that threshold, not an error.

    No interpolation: crossing times are exactly the sampled timestamps.
    """
    if sustained_s < 0:
        raise ValueError("sustained_s must be non-negative")
    if not trace:
        return ()

    times = [t for t, _ in trace]
    for previous, current in zip(times, times[1:]):
        if current <= previous:
            raise ValueError("trace timestamps must be strictly increasing")
    for _, snr in trace:
        if snr < 0:
            raise ValueError("snr must be non-negative")

    dt = times[1] - times[0] if len(times) > 1 else None
    n_required = 1 if sustained_s == 0 else (
        max(1, math.ceil(sustained_s / dt)) if dt is not None else math.inf
    )

    crossings = []
    for threshold in thresholds:
        crossing_t = _first_sustained_run_start(trace, threshold, n_required)
        if crossing_t is not None:
            crossings.append(
                DetectabilityCrossing(
                    channel=channel,
                    snr_threshold=threshold,
                    t_sim_s=crossing_t,
                    sustained_s=sustained_s,
                )
            )
    return tuple(crossings)


def _first_sustained_run_start(
    trace: Sequence[tuple[float, float]], threshold: float, n_required: float
) -> float | None:
    run_start_t: float | None = None
    run_len = 0
    for t, snr in trace:
        if snr >= threshold:
            if run_start_t is None:
                run_start_t = t
            run_len += 1
            if run_len >= n_required:
                return run_start_t
        else:
            run_start_t = None
            run_len = 0
    return None
