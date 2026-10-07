"""Records a fault's true onset to the FaultLedger.

Detectability is computed here, not by the injector: the injector only knows
how to evaluate one tick's SNR, and the ledger stores the reduced crossing
curve via `crossings_from_snr_trace`, not the raw trace. `injection_t_s` is
copied from the FaultSpec verbatim -- never derived, rounded, or recomputed --
since it is ADR-002's ground truth.
"""

from __future__ import annotations

from typing import Mapping, Sequence

from fleet_evals.faults.ground_truth import (
    DetectabilityCrossing,
    FaultEvent,
    FaultLedger,
    InjectionSite,
)
from fleet_evals.faults.ground_truth_builder import crossings_from_snr_trace
from fleet_evals.faults.interface import FaultInjector


def record_fault_event(
    injector: FaultInjector,
    fault_id: str,
    injection_site: InjectionSite,
    ledger: FaultLedger,
    snr_traces: Mapping[str, Sequence[tuple[float, float]]],
    thresholds: Sequence[float],
    sustained_s: float,
) -> FaultEvent:
    """Build one FaultEvent from an injector's spec and pre-sampled per-channel
    SNR traces, and append it to `ledger`.

    `snr_traces` maps channel -> (t_sim_s, snr) samples, one entry per channel
    in `injector.affected_channels()`. Sampling those traces from a running
    simulator is the caller's job, not this function's -- keeping this module
    testable against hand-built traces without a full simulator run.
    """
    spec = injector.spec
    detectability: list[DetectabilityCrossing] = []
    for channel in injector.affected_channels():
        trace = snr_traces.get(channel, ())
        detectability.extend(crossings_from_snr_trace(trace, channel, thresholds, sustained_s))

    event = FaultEvent(
        fault_id=fault_id,
        robot_id=spec.robot_id,
        fault_type=spec.fault_type,
        injection_site=injection_site,
        injection_t_s=spec.injection_t_s,
        end_t_s=None if spec.duration_s is None else spec.injection_t_s + spec.duration_s,
        params=spec.params,
        detectability=tuple(detectability),
    )
    ledger.add(event)
    return event
