"""Fault injection boundary.

Separate from the simulator so the same faults can be injected behind any
backend. Two attachment points because backends differ in what they expose,
and pretending otherwise would overstate the realism of the results.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol, Sequence

from fleet_evals.faults.ground_truth import DetectabilityCrossing, FaultType
from fleet_evals.sim.interface import RobotState, TelemetrySample


@dataclass(frozen=True, slots=True)
class FaultSpec:
    """Declarative description of a fault to inject. Config-driven so a run is
    reproducible from `configs/*.yaml` plus a seed, nothing else."""

    fault_type: FaultType
    robot_id: str
    injection_t_s: float
    duration_s: float | None
    params: Mapping[str, float]


class FaultInjector(Protocol):
    """One injector instance per active FaultSpec."""

    @property
    def spec(self) -> FaultSpec: ...

    def affected_channels(self) -> Sequence[str]:
        """Channels this fault is expected to move. Used by the ground-truth
        layer to know where to watch for detectability crossings, and by the
        confusion matrix to attribute a detection to a fault type."""

    def on_state(self, state: RobotState, t_sim_s: float) -> RobotState:
        """Perturb latent physics. Preferred: the fault propagates into the
        channels causally, so a detector cannot exploit an artifact of how we
        drew the symptom. Identity when the backend exposes no state."""

    def on_telemetry(self, sample: TelemetrySample, t_sim_s: float) -> TelemetrySample:
        """Perturb at the sensor boundary. Correct for faults that genuinely
        live in the sensor path (drift, comms loss) and a documented fallback
        for the others."""

    def expected_signal_snr(
        self, state: RobotState, t_sim_s: float, channel: str
    ) -> float:
        """Fault-induced deviation on `channel`, in multiples of that channel's
        nominal noise sigma.

        This is how detectability crossings get computed, and it is the reason
        the injector rather than the evaluator owns the calculation: only the
        injector knows the counterfactual -- what this channel would have read
        with no fault present. Deriving it from observed telemetry after the
        fact would mean estimating the clean signal, which reintroduces exactly
        the interpretive assumptions ADR-002 exists to keep out.
        """


def crossings_from_snr_trace(
    trace: Sequence[tuple[float, float]],
    channel: str,
    thresholds: Sequence[float],
    sustained_s: float,
) -> tuple[DetectabilityCrossing, ...]:
    """Reduce a per-tick SNR trace to first sustained crossings per threshold.

    Implemented in `ground_truth_builder.py` once the simulator lands; declared
    here because it is part of the fault layer's contract, not the evaluator's.
    """
    raise NotImplementedError
