"""The labels. This module is the point of the repository.

Written once by the fault injection layer during simulation, read only by
`fleet_evals.eval`. It is a separate store with no reader in `fleet_evals.detect`
by construction -- label leakage into a detector makes every published number
meaningless, and that should be prevented by the module graph rather than by
asking a reviewer to trust us.

Onset convention: see ADR-002. `injection_t_s` is ground truth because it is the
only quantity known without interpretation. Detectability is recorded alongside
it, never in place of it.
"""

from __future__ import annotations

import types
from dataclasses import dataclass
from enum import StrEnum
from typing import Mapping, Sequence


class FaultType(StrEnum):
    BEARING_DEGRADATION = "bearing_degradation"
    THERMAL_RUNAWAY = "thermal_runaway"
    SENSOR_DRIFT = "sensor_drift"
    COMMS_LOSS = "comms_loss"


class InjectionSite(StrEnum):
    """Where the fault entered the world.

    LATENT_STATE means the fault perturbed physics and propagated into the
    channels the way the real failure mode does. SENSOR_BOUNDARY means we
    painted the symptom onto telemetry because the backend would not let us
    reach its state. The realism claim in the README differs between these two
    and the distinction is therefore recorded per event, not per repo.
    """

    LATENT_STATE = "latent_state"
    SENSOR_BOUNDARY = "sensor_boundary"


@dataclass(frozen=True, slots=True)
class DetectabilityCrossing:
    """First sustained crossing of one SNR threshold on one channel.

    Detectability of a slow-degradation fault is a curve, not a moment: the
    answer to "when did this become visible?" depends entirely on where you put
    the threshold. Recording a single detectability timestamp would bury that
    choice. Recording the crossings at several thresholds exposes the curve and
    forces the README to name which threshold its headline number uses.
    """

    channel: str
    snr_threshold: float
    """Multiples of the channel's nominal noise sigma at injection time."""
    t_sim_s: float
    sustained_s: float
    """The crossing had to hold this long to count, so a single noisy tick
    does not get recorded as the onset of observability."""


@dataclass(frozen=True, slots=True)
class FaultEvent:
    fault_id: str
    robot_id: str
    fault_type: FaultType
    injection_site: InjectionSite
    injection_t_s: float
    """Ground truth. The denominator for recall and the zero point for total
    detection latency. Fixed, uninterpreted, independent of any detector."""
    end_t_s: float | None
    """None for faults that run to the end of the horizon, e.g. bearing wear."""
    params: Mapping[str, float]
    detectability: tuple[DetectabilityCrossing, ...]
    """Observability curve, sampled at the configured thresholds. May be empty:
    a fault injected near the end of the horizon can be genuinely
    unobservable, and that is a real and reportable outcome."""

    def __post_init__(self) -> None:
        object.__setattr__(self, "params", types.MappingProxyType(dict(self.params)))

    def crossing_at(self, snr_threshold: float) -> DetectabilityCrossing | None:
        """Earliest crossing at this threshold across all channels.

        Earliest rather than per-channel because the detector is free to watch
        any channel; the fault became observable as soon as one of them carried
        it. Returns None when the fault never reached this threshold.
        """
        candidates = [c for c in self.detectability if c.snr_threshold == snr_threshold]
        return min(candidates, key=lambda c: c.t_sim_s) if candidates else None


@dataclass(frozen=True, slots=True)
class LatencyDecomposition:
    """Total detection lag, split into the part the algorithm owns and the part
    it does not.

    A detector that fires 40 h after injection reads as a failure until you can
    show the fault was not observable above noise for the first 35 of them. The
    honest claim is then a 5 h algorithm lag, and it is provable. Reporting only
    `total_s` hides which of the two a reader is actually looking at.
    """

    total_s: float
    """detection_t - injection_t. The operationally honest number: this is how
    long the fault was live on the fleet before anyone was told."""
    signal_limited_s: float | None
    """crossing_t - injection_t. A property of the fault and the sensors. No
    algorithm can improve it. None when the fault never crossed the threshold."""
    algorithm_s: float | None
    """detection_t - crossing_t. The only part the detector controls.

    May be negative, when the detector fired while the fault was still below
    the single-channel SNR threshold -- legitimate, via cross-channel or
    temporal structure. Never clamped to zero: clamping would silently convert
    the harness's most interesting result into a boring one."""
    snr_threshold: float
    """Which point on the detectability curve this decomposition is relative to.
    A latency figure quoted without this number is not reproducible."""


@dataclass(slots=True)
class FaultLedger:
    """Append-only in memory, then persisted next to the run, not inside the
    telemetry store. Separate file, separate lifecycle, no shared connection.
    """

    events: list[FaultEvent]

    def add(self, event: FaultEvent) -> None:
        self.events.append(event)

    def for_robot(self, robot_id: str) -> Sequence[FaultEvent]:
        return [e for e in self.events if e.robot_id == robot_id]
