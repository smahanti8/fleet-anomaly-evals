"""Intermittent comms loss.

Not a magnitude fault: it marks samples stale rather than moving a telemetry
value, so it has no SNR-based detectability curve. Per ADR-003, comms gaps are
handled as excluded windows in the false-positive denominator, not as a
detectability curve, so `expected_signal_snr` returns 0.0 and
`affected_channels` is empty -- there is nothing for the ground-truth builder
to watch a threshold on.

Gaps recur on a deterministic schedule (`period_s`, `gap_duration_s` in
params) starting at injection -- no randomness, so replaying the same spec
against the same trajectory reproduces the same stale windows.

Per TelemetrySample's own contract, a comms-down tick is a present record with
a frozen payload, never an absent one: on_telemetry replaces every telemetry
field except robot_id, t_sim_s and comms_ok with the last known-good reading.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.interface import FaultSpec
from fleet_evals.sim.interface import RobotState, TelemetrySample


@dataclass(slots=True)
class CommsLossInjector:
    spec: FaultSpec
    _last_good: TelemetrySample | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.spec.fault_type is not FaultType.COMMS_LOSS:
            raise ValueError(f"spec.fault_type must be COMMS_LOSS, got {self.spec.fault_type}")

    def affected_channels(self) -> tuple[str, ...]:
        return ()

    def _active(self, t_sim_s: float) -> bool:
        if t_sim_s < self.spec.injection_t_s:
            return False
        if self.spec.duration_s is not None and t_sim_s > self.spec.injection_t_s + self.spec.duration_s:
            return False
        period = self.spec.params["period_s"]
        gap_duration = self.spec.params["gap_duration_s"]
        phase = (t_sim_s - self.spec.injection_t_s) % period
        return phase < gap_duration

    def on_state(self, state: RobotState, t_sim_s: float) -> RobotState:
        return state  # No latent-physics effect; comms is a sensor-path availability fault.

    def on_telemetry(self, sample: TelemetrySample, t_sim_s: float) -> TelemetrySample:
        if not self._active(t_sim_s):
            self._last_good = sample
            return sample
        if self._last_good is None:
            # Gap starting before any good sample was ever seen -- nothing to
            # freeze to; pass the sample through rather than inventing data.
            return sample
        return replace(
            self._last_good,
            robot_id=sample.robot_id,
            t_sim_s=sample.t_sim_s,
            comms_ok=False,
        )

    def expected_signal_snr(self, state: RobotState, t_sim_s: float, channel: str) -> float:
        return 0.0
