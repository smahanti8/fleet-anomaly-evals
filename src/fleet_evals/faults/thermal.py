"""Motor thermal runaway under sustained load.

Perturbs latent state causally -- raises duty_cycle and/or lowers
thermal_mass_c so the simulator's own thermal model drives motor_temp_c up --
rather than painting the symptom onto telemetry. InjectionSite.LATENT_STATE.

`expected_signal_snr` models the same first-order relaxation the reference
simulator uses (see sim/reference.py), analytically: the deviation between
faulted and nominal steady-state temperature, scaled by how much of that gap
has developed by `t_sim_s`. It needs `ambient_c` and `temp_gain_c_per_duty`
from the simulator config to compute the counterfactual, since only the
injector -- not the evaluator -- is allowed to know it (ADR-002).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.interface import FaultSpec
from fleet_evals.sim.interface import RobotState, TelemetrySample

CHANNEL = "motor_temp_c"


@dataclass(frozen=True, slots=True)
class ThermalRunawayInjector:
    spec: FaultSpec
    ambient_c: float
    temp_gain_c_per_duty: float

    def __post_init__(self) -> None:
        if self.spec.fault_type is not FaultType.THERMAL_RUNAWAY:
            raise ValueError(f"spec.fault_type must be THERMAL_RUNAWAY, got {self.spec.fault_type}")

    def affected_channels(self) -> tuple[str, ...]:
        return (CHANNEL,)

    def _active(self, t_sim_s: float) -> bool:
        if t_sim_s < self.spec.injection_t_s:
            return False
        if self.spec.duration_s is None:
            return True
        return t_sim_s <= self.spec.injection_t_s + self.spec.duration_s

    def _faulted_duty_cycle(self, nominal_duty_cycle: float) -> float:
        boost = self.spec.params.get("duty_cycle_boost", 0.0)
        return min(1.0, nominal_duty_cycle + boost)

    def _faulted_thermal_mass_c(self, nominal_thermal_mass_c: float) -> float:
        scale = self.spec.params.get("thermal_mass_scale", 1.0)
        return nominal_thermal_mass_c * scale

    def on_state(self, state: RobotState, t_sim_s: float) -> RobotState:
        if not self._active(t_sim_s):
            return state
        return replace(
            state,
            duty_cycle=self._faulted_duty_cycle(state.duty_cycle),
            thermal_mass_c=self._faulted_thermal_mass_c(state.thermal_mass_c),
        )

    def on_telemetry(self, sample: TelemetrySample, t_sim_s: float) -> TelemetrySample:
        return sample  # Fault lives entirely in latent state; sensor path is untouched.

    def expected_signal_snr(self, state: RobotState, t_sim_s: float, channel: str) -> float:
        if channel != CHANNEL or not self._active(t_sim_s):
            return 0.0
        nominal_steady = self.ambient_c + state.duty_cycle * self.temp_gain_c_per_duty
        faulted_duty = self._faulted_duty_cycle(state.duty_cycle)
        faulted_steady = self.ambient_c + faulted_duty * self.temp_gain_c_per_duty
        faulted_mass = self._faulted_thermal_mass_c(state.thermal_mass_c)
        elapsed = t_sim_s - self.spec.injection_t_s
        developed_fraction = 1.0 - math.exp(-elapsed / faulted_mass)
        deviation_c = (faulted_steady - nominal_steady) * developed_fraction
        return deviation_c / state.nominal_sigma[channel]
