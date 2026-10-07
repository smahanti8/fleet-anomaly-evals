"""Sensor drift: a fault that genuinely lives in the sensor path.

Physics is untouched -- on_state is identity -- the injector adds a bias
directly to one telemetry channel, growing linearly since injection.
InjectionSite.SENSOR_BOUNDARY. `expected_signal_snr` is exact rather than
modeled, since the injector added the bias itself.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.interface import FaultSpec
from fleet_evals.sim.interface import RobotState, TelemetrySample

_TELEMETRY_FIELDS = {
    "battery_soc",
    "motor_temp_c",
    "vibration_rms_g",
}


@dataclass(frozen=True, slots=True)
class SensorDriftInjector:
    spec: FaultSpec

    def __post_init__(self) -> None:
        if self.spec.fault_type is not FaultType.SENSOR_DRIFT:
            raise ValueError(f"spec.fault_type must be SENSOR_DRIFT, got {self.spec.fault_type}")
        channel = self.spec.params.get("channel")
        if channel not in _TELEMETRY_FIELDS:
            raise ValueError(f"params['channel'] must be one of {_TELEMETRY_FIELDS}, got {channel!r}")

    @property
    def _channel(self) -> str:
        return self.spec.params["channel"]

    @property
    def _drift_rate(self) -> float:
        return self.spec.params.get("drift_rate_per_s", 0.0)

    def affected_channels(self) -> tuple[str, ...]:
        return (self._channel,)

    def _active(self, t_sim_s: float) -> bool:
        if t_sim_s < self.spec.injection_t_s:
            return False
        if self.spec.duration_s is None:
            return True
        return t_sim_s <= self.spec.injection_t_s + self.spec.duration_s

    def _bias(self, t_sim_s: float) -> float:
        if not self._active(t_sim_s):
            return 0.0
        return self._drift_rate * (t_sim_s - self.spec.injection_t_s)

    def on_state(self, state: RobotState, t_sim_s: float) -> RobotState:
        return state  # Fault lives at the sensor boundary; physics is untouched.

    def on_telemetry(self, sample: TelemetrySample, t_sim_s: float) -> TelemetrySample:
        bias = self._bias(t_sim_s)
        if bias == 0.0:
            return sample
        current = getattr(sample, self._channel)
        return replace(sample, **{self._channel: current + bias})

    def expected_signal_snr(self, state: RobotState, t_sim_s: float, channel: str) -> float:
        if channel != self._channel:
            return 0.0
        return self._bias(t_sim_s) / state.nominal_sigma[channel]
