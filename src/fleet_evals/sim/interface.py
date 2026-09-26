"""Fleet simulator boundary.

Deliberately thin. The reference backend is physics-lite, but the evaluation
harness must run unchanged against Gazebo, Isaac Sim, or recorded telemetry from
a real fleet. Nothing downstream of this module may learn which backend it has.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    """One robot, one 1 Hz tick, as it would arrive off the wire.

    `comms_ok=False` is a present record with stale payload, never an absent
    record. A detector that reads a comms gap as nominal is the most common
    silent failure in fleet monitoring; keeping the row makes that failure
    measurable instead of invisible.
    """

    robot_id: str
    t_sim_s: float
    battery_soc: float
    motor_temp_c: float
    joint_torque_nm: tuple[float, ...]
    vibration_rms_g: float
    cycle_count: int
    comms_ok: bool


@dataclass(frozen=True, slots=True)
class RobotState:
    """Latent physical state, unobservable to a detector.

    Exists so fault injection can perturb causes rather than paint symptoms.
    Backends that do not expose their internals return None from `peek_state`,
    and the fault layer degrades to sensor-boundary injection. See ADR-002.
    """

    robot_id: str
    bearing_wear: float
    thermal_mass_c: float
    duty_cycle: float
    nominal_sigma: dict[str, float]
    """Per-channel nominal sensor noise sigma. Required by the ground-truth
    layer to express detectability in SNR units rather than raw engineering
    units, which is what makes thresholds comparable across channels."""


class FleetSimulator(Protocol):
    @property
    def robot_ids(self) -> Sequence[str]: ...

    def reset(self, seed: int) -> None:
        """Full determinism from `seed`. CI gates are meaningless without it."""

    def step(self, dt_s: float) -> Sequence[TelemetrySample]:
        """Advance the world and return this tick's samples for the whole fleet."""

    def peek_state(self, robot_id: str) -> RobotState | None:
        """Latent state, or None for backends that cannot expose it."""
