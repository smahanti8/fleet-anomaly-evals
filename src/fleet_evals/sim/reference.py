"""Physics-lite reference backend for the FleetSimulator protocol.

Each channel model is deliberately closed-form or single-step-Euler: enough to
give a detector real per-robot dynamics and configurable noise, not a claim
about real robot physics. `duty_cycle` is fixed per robot for the run rather
than time-varying, since nothing in this session's scope needs it to move.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Sequence

from fleet_evals.sim.interface import RobotState, TelemetrySample


@dataclass(frozen=True, slots=True)
class SimulatorConfig:
    """Fleet-wide constants. Same nominal sigma and duty-cycle range for every
    robot in this reference backend; per-robot variation is not modeled here."""

    num_joints: int = 6
    ambient_c: float = 22.0
    thermal_mass_c: float = 900.0
    """Effective thermal time constant in seconds, not a literal mass."""
    temp_gain_c_per_duty: float = 55.0
    battery_drain_per_hour_at_full_duty: float = 0.06
    torque_baseline_nm: float = 4.0
    torque_gain_nm: float = 6.0
    vibration_baseline_g: float = 0.05
    vibration_gain_g: float = 0.03
    cycle_period_s: float = 20.0
    duty_cycle_range: tuple[float, float] = (0.3, 0.9)
    nominal_sigma: dict[str, float] = field(
        default_factory=lambda: {
            "battery_soc": 0.002,
            "motor_temp_c": 0.4,
            "joint_torque_nm": 0.15,
            "vibration_rms_g": 0.01,
        }
    )


@dataclass(slots=True)
class _RobotRuntime:
    """Mutable per-robot simulation state. Not part of the public protocol —
    RobotState is the read-only view handed out via peek_state."""

    state: RobotState
    rng: random.Random
    t_sim_s: float
    battery_soc: float
    motor_temp_c: float
    cycle_accumulator_s: float
    cycle_count: int


class ReferenceSimulator:
    """FleetSimulator implementation. Nothing downstream may read which
    backend it is; only `reset`, `step`, `peek_state` and `robot_ids` are used."""

    def __init__(self, robot_ids: Sequence[str], config: SimulatorConfig | None = None) -> None:
        self._robot_ids = tuple(robot_ids)
        self._config = config or SimulatorConfig()
        self._runtimes: dict[str, _RobotRuntime] = {}

    @property
    def robot_ids(self) -> Sequence[str]:
        return self._robot_ids

    def reset(self, seed: int) -> None:
        # One master RNG derives one seed per robot, in robot_ids order, so
        # the whole fleet trajectory is fixed by `seed` alone, while each
        # robot's own noise stream is independent of how many robots there are.
        master = random.Random(seed)
        cfg = self._config
        self._runtimes = {}
        for robot_id in self._robot_ids:
            robot_seed = master.randrange(2**32)
            rng = random.Random(robot_seed)
            duty_cycle = rng.uniform(*cfg.duty_cycle_range)
            state = RobotState(
                robot_id=robot_id,
                bearing_wear=0.0,
                thermal_mass_c=cfg.thermal_mass_c,
                duty_cycle=duty_cycle,
                nominal_sigma=cfg.nominal_sigma,
            )
            self._runtimes[robot_id] = _RobotRuntime(
                state=state,
                rng=rng,
                t_sim_s=0.0,
                battery_soc=1.0,
                motor_temp_c=cfg.ambient_c,
                cycle_accumulator_s=0.0,
                cycle_count=0,
            )

    def peek_state(self, robot_id: str) -> RobotState | None:
        runtime = self._runtimes.get(robot_id)
        return runtime.state if runtime is not None else None

    def step(self, dt_s: float) -> Sequence[TelemetrySample]:
        cfg = self._config
        samples = []
        for robot_id in self._robot_ids:
            rt = self._runtimes[robot_id]
            rt.t_sim_s += dt_s
            duty = rt.state.duty_cycle

            # Battery: linear drain scaled by duty cycle and elapsed hours.
            drain = cfg.battery_drain_per_hour_at_full_duty * duty * (dt_s / 3600.0)
            rt.battery_soc = max(0.0, rt.battery_soc - drain)
            battery_reading = _clip(
                rt.battery_soc + rt.rng.gauss(0.0, cfg.nominal_sigma["battery_soc"]), 0.0, 1.0
            )

            # Motor temp: first-order Euler relaxation toward a duty-driven
            # steady state, thermal_mass_c acting as the time constant.
            t_steady = cfg.ambient_c + duty * cfg.temp_gain_c_per_duty
            rt.motor_temp_c += (dt_s / rt.state.thermal_mass_c) * (t_steady - rt.motor_temp_c)
            motor_temp_reading = rt.motor_temp_c + rt.rng.gauss(0.0, cfg.nominal_sigma["motor_temp_c"])

            # Joint torque: duty-scaled baseline, independent noise per joint.
            torque_mean = cfg.torque_baseline_nm + duty * cfg.torque_gain_nm
            joint_torque = tuple(
                torque_mean + rt.rng.gauss(0.0, cfg.nominal_sigma["joint_torque_nm"])
                for _ in range(cfg.num_joints)
            )

            # Vibration RMS: duty-scaled baseline, floored at 0 since an RMS
            # value cannot be negative.
            vibration_mean = cfg.vibration_baseline_g + duty * cfg.vibration_gain_g
            vibration_reading = max(
                0.0, vibration_mean + rt.rng.gauss(0.0, cfg.nominal_sigma["vibration_rms_g"])
            )

            # Cycle count: deterministic accumulator, no noise -- a duty cycle
            # either completed a full period or it did not.
            rt.cycle_accumulator_s += dt_s * duty
            while rt.cycle_accumulator_s >= cfg.cycle_period_s:
                rt.cycle_accumulator_s -= cfg.cycle_period_s
                rt.cycle_count += 1

            samples.append(
                TelemetrySample(
                    robot_id=robot_id,
                    t_sim_s=rt.t_sim_s,
                    battery_soc=battery_reading,
                    motor_temp_c=motor_temp_reading,
                    joint_torque_nm=joint_torque,
                    vibration_rms_g=vibration_reading,
                    cycle_count=rt.cycle_count,
                    comms_ok=True,
                )
            )
        return tuple(samples)


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
