from __future__ import annotations

import pytest

from fleet_evals.faults.ground_truth import FaultEvent, FaultType, InjectionSite
from fleet_evals.faults.interface import FaultSpec
from fleet_evals.sim.interface import RobotState, TelemetrySample


@pytest.fixture
def make_robot_state():
    def _make(**overrides) -> RobotState:
        defaults = dict(
            robot_id="robot-0",
            bearing_wear=0.0,
            thermal_mass_c=12.0,
            duty_cycle=0.5,
            nominal_sigma={"motor_temp_c": 0.5, "vibration_rms_g": 0.02},
        )
        defaults.update(overrides)
        return RobotState(**defaults)

    return _make


@pytest.fixture
def make_telemetry_sample():
    def _make(**overrides) -> TelemetrySample:
        defaults = dict(
            robot_id="robot-0",
            t_sim_s=0.0,
            battery_soc=1.0,
            motor_temp_c=25.0,
            joint_torque_nm=(1.0, 1.0, 1.0),
            vibration_rms_g=0.01,
            cycle_count=0,
            comms_ok=True,
        )
        defaults.update(overrides)
        return TelemetrySample(**defaults)

    return _make


@pytest.fixture
def make_fault_spec():
    def _make(**overrides) -> FaultSpec:
        defaults = dict(
            fault_type=FaultType.THERMAL_RUNAWAY,
            robot_id="robot-0",
            injection_t_s=10.0,
            duration_s=None,
            params={"duty_cycle_boost": 0.3},
        )
        defaults.update(overrides)
        return FaultSpec(**defaults)

    return _make


@pytest.fixture
def make_fault_event():
    def _make(**overrides) -> FaultEvent:
        defaults = dict(
            fault_id="fault-0",
            robot_id="robot-0",
            fault_type=FaultType.THERMAL_RUNAWAY,
            injection_site=InjectionSite.LATENT_STATE,
            injection_t_s=10.0,
            end_t_s=None,
            params={"duty_cycle_boost": 0.3},
            detectability=(),
        )
        defaults.update(overrides)
        return FaultEvent(**defaults)

    return _make
