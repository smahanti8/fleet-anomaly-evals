from __future__ import annotations

import pytest

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.thermal import ThermalRunawayInjector


def _injector(make_fault_spec, **spec_overrides):
    spec = make_fault_spec(
        fault_type=FaultType.THERMAL_RUNAWAY,
        injection_t_s=10.0,
        duration_s=None,
        params={"duty_cycle_boost": 0.3, "thermal_mass_scale": 0.5},
        **spec_overrides,
    )
    return ThermalRunawayInjector(spec=spec, ambient_c=22.0, temp_gain_c_per_duty=55.0)


def test_wrong_fault_type_rejected(make_fault_spec):
    spec = make_fault_spec(fault_type=FaultType.SENSOR_DRIFT)
    with pytest.raises(ValueError):
        ThermalRunawayInjector(spec=spec, ambient_c=22.0, temp_gain_c_per_duty=55.0)


def test_affected_channels(make_fault_spec):
    injector = _injector(make_fault_spec)
    assert injector.affected_channels() == ("motor_temp_c",)


def test_on_state_identity_before_injection(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(duty_cycle=0.5)
    assert injector.on_state(state, t_sim_s=5.0) == state


def test_on_state_perturbs_after_injection(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(duty_cycle=0.5, thermal_mass_c=900.0)
    faulted = injector.on_state(state, t_sim_s=10.0)
    assert faulted.duty_cycle == pytest.approx(0.8)
    assert faulted.thermal_mass_c == pytest.approx(450.0)


def test_on_state_clips_duty_cycle_at_one(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(duty_cycle=0.9)
    faulted = injector.on_state(state, t_sim_s=10.0)
    assert faulted.duty_cycle == 1.0


def test_on_telemetry_is_identity(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)
    sample = make_telemetry_sample()
    assert injector.on_telemetry(sample, t_sim_s=99.0) == sample


def test_snr_zero_before_injection(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(duty_cycle=0.5, nominal_sigma={"motor_temp_c": 0.5})
    assert injector.expected_signal_snr(state, t_sim_s=9.99, channel="motor_temp_c") == 0.0


def test_snr_zero_for_other_channels(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(nominal_sigma={"motor_temp_c": 0.5, "vibration_rms_g": 0.02})
    assert injector.expected_signal_snr(state, t_sim_s=50.0, channel="vibration_rms_g") == 0.0


def test_snr_grows_toward_asymptote(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(duty_cycle=0.5, thermal_mass_c=900.0, nominal_sigma={"motor_temp_c": 0.5})
    early = injector.expected_signal_snr(state, t_sim_s=10.1, channel="motor_temp_c")
    later = injector.expected_signal_snr(state, t_sim_s=500.0, channel="motor_temp_c")
    much_later = injector.expected_signal_snr(state, t_sim_s=100_000.0, channel="motor_temp_c")
    asymptote = (0.8 * 55.0 - 0.5 * 55.0) / 0.5
    assert 0.0 < early < later < much_later
    assert much_later == pytest.approx(asymptote, rel=1e-3)


def test_expected_signal_snr_is_deterministic(make_fault_spec, make_robot_state):
    injector_a = _injector(make_fault_spec)
    injector_b = _injector(make_fault_spec)
    state = make_robot_state(duty_cycle=0.5, thermal_mass_c=900.0, nominal_sigma={"motor_temp_c": 0.5})
    assert injector_a.expected_signal_snr(
        state, t_sim_s=42.0, channel="motor_temp_c"
    ) == injector_b.expected_signal_snr(state, t_sim_s=42.0, channel="motor_temp_c")
