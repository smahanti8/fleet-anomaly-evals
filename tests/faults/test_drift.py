from __future__ import annotations

import pytest

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.drift import SensorDriftInjector


def _injector(make_fault_spec, **spec_overrides):
    defaults = dict(
        fault_type=FaultType.SENSOR_DRIFT,
        injection_t_s=10.0,
        duration_s=None,
        params={"channel": "battery_soc", "drift_rate_per_s": 0.001},
    )
    defaults.update(spec_overrides)
    return SensorDriftInjector(spec=make_fault_spec(**defaults))


def test_wrong_fault_type_rejected(make_fault_spec):
    spec = make_fault_spec(fault_type=FaultType.THERMAL_RUNAWAY, params={"channel": "battery_soc"})
    with pytest.raises(ValueError):
        SensorDriftInjector(spec=spec)


def test_unknown_channel_rejected(make_fault_spec):
    spec = make_fault_spec(
        fault_type=FaultType.SENSOR_DRIFT, params={"channel": "joint_torque_nm"}
    )
    with pytest.raises(ValueError):
        SensorDriftInjector(spec=spec)


def test_affected_channels(make_fault_spec):
    injector = _injector(make_fault_spec)
    assert injector.affected_channels() == ("battery_soc",)


def test_on_state_is_identity(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state()
    assert injector.on_state(state, t_sim_s=999.0) is state


def test_on_telemetry_identity_before_injection(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)
    sample = make_telemetry_sample(battery_soc=0.9)
    assert injector.on_telemetry(sample, t_sim_s=9.99) == sample


def test_on_telemetry_adds_growing_bias(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)
    sample = make_telemetry_sample(battery_soc=0.9)
    at_injection = injector.on_telemetry(sample, t_sim_s=10.0).battery_soc
    later = injector.on_telemetry(sample, t_sim_s=110.0).battery_soc
    assert at_injection == pytest.approx(0.9)
    assert later == pytest.approx(0.9 + 0.001 * 100.0)


def test_on_telemetry_only_touches_target_field(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)
    sample = make_telemetry_sample(battery_soc=0.9, motor_temp_c=30.0)
    drifted = injector.on_telemetry(sample, t_sim_s=110.0)
    assert drifted.motor_temp_c == 30.0


def test_snr_zero_for_other_channels(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(nominal_sigma={"battery_soc": 0.002, "motor_temp_c": 0.5})
    assert injector.expected_signal_snr(state, t_sim_s=110.0, channel="motor_temp_c") == 0.0


def test_snr_is_exact_bias_over_sigma(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state(nominal_sigma={"battery_soc": 0.002})
    snr = injector.expected_signal_snr(state, t_sim_s=110.0, channel="battery_soc")
    assert snr == pytest.approx((0.001 * 100.0) / 0.002)


def test_duration_bounded_drift_stops_after_end(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec, duration_s=20.0)
    sample = make_telemetry_sample(battery_soc=0.9)
    within = injector.on_telemetry(sample, t_sim_s=25.0).battery_soc
    after = injector.on_telemetry(sample, t_sim_s=35.0).battery_soc
    assert within != 0.9
    assert after == 0.9
