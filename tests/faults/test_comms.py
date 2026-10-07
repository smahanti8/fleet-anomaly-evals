from __future__ import annotations

import pytest

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.comms import CommsLossInjector


def _injector(make_fault_spec, **spec_overrides):
    defaults = dict(
        fault_type=FaultType.COMMS_LOSS,
        injection_t_s=10.0,
        duration_s=None,
        params={"period_s": 10.0, "gap_duration_s": 3.0},
    )
    defaults.update(spec_overrides)
    return CommsLossInjector(spec=make_fault_spec(**defaults))


def test_wrong_fault_type_rejected(make_fault_spec):
    spec = make_fault_spec(fault_type=FaultType.SENSOR_DRIFT, params={"period_s": 1.0, "gap_duration_s": 1.0})
    with pytest.raises(ValueError):
        CommsLossInjector(spec=spec)


def test_affected_channels_empty(make_fault_spec):
    injector = _injector(make_fault_spec)
    assert injector.affected_channels() == ()


def test_snr_always_zero(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state()
    assert injector.expected_signal_snr(state, t_sim_s=1000.0, channel="motor_temp_c") == 0.0


def test_on_state_is_identity(make_fault_spec, make_robot_state):
    injector = _injector(make_fault_spec)
    state = make_robot_state()
    assert injector.on_state(state, t_sim_s=999.0) is state


def test_passthrough_before_injection(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)
    sample = make_telemetry_sample(t_sim_s=5.0, comms_ok=True)
    assert injector.on_telemetry(sample, t_sim_s=5.0) == sample


def test_gap_freezes_last_good_payload(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)  # period=10, gap=3, starting at t=10
    good = make_telemetry_sample(t_sim_s=9.0, battery_soc=0.77, motor_temp_c=41.0, comms_ok=True)
    injector.on_telemetry(good, t_sim_s=9.0)

    during_gap_sample = make_telemetry_sample(t_sim_s=11.0, battery_soc=0.5, motor_temp_c=99.0)
    result = injector.on_telemetry(during_gap_sample, t_sim_s=11.0)

    assert result.comms_ok is False
    assert result.battery_soc == 0.77
    assert result.motor_temp_c == 41.0
    assert result.t_sim_s == 11.0
    assert result.robot_id == during_gap_sample.robot_id


def test_recovers_after_gap(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)  # gap is [10, 13); 14 should be live again
    good = make_telemetry_sample(t_sim_s=9.0, battery_soc=0.77, comms_ok=True)
    injector.on_telemetry(good, t_sim_s=9.0)
    injector.on_telemetry(make_telemetry_sample(t_sim_s=11.0, battery_soc=0.5), t_sim_s=11.0)

    live = make_telemetry_sample(t_sim_s=14.0, battery_soc=0.4, comms_ok=True)
    result = injector.on_telemetry(live, t_sim_s=14.0)
    assert result == live


def test_gap_recurs_periodically(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)  # period=10, gap=3 -> gaps at [10,13), [20,23), ...
    good = make_telemetry_sample(t_sim_s=0.0, battery_soc=0.9, comms_ok=True)
    injector.on_telemetry(good, t_sim_s=0.0)
    injector.on_telemetry(make_telemetry_sample(t_sim_s=15.0, battery_soc=0.6, comms_ok=True), t_sim_s=15.0)

    during_second_gap = make_telemetry_sample(t_sim_s=21.0, battery_soc=0.3)
    result = injector.on_telemetry(during_second_gap, t_sim_s=21.0)
    assert result.comms_ok is False
    assert result.battery_soc == 0.6


def test_gap_before_any_good_sample_passes_through(make_fault_spec, make_telemetry_sample):
    injector = _injector(make_fault_spec)
    first_ever = make_telemetry_sample(t_sim_s=11.0, battery_soc=0.5)
    result = injector.on_telemetry(first_ever, t_sim_s=11.0)
    assert result == first_ever
