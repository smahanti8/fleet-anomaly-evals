from __future__ import annotations

import pytest


def test_nominal_sigma_key_assignment_raises(make_robot_state):
    state = make_robot_state()
    with pytest.raises(TypeError):
        state.nominal_sigma["motor_temp_c"] = 999.0


def test_nominal_sigma_defensive_copy(make_robot_state):
    original = {"motor_temp_c": 0.5}
    state = make_robot_state(nominal_sigma=original)
    original["motor_temp_c"] = 999.0
    assert state.nominal_sigma["motor_temp_c"] == 0.5


def test_nominal_sigma_equality_and_repr(make_robot_state):
    a = make_robot_state(nominal_sigma={"motor_temp_c": 0.5})
    b = make_robot_state(nominal_sigma={"motor_temp_c": 0.5})
    assert a == b
    assert "motor_temp_c" in repr(a)
