from __future__ import annotations

import pytest


def test_fault_spec_params_key_assignment_raises(make_fault_spec):
    spec = make_fault_spec()
    with pytest.raises(TypeError):
        spec.params["duty_cycle_boost"] = 999.0


def test_fault_spec_params_defensive_copy(make_fault_spec):
    original = {"duty_cycle_boost": 0.3}
    spec = make_fault_spec(params=original)
    original["duty_cycle_boost"] = 999.0
    assert spec.params["duty_cycle_boost"] == 0.3


def test_fault_spec_params_equality_and_repr(make_fault_spec):
    a = make_fault_spec(params={"duty_cycle_boost": 0.3})
    b = make_fault_spec(params={"duty_cycle_boost": 0.3})
    assert a == b
    assert "duty_cycle_boost" in repr(a)
