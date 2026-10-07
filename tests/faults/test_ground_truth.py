from __future__ import annotations

import pytest


def test_fault_event_params_key_assignment_raises(make_fault_event):
    event = make_fault_event()
    with pytest.raises(TypeError):
        event.params["duty_cycle_boost"] = 999.0


def test_fault_event_params_defensive_copy(make_fault_event):
    original = {"duty_cycle_boost": 0.3}
    event = make_fault_event(params=original)
    original["duty_cycle_boost"] = 999.0
    assert event.params["duty_cycle_boost"] == 0.3


def test_fault_event_params_equality_and_repr(make_fault_event):
    a = make_fault_event(params={"duty_cycle_boost": 0.3})
    b = make_fault_event(params={"duty_cycle_boost": 0.3})
    assert a == b
    assert "duty_cycle_boost" in repr(a)
