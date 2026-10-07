from __future__ import annotations

from fleet_evals.faults.ground_truth import FaultLedger, FaultType, InjectionSite
from fleet_evals.faults.thermal import ThermalRunawayInjector
from fleet_evals.faults.wiring import record_fault_event


def _thermal_injector(make_fault_spec, **spec_overrides):
    defaults = dict(
        fault_type=FaultType.THERMAL_RUNAWAY,
        injection_t_s=10.0,
        duration_s=None,
        params={"duty_cycle_boost": 0.3, "thermal_mass_scale": 0.5},
    )
    defaults.update(spec_overrides)
    return ThermalRunawayInjector(
        spec=make_fault_spec(**defaults), ambient_c=22.0, temp_gain_c_per_duty=55.0
    )


def test_onset_recorded_exactly(make_fault_spec):
    injector = _thermal_injector(make_fault_spec, injection_t_s=17.5)
    ledger = FaultLedger(events=[])
    event = record_fault_event(
        injector,
        fault_id="f0",
        injection_site=InjectionSite.LATENT_STATE,
        ledger=ledger,
        snr_traces={"motor_temp_c": [(20.0, 4.0), (30.0, 4.0), (40.0, 4.0)]},
        thresholds=[3.0],
        sustained_s=10.0,
    )
    assert event.injection_t_s == 17.5
    assert event.injection_t_s == injector.spec.injection_t_s
    assert ledger.events == [event]


def test_reproducibility_given_identical_inputs(make_fault_spec):
    trace = {"motor_temp_c": [(20.0, 4.0), (30.0, 4.0), (40.0, 4.0)]}

    injector_a = _thermal_injector(make_fault_spec)
    ledger_a = FaultLedger(events=[])
    event_a = record_fault_event(
        injector_a, "f0", InjectionSite.LATENT_STATE, ledger_a, trace, [3.0], sustained_s=10.0
    )

    injector_b = _thermal_injector(make_fault_spec)
    ledger_b = FaultLedger(events=[])
    event_b = record_fault_event(
        injector_b, "f0", InjectionSite.LATENT_STATE, ledger_b, trace, [3.0], sustained_s=10.0
    )

    assert event_a == event_b


def test_never_detectable_fault_still_recorded(make_fault_spec):
    injector = _thermal_injector(make_fault_spec)
    ledger = FaultLedger(events=[])
    event = record_fault_event(
        injector,
        fault_id="f0",
        injection_site=InjectionSite.LATENT_STATE,
        ledger=ledger,
        snr_traces={"motor_temp_c": [(20.0, 0.1), (30.0, 0.1), (40.0, 0.1)]},
        thresholds=[3.0],
        sustained_s=10.0,
    )
    assert event.detectability == ()
    assert ledger.events == [event]  # present, not dropped


def test_missing_channel_trace_treated_as_empty(make_fault_spec):
    injector = _thermal_injector(make_fault_spec)
    ledger = FaultLedger(events=[])
    event = record_fault_event(
        injector,
        fault_id="f0",
        injection_site=InjectionSite.LATENT_STATE,
        ledger=ledger,
        snr_traces={},
        thresholds=[3.0],
        sustained_s=10.0,
    )
    assert event.detectability == ()


def test_end_t_s_derived_from_duration(make_fault_spec):
    injector = _thermal_injector(make_fault_spec, injection_t_s=10.0, duration_s=200.0)
    ledger = FaultLedger(events=[])
    event = record_fault_event(
        injector, "f0", InjectionSite.LATENT_STATE, ledger, {}, [3.0], sustained_s=10.0
    )
    assert event.end_t_s == 210.0


def test_end_t_s_none_when_duration_none(make_fault_spec):
    injector = _thermal_injector(make_fault_spec, duration_s=None)
    ledger = FaultLedger(events=[])
    event = record_fault_event(
        injector, "f0", InjectionSite.LATENT_STATE, ledger, {}, [3.0], sustained_s=10.0
    )
    assert event.end_t_s is None
