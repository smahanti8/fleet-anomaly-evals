"""Cross-checks ThermalRunawayInjector.expected_signal_snr against the
ReferenceSimulator's own dynamics.

The injectors are not wired into ReferenceSimulator (nothing in step() calls
on_state), so this test drives the perturbation itself: it runs two otherwise
identical simulators from the same seed, and before each post-injection tick
pokes the injector's perturbed RobotState directly into the private per-robot
runtime -- the only place ReferenceSimulator reads duty_cycle from. There is
no public seam for this yet; that absence is what the test exercises, not a
gap this test fixes.

Because duty_cycle only shifts the deterministic steady-state target (not
which, or how many, rng.gauss() calls happen per tick), the clean and faulted
runs draw bit-identical noise at every tick. Subtracting the two motor_temp_c
readings therefore cancels the noise exactly and isolates the injected mean
deviation, instead of needing many trials to average it out.
"""

from __future__ import annotations

import pytest

from fleet_evals.faults.ground_truth import FaultType
from fleet_evals.faults.interface import FaultSpec
from fleet_evals.faults.thermal import ThermalRunawayInjector
from fleet_evals.sim.reference import ReferenceSimulator, SimulatorConfig

ROBOT_ID = "robot-0"
SEED = 123

# Warm-up long enough that the residual gap to steady state is negligible next
# to nominal_sigma: exp(-10) times the duty-driven gap is a few thousandths of
# a degree, against a 0.4 C sigma.
WARM_UP_TIME_CONSTANTS = 10

# Ticks after injection at which observed vs. expected SNR is compared,
# spanning early ramp-up through near the asymptote (~4 time constants).
CHECK_ELAPSED_S = (50.0, 100.0, 300.0, 900.0, 1800.0, 3600.0)

# expected_signal_snr models continuous exponential relaxation; the simulator
# takes discrete forward-Euler steps. Their fractional mismatch grows with
# elapsed / thermal_mass_c but stays under ~0.5% over the horizons checked
# here, which for an asymptotic SNR in the tens of sigma is a few hundredths
# of an SNR unit -- the residual warm-up gap contributes far less than that.
# 0.05 is roughly 5x the largest discrepancy observed while writing this
# test, leaving margin without hiding a real regression.
SNR_TOLERANCE = 0.05


@pytest.mark.parametrize(
    "params",
    [
        {"duty_cycle_boost": 0.3},
        # Regression test for a real gap: step() used the fleet-wide
        # cfg.thermal_mass_c, so this per-robot mass perturbation had no effect
        # on the simulation and the injector's model disagreed with it.
        {"duty_cycle_boost": 0.3, "thermal_mass_scale": 0.5},
    ],
    ids=["duty_boost_only", "duty_boost_and_thermal_mass_scale"],
)
def test_observed_motor_temp_snr_matches_injector_model(params):
    cfg = SimulatorConfig()
    dt_s = 1.0
    warm_up_ticks = int(WARM_UP_TIME_CONSTANTS * cfg.thermal_mass_c)

    clean_sim = ReferenceSimulator([ROBOT_ID], cfg)
    clean_sim.reset(SEED)
    faulted_sim = ReferenceSimulator([ROBOT_ID], cfg)
    faulted_sim.reset(SEED)

    for _ in range(warm_up_ticks):
        clean_sim.step(dt_s)
        faulted_sim.step(dt_s)

    # duty_cycle is fixed per robot for the whole run (see sim/reference.py),
    # so this single post-warm-up snapshot is the correct "no fault" baseline
    # to feed on_state at every later tick.
    nominal_state = faulted_sim.peek_state(ROBOT_ID)
    injection_t_s = warm_up_ticks * dt_s

    spec = FaultSpec(
        fault_type=FaultType.THERMAL_RUNAWAY,
        robot_id=ROBOT_ID,
        injection_t_s=injection_t_s,
        duration_s=None,
        params=params,
    )
    injector = ThermalRunawayInjector(
        spec=spec, ambient_c=cfg.ambient_c, temp_gain_c_per_duty=cfg.temp_gain_c_per_duty
    )

    max_elapsed = int(max(CHECK_ELAPSED_S))
    observed_snr_at_elapsed = {}
    for i in range(1, max_elapsed + 1):
        t_sim_s = injection_t_s + i * dt_s

        faulted_runtime = faulted_sim._runtimes[ROBOT_ID]
        faulted_runtime.state = injector.on_state(nominal_state, t_sim_s=t_sim_s)

        clean_sample = clean_sim.step(dt_s)[0]
        faulted_sample = faulted_sim.step(dt_s)[0]

        if float(i) in CHECK_ELAPSED_S:
            observed_snr_at_elapsed[i] = (
                faulted_sample.motor_temp_c - clean_sample.motor_temp_c
            ) / cfg.nominal_sigma["motor_temp_c"]

    disagreements = []
    for elapsed in CHECK_ELAPSED_S:
        t_sim_s = injection_t_s + elapsed
        expected = injector.expected_signal_snr(
            nominal_state, t_sim_s=t_sim_s, channel="motor_temp_c"
        )
        observed = observed_snr_at_elapsed[int(elapsed)]
        if abs(observed - expected) > SNR_TOLERANCE:
            disagreements.append((elapsed, observed, expected, observed - expected))

    assert not disagreements, (
        "observed vs. expected motor_temp_c SNR disagree beyond tolerance "
        f"({SNR_TOLERANCE}): elapsed_s, observed, expected, diff = {disagreements}"
    )
