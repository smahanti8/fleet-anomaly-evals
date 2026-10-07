from __future__ import annotations

import math
import statistics

from fleet_evals.sim.reference import ReferenceSimulator, SimulatorConfig


def _noise_sigma(samples: list[float]) -> float:
    # First differences cancel any slow underlying trend (battery drain,
    # thermal relaxation) and leave iid noise, whose variance doubles under
    # subtraction -- hence the sqrt(2) correction.
    diffs = [b - a for a, b in zip(samples, samples[1:])]
    return statistics.pstdev(diffs) / math.sqrt(2)

ROBOT_IDS = [f"robot-{i}" for i in range(10)]


def _run(seed: int, ticks: int, dt_s: float = 1.0):
    sim = ReferenceSimulator(ROBOT_IDS)
    sim.reset(seed)
    return [sim.step(dt_s) for _ in range(ticks)]


def test_reset_seed_determinism():
    run_a = _run(seed=42, ticks=200)
    run_b = _run(seed=42, ticks=200)
    assert run_a == run_b


def test_different_seed_differs():
    run_a = _run(seed=42, ticks=50)
    run_b = _run(seed=43, ticks=50)
    assert run_a != run_b


def test_battery_soc_noise_level():
    cfg = SimulatorConfig()
    sim = ReferenceSimulator(ROBOT_IDS, cfg)
    sim.reset(seed=7)
    # Warm up past the t=0 ceiling clip at soc=1.0 so noise isn't truncated.
    for _ in range(500):
        sim.step(1.0)
    samples = [sim.step(1.0)[0].battery_soc for _ in range(2000)]
    sigma = _noise_sigma(samples)
    assert abs(sigma - cfg.nominal_sigma["battery_soc"]) < 0.15 * cfg.nominal_sigma["battery_soc"]


def test_motor_temp_noise_level():
    cfg = SimulatorConfig()
    sim = ReferenceSimulator(ROBOT_IDS, cfg)
    sim.reset(seed=7)
    # Warm up near the thermal steady state so relaxation drift is negligible
    # next to the noise (thermal_mass_c is the relaxation time constant).
    for _ in range(int(5 * cfg.thermal_mass_c)):
        sim.step(1.0)
    samples = [sim.step(1.0)[0].motor_temp_c for _ in range(2000)]
    sigma = _noise_sigma(samples)
    assert abs(sigma - cfg.nominal_sigma["motor_temp_c"]) < 0.15 * cfg.nominal_sigma["motor_temp_c"]


def test_channel_ranges():
    cfg = SimulatorConfig(num_joints=4)
    sim = ReferenceSimulator(ROBOT_IDS, cfg)
    sim.reset(seed=1)
    prev_cycle_counts = {rid: 0 for rid in ROBOT_IDS}
    for _ in range(2000):
        tick = sim.step(1.0)
        for sample in tick:
            assert 0.0 <= sample.battery_soc <= 1.0
            assert sample.vibration_rms_g >= 0.0
            assert len(sample.joint_torque_nm) == cfg.num_joints
            assert sample.cycle_count >= prev_cycle_counts[sample.robot_id]
            prev_cycle_counts[sample.robot_id] = sample.cycle_count


def test_peek_state_reflects_fixed_duty_cycle():
    sim = ReferenceSimulator(ROBOT_IDS)
    sim.reset(seed=5)
    before = sim.peek_state(ROBOT_IDS[0]).duty_cycle
    sim.step(1.0)
    after = sim.peek_state(ROBOT_IDS[0]).duty_cycle
    assert before == after


def test_peek_state_unknown_robot_returns_none():
    sim = ReferenceSimulator(ROBOT_IDS)
    sim.reset(seed=5)
    assert sim.peek_state("not-a-robot") is None
