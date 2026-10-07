# fleet-anomaly-evals

An evaluation harness for anomaly detection on a robot fleet, scored against labeled ground truth.

> **Status: early.** The simulator boundary, a physics-lite reference simulator, three of the four fault injectors, the ground-truth ledger wiring and the detectability-crossing reducer exist and are tested. The ingestion layer, detectors, evaluation harness and CI do not exist yet, and nothing connects simulator, injector, ledger and detector into one run. This README makes no claim about results that do not exist.

## The idea

An anomaly detector that acts on a fleet has to be evaluated against known ground truth before anyone trusts it. Most fleet-monitoring demos show a detector firing on obvious faults. This repository is being built to measure whether a detector catches the faults that matter, how long after they begin it does so, and how often it fires when nothing is wrong.

The central design decision is already recorded ([ADR-002](docs/ADR-002-fault-onset.md)): ground truth is the time a fault was **injected**. When it first became **detectable** is logged alongside it, never in its place. That separation is what will let detection latency be split into how long the fault was invisible and how long the detector took after it became visible.

## What exists

| Path | What it is |
|---|---|
| `src/fleet_evals/sim/interface.py` | The simulator boundary: a protocol thin enough that a physics-lite simulator, Gazebo, Isaac Sim or recorded fleet data could sit behind it |
| `src/fleet_evals/sim/reference.py` | A seeded, reproducible physics-lite backend: battery, motor temperature, joint torque, vibration and a cycle counter, with per-channel noise |
| `src/fleet_evals/faults/ground_truth.py` | The labels: fault events, the ledger that stores them, detectability crossings and latency decomposition |
| `src/fleet_evals/faults/interface.py` | The fault-injection boundary |
| `src/fleet_evals/faults/ground_truth_builder.py` | Reduces an expected-SNR trace to its first sustained crossing at each threshold, as ADR-002 defines it |
| `src/fleet_evals/faults/thermal.py`, `drift.py`, `comms.py` | Three injectors: motor thermal runaway (a latent-state fault), sensor drift (a sensor-boundary fault) and comms loss (a stale-payload gap, which has no SNR curve) |
| `src/fleet_evals/faults/wiring.py` | Writes a fault's true onset to the ledger from pre-sampled SNR traces |
| `src/fleet_evals/faults/bearing.py` | Bearing degradation over a ~200 h horizon (unfinished, see below) |
| `tests/` | Unit tests for the pieces above, including a check that the thermal injector's expected SNR matches what the simulator actually produces when the test applies the injector by hand |
| `docs/` | Three design records: what "true onset" means (accepted), what stays out of the telemetry store, and how detections are matched to faults (both drafts) |

## Deliberately unfinished

`bearing.wear_fraction` raises `NotImplementedError`, so there is no bearing injector yet. The shape of the wear curve decides whether the evaluation ever exercises slow degradation, so it is left as an explicit design decision rather than a guess.

Also not built: the simulator does not drive the injectors itself (a caller applies them), ingestion, detectors, the evaluation harness and CI. The two draft design records will be completed with the ingestion layer and the evaluation harness.

## Tests

Run `pytest` from the repository root. pytest is declared as the `dev` extra; the package itself has no runtime dependencies.

## Limits

This will be a simulated fleet with injected faults. That is what makes the evaluation meaningful, because the truth is known, and it is also the limitation: real fleets have failure modes nobody anticipated or labeled.

## License

MIT. See [LICENSE](LICENSE).
