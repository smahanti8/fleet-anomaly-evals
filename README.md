# fleet-anomaly-evals

An evaluation harness for anomaly detection on a robot fleet, scored against labeled ground truth.

> **Status: early.** So far this repository contains interfaces and design records only. The simulator, fault injection, ingestion layer, detector and evaluation harness are being added one commit at a time, and this README makes no claim about results that do not exist yet.

## The idea

An anomaly detector that acts on a fleet has to be evaluated against known ground truth before anyone trusts it. Most fleet-monitoring demos show a detector firing on obvious faults. This repository is being built to measure whether a detector catches the faults that matter, how long after they begin it does so, and how often it fires when nothing is wrong.

The central design decision is already recorded ([ADR-002](docs/ADR-002-fault-onset.md)): ground truth is the time a fault was **injected**. When it first became **detectable** is logged alongside it, never in its place. That separation is what will let detection latency be split into how long the fault was invisible and how long the detector took after it became visible.

## What exists

| Path | What it is |
|---|---|
| `src/fleet_evals/sim/interface.py` | The simulator boundary: a protocol thin enough that a physics-lite simulator, Gazebo, Isaac Sim or recorded fleet data could sit behind it |
| `src/fleet_evals/faults/ground_truth.py` | The labels: fault events, the ledger that stores them, detectability crossings and latency decomposition |
| `src/fleet_evals/faults/interface.py` | The fault-injection boundary |
| `src/fleet_evals/faults/bearing.py` | Bearing degradation over a ~200 h horizon (unfinished, see below) |
| `docs/` | Three design records: what "true onset" means (accepted), what stays out of the telemetry store, and how detections are matched to faults (both drafts) |

## Deliberately unfinished

`bearing.wear_fraction` and `crossings_from_snr_trace` raise `NotImplementedError`. The shape of the wear curve decides whether the evaluation ever exercises slow degradation, so it is left as an explicit design decision rather than a guess.

## Limits

This will be a simulated fleet with injected faults. That is what makes the evaluation meaningful, because the truth is known, and it is also the limitation: real fleets have failure modes nobody anticipated or labeled.

## License

MIT. See [LICENSE](LICENSE).
