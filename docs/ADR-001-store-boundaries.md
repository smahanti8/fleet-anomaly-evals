# ADR-001: What does not go in the telemetry store

Status: draft — to be written with the ingestion layer
Date: 2026-09-14

## Context

Telemetry at 1 Hz from 50 robots over a 200 h horizon is ~36M samples before
channel fan-out. The store is a local DuckDB file. The interesting engineering
decision is not what to persist, it is what to refuse to persist.

## To be decided and recorded

- Raw high-rate waveform vs. the 1 Hz aggregate that the detector actually reads.
- Derived features and rolling statistics: recomputed from raw, or persisted.
- Detector scores and detections: same store as telemetry, or separate.
- **Ground-truth labels: separate store, no shared connection.** Already settled
  in ADR-002 and enforced by the module graph, but it is restated here because
  this is the file a reader checks for store layout.
- Full-fleet run vs. the scaled subset CI executes, and why the README states
  the split rather than hiding it in a workflow file.

## Open

Align the framing with the observation-count judgment already documented in
`surgical-fhir-pipeline` so the two repos use one vocabulary for the same call,
rather than inventing a second.
