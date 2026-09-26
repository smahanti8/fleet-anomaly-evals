# ADR-003: How detections are matched to fault events

Status: draft — to be written with the evaluation harness
Date: 2026-09-14

## Context

Precision, recall and the confusion matrix all depend on a rule that decides
when a detection "is" a given fault. This rule, not the detector, is what most
fleet-monitoring demos leave undefined — and it is the single largest lever on
the published numbers.

## To be decided and recorded

- **Matching window.** A detection on the correct robot counts as a true
  positive if it falls in `[injection_t, end_t + grace]`. What grace, and why.
- **Multiplicity.** One fault, many detections during its window: first
  detection is the true positive, the rest are suppressed rather than counted
  as false positives. Otherwise a correctly-firing detector accumulates
  thousands of "false" alarms on a fault it caught.
- **Attribution for the confusion matrix.** A detection is attributed to a fault
  type via the channel it fired on versus `FaultInjector.affected_channels()`.
  Overlapping faults on one robot need a documented tie-break.
- **False positive denominator.** FP per robot-hour must be computed over
  fault-free robot-hours only. Including fault windows in the denominator
  dilutes the rate with time where firing was correct.
- **Comms-loss interaction.** Detections are impossible during a comms gap.
  Gap hours are excluded from the FP denominator and counted explicitly, so a
  detector cannot earn a good false-positive rate by being blind.

## Note

Whatever is decided here goes in the README next to the numbers it produced. The
matching rule is a result, not an implementation detail.
