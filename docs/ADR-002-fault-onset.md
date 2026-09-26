# ADR-002: What "true onset" means, and why detectability is not it

Status: accepted
Date: 2026-09-14

## Context

Every number this repository publishes is measured against injected fault
onsets. Recall uses onset as its denominator; detection latency uses it as its
zero point. So the definition of onset is not a detail — it determines whether
the results mean anything comparable.

There are two candidate definitions:

1. **Injection time** — the moment the fault was introduced into the world.
2. **Detectability time** — the moment the fault became observable in the
   telemetry.

## Decision

**Ground truth is injection time.** Detectability is recorded alongside it, and
never in place of it.

Injection time is the only quantity known without interpretation. Detectability
depends on the noise level, on what magnitude you are willing to call a signal,
and on which channels you happen to be watching. Defining ground truth that way
would mean defining the answer using assumptions borrowed from the thing being
measured, and recall would stop being comparable across detectors, across
configurations, or across runs of this repo.

Injection time keeps the denominator fixed and honest.

## Consequence: latency decomposes

With both quantities logged, detection latency splits into two parts:

| Component | Definition | Owned by |
|---|---|---|
| Signal-limited lag | injection → detectability crossing | the fault and the sensors |
| Algorithm lag | detectability crossing → detection | the detector |
| Total | injection → detection | operations |

The first is a physical property no algorithm can improve. The second is the
only part the detector controls. The third is what the fleet actually
experienced, and it is the number that belongs in an operational conversation.

This is the distinction that separates an evaluation from a demo. A detector
that fires 40 hours after injection reads as a failure — until you show that
bearing degradation was not observable above noise for the first 35 of them.
The defensible claim is then a 5-hour algorithm lag, and it can be proven rather
than asserted.

Algorithm lag is **not clamped at zero**. A negative value means the detector
fired while the fault was still below the single-channel SNR threshold, which is
a legitimate result achieved through cross-channel or temporal structure.
Clamping would convert the harness's most interesting finding into a boring one.

## Consequence: detectability is a curve, not a moment

For slow-degradation faults, "when did this become observable?" has no single
answer. The signal rises continuously through the noise floor, so the reported
time is entirely a function of where the threshold sits. A single detectability
timestamp would bury that choice inside a number that looks objective.

So the ground-truth log records **threshold crossings, not a timestamp**:
`DetectabilityCrossing(channel, snr_threshold, t_sim_s, sustained_s)`, evaluated
at every configured threshold. The published latency decomposition states which
threshold it is relative to; a latency figure quoted without that threshold is
not reproducible, and this repo will not print one.

A crossing must be sustained for `sustained_s` to count, so a single noisy tick
is not recorded as the onset of observability.

The crossing set may be empty. A fault injected near the end of the horizon can
be genuinely unobservable, and that is a real outcome the report will show
rather than drop.

SNR is expressed in multiples of the channel's nominal noise sigma at injection
time, not in engineering units, so thresholds are comparable across channels
that have nothing else in common.

## Consequence: the injector computes SNR, not the evaluator

Only the injection layer knows the counterfactual — what the channel would have
read with no fault present. Estimating the clean signal from observed telemetry
after the fact would reintroduce precisely the interpretive assumptions this ADR
exists to keep out. Hence `FaultInjector.expected_signal_snr`.

## Consequence: injection site is recorded per event

`InjectionSite.LATENT_STATE` means the fault perturbed physics and propagated
into the channels causally. `SENSOR_BOUNDARY` means the symptom was painted onto
telemetry because the backend exposed no state to reach. These support different
realism claims, so the site is recorded per fault event rather than asserted
once for the repository.

## README obligation

The README must state the onset convention, the headline SNR threshold, and the
three-way latency split in the results section itself — not in a footnote and
not only here. The specific, slightly uncomfortable precision is what makes the
rest of the numbers trustworthy.

## Alternatives rejected

- **Detectability as ground truth.** Flatters recall and latency, and conceals
  that the fault existed before any detector could have seen it. Not comparable
  across noise configurations.
- **Both, evaluated as two full independent metric sets.** Rigorous, but roughly
  doubles the published metrics surface and the CI gate count for information
  the decomposition already carries in one table.
