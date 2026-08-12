# Study 3 — correction specification V1

Written **before implementation**, as instructed. No parameter in this document
was chosen by searching against previous runs, and no value is justified by a
previous RMSE result. Date: 2026-08-11.

Scope: two defects, repaired separately and minimally. No broad redesign. The
8 s evidence TTL, the 4 s probe opportunity period and the 8 s action hold are
**not changed**.

---

## Part A — PREDICTIVE: the unreachable pre-emption gate

### The defect

`predicted_trigger` in `Study3Policy.step` requires `trigger`, which requires
`mode.mode is RELATIVE_DEAD_RECKONING`. `ObservableModeSelector` returns that
mode only when no acoustic service responds **and** `optical_probability` is
below the boundary — which is exactly `optical_now`. `predicted_trigger` then
also requires `not current_loss`, where
`current_loss = velocity_now or (optical_now and acoustic_now)`. Reaching the
gate therefore guarantees one half of the condition that closes it. Measured:
61 of 61 steps that reach `trigger` are excluded by `optical_now and
acoustic_now`; `predicted_trigger` fired 0 times in 5,299 steps.

A second, independent block: even if an episode started, `recovery_active` also
requires `mode.fallback_required`, which is only true once velocity aiding is
already unusable. Pre-emption could not take effect while the vehicle was still
healthy — the only time pre-emption means anything.

### Why the existing discriminator is wrong

The gate tried to separate "predicted loss" from "present loss" using the
belief-derived `*_now` predicates. That is the wrong instrument.
`CapabilityDegradationPredictor._time_to_floor` returns **0.0 when a capability
is already at or below its floor**, and `impending` is
`{name : time_to_loss_s[name] <= horizon_s}`. So `impending` contains
already-lost capabilities as well as soon-to-be-lost ones, which is why it is
non-empty on 98.6% of steps and cannot discriminate anything.

The correct discriminator was already present and unused:

> A capability is **predicted to be lost** when
> `0 < time_to_loss_s[name] <= horizon_s`.
> Strictly positive means it has *not yet* reached its floor; bounded by the
> horizon means the projection says it will.

This is a statement about the forecast's own semantics, not a tuned threshold.

### The repair

1. Define `predicted = {name : 0 < time_to_loss_s[name] <= prediction_horizon_s}`,
   plus the optical evidence forecaster's `warning` flag, which already carries
   the same "declining but not yet failed" meaning.
2. PREDICTIVE may open a recovery episode when a capability the **currently
   selected mode depends on** is in `predicted` and is not already lost.
   Dependence is read from the mode itself: an acoustic-aided mode depends on
   `acoustic`, an optical mode on `optical`, and every mode depends on
   `velocity`. No scenario identity or future state is consulted.
3. `recovery_active` additionally admits an episode that was opened
   pre-emptively, so a pre-emptive action can take effect while the mode is
   still viable.

REACTIVE is untouched: `predicted_trigger` remains gated on
`kind is PolicyKind.PREDICTIVE`, so `_recovery_preemptive` is never set for
REACTIVE and the added disjunct is always false for it.

No new constant is introduced. `prediction_horizon_s` already exists and keeps
its value.

---

## Part B — REACTIVE: switching between simultaneously viable acoustic modes

### The defect

Three facts compose. `SerializedServiceDiscovery` drops evidence older than
`evidence_ttl_s = 8.0`, and with a two-service catalogue each service is
re-probed every `2 × opportunity_period_s = 8.0 s`, so evidence expires exactly
as its replacement falls due. `ObservableModeSelector._candidate` treats service
availability as a binary drawn from the *currently visible* evidence and ranks
by fixed priority. And the anti-chatter hold is bypassed whenever the incumbent
leaves the visible set, because `old_still_viable` is then false — it is
disabled precisely in the case it exists for.

The selector conflates **absence of evidence** with **evidence of absence**.

### The observable distinction that already exists

`SerializedServiceDiscovery.observe` returns evidence entries for probes that
*completed*, including ones where the service did **not** answer
(`responding=False`, from `not packet.dropped`). So three states are already
distinguishable from observation alone:

| Observed | Meaning |
|---|---|
| entry present, `responding=True`, `gives_position=True` | confirmed viable |
| entry present, `responding=False` | **refuted** — the service was probed and did not answer |
| no entry at all | **stale** — no news; the service has not been re-probed yet |

The current code collapses the second and third into "not available".

### Selection objective

Replace *"select the highest-priority currently visible service"* with:

> **Among absolute-positioning sources believed viable on present evidence,
> select the one offering the smallest observable position uncertainty; retain
> the incumbent unless a candidate is meaningfully better; and abandon a source
> immediately when observation refutes it.**

### The rule, exactly

Let the selector hold, per service, the most recent *positive* evidence
`(sigma_m, dop)`. On each step, for each evidence entry received:

* `responding and gives_position` → record it; the service is **viable**;
* `not responding` → **erase** any held record; the service is **refuted** and
  becomes non-viable in the same step.

Entries absent from this step's evidence leave the held record untouched: the
service remains viable on its last positive observation. Round-robin probing
guarantees every catalogued service is re-probed every revisit interval, so a
stale record is replaced by either a positive or a negative observation within
one interval. **Retention therefore needs no timeout constant**; it is bounded
by the probe schedule itself.

Ordering among viable absolute acoustic sources is by observable uncertainty
`σ̂ = sigma_m`, with `dop` breaking exact ties, replacing the fixed LBL-over-USBL
precedence *between those two modes only*. The optical and dead-reckoning tiers
keep their existing precedence.

**Switching margin.** Change from a viable incumbent to a viable candidate only
when

```
    σ̂(candidate)  ≤  σ̂(incumbent) / √2
```

### Justification of √2, from sensor semantics only

A change of acoustic mode costs one probe opportunity: the newly selected
service must be interrogated and answer before it yields a fix, and while that
is pending the incumbent's next scheduled fix is forgone. The switch therefore
buys the vehicle one fix from the candidate in place of one fix from the
incumbent.

A position fix of standard deviation σ contributes Fisher information 1/σ² to
the horizontal estimate. For the exchange to be worth making, the candidate's
single fix must carry at least twice the information of the fix it replaces —
that is, it must repay the one opportunity spent making the change and still be
ahead:

```
    1/σ̂(candidate)²  ≥  2 / σ̂(incumbent)²
    σ̂(candidate)     ≤  σ̂(incumbent) / √2
```

√2 is the square root of that information ratio. It is derived from the
information content of a Gaussian position fix and from the serialized probe
schedule, both of which are properties of the sensing model. **No value was
tried against a previous run, and no outcome metric entered this derivation.**

In the pathology this addresses, both services report σ = 0.03 m; 0.03 is not
≤ 0.0212, so no switch occurs. That is the intended consequence and it follows
from the rule, not from fitting.

**Genuine loss is unaffected.** Refutation removes viability in the same step,
with no margin and no hold, so a real modality loss still adapts immediately.
Terminal safety remains immediate and is not routed through any of this.

---

## What is deliberately not done

* No change to `evidence_ttl_s`, `opportunity_period_s`, `minimum_hold_s`,
  `minimum_action_hold_s`, `usable_probability_boundary` or any recovery timing.
* No change to the optical or dead-reckoning tiers, scenario severity, or any
  baseline configuration.
* No parameter search of any kind.
* The existing `minimum_hold_s` hysteresis is left in place unchanged; the new
  rule works alongside it rather than replacing it.
