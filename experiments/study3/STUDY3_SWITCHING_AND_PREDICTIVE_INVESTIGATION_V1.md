# Study 3 — mode-switching and PREDICTIVE investigation (V1)

Classification: **READ-ONLY INVESTIGATION.** No controller, threshold or timing
constant was changed. No design proposed here is implemented.
Date: 2026-08-11.

Sources: every REACTIVE run in the Part E1 block (60) and every interactive
recording (10), replayed deterministically and verified against their stored
`trace_digest`. No case was generated or selected on its outcome; every switch
in every run is classified.

## 1. Why unnecessary acoustic switching occurs

Three mechanisms compose. Only the third is a design fault.

**(a) The evidence lifetime equals the service revisit period.**
`SerializedServiceDiscovery` probes strict round-robin, one service per
`opportunity_period_s = 4.0`, and drops evidence older than
`evidence_ttl_s = 8.0`. With the two-service catalogue `("lbl", "usbl")` each
service is revisited every 2 × 4.0 = **8.0 s — exactly the lifetime**. Each
service's evidence therefore expires at the very moment its replacement is due,
so the visible set oscillates between `{lbl}` and `{usbl}`.

**(b) The selector reads visibility as a binary and ranks by fixed priority.**
`ObservableModeSelector._candidate` asks only whether a service is in
`{s for s in services if s.responding and s.gives_position}`, then applies the
strict order LBL → USBL → optical+lock → optical → DR. Any flicker in visibility
maps one-for-one onto a mode change. `AcousticServiceEvidence` already carries
`sigma_m`, `dop` and `age_s`, and **the selector uses none of them.**

**(c) The anti-chatter hold is disabled exactly when chatter happens.**
This is the design fault. The hold in `ObservableModeSelector.select` applies
only when `old_still_viable` is true, and viability for LBL is
`any(s.name=="lbl" and s.responding and s.gives_position for s in services)`.
When LBL's evidence expires it leaves the visible set, so `old_still_viable`
becomes false and the hold is bypassed — in precisely the case it was written to
damp. The mechanism conflates **absence of evidence** with **evidence of
absence**.

### Measured

666 switches classified across 70 runs.

| Class | Count | Share |
|---|---|---|
| `staleness_driven` (unnecessary) | 196 | 29.4% |
| `loss_driven` (useful) | 141 | 21.2% |
| `ambiguous` | 329 | 49.4% |

118 of the 196 staleness-driven switches (60%) are LBL↔USBL flips: 61
`lbl→usbl` and 57 `usbl→lbl`. A further 24 are `usbl→relative_dead_reckoning`,
which is worse — absolute aiding abandoned entirely because evidence lapsed.

The evidence age at staleness-driven switches has **median 7.75 s and maximum
7.91 s against the 8.0 s TTL**. These switches occur at the lifetime boundary,
not when quality degrades.

A representative triple from `S1_optical_degrade_recover`, whose only
disturbance is turbidity:

```
t=105 s  lbl_aided -> usbl_aided   reason=observable_usbl_fix
   before: lbl(responding=True, age=8s, sigma=0.03)
           usbl(responding=True, age=4s, sigma=0.03)
t=113 s  usbl_aided -> lbl_aided   reason=observable_lbl_fix
t=121 s  lbl_aided -> usbl_aided   reason=observable_usbl_fix
   before: lbl(responding=True, age=8s, sigma=0.04)
           usbl(responding=True, age=4s, sigma=0.04)
```

Both services are responding with **identical σ**. The only difference is age.
The vehicle abandons a working mode for one that is no better by any observable
quality measure.

### The same constant also blocks legitimate adaptation

104 missed opportunities (episodes where the truth-side best viable mode changed
for ≥ 6 s and was never selected), **median duration 8.0 s** — one hold period.
Targets: `optical_no_bottom_lock` 26, `terminal_degraded` 25, `usbl_aided` 18,
`lbl_aided` 17, `relative_dead_reckoning` 14, `optical_dvl` 4.

So the 8 s hold fails in both directions: it does not damp the churn it was
written for, and it suppresses real mode changes that last about one hold. This
is why raising or lowering the constant cannot fix the behaviour — it trades one
failure for the other.

## 2. Recommended anti-chatter design — proposed only, not implemented

The objective should change from *"pick the highest-priority visible modality"*
to *"pick the modality minimising expected horizontal position error, and change
only when the gain exceeds the cost of changing."*

**1. Score modalities, do not rank them.** Give each candidate an expected
position-error estimate from quantities already observable:
`σ̂(lbl) = sigma_m × f(dop)`, `σ̂(usbl)` likewise, `σ̂(optical)` from the P5-v4
adapter's covariance, `σ̂(dead reckoning)` from the filter covariance trace and
its growth rate. Fixed priority becomes an emergent result — LBL usually wins
because its σ is usually smallest — rather than an axiom.

**2. Inflate uncertainty with age instead of deleting evidence at a cliff.**
Replace the hard TTL with `σ̂_eff = σ̂ + growth × age`, so a service at age 9 s is
*slightly worse*, not *gone*. This removes the oscillation at its source: the
incumbent degrades smoothly instead of vanishing. It also distinguishes "no
news" from "bad news" — a service that failed to answer a probe is different
from one not yet re-probed, and only the former is evidence of loss.

**3. Switch on a margin that reflects switching cost.** Change mode only when
`σ̂(candidate) < σ̂(incumbent) − δ`, with δ derived from the measured cost of a
change: reacquisition delay, fixes forgone during the transition, and any
physical reconfiguration. Under this rule the t=105 s example does not switch at
all, because 0.03 is not better than 0.03 by any margin.

**4. Keep dwell time for safety only.** Terminal safety must stay immediate.
The general dwell timer becomes unnecessary once (2) and (3) are in place, and
should be removed rather than retuned, since it is what suppresses the 8 s
adaptation episodes.

**5. Decouple decision rate from probe rate.** Maintain a per-service belief
that persists across the revisit interval. The selector should never be forced
to decide at the cadence at which the acoustic channel happens to be sampled.

This is a specification, not a patch. It changes the selector's objective, so it
would require its own predeclared protocol, mechanism tests and a fresh
comparison against the current behaviour on non-held-out seeds. **None of that
was started.**

## 3. Why PREDICTIVE never acts

**It is a design disconnect, not merely unmet conditions.** Pre-emption is
gated behind a state that is definitionally post-loss.

From `policies.py`:

```
trigger           = raw_trigger and mode.fallback_required
                                and mode.mode is RELATIVE_DEAD_RECKONING
current_loss      = velocity_now or (optical_now and acoustic_now)
predicted_trigger = trigger and not current_loss
```

`RELATIVE_DEAD_RECKONING` is returned by `_candidate` only when no LBL responds,
no USBL responds, **and** `optical_probability < boundary`. That last condition
is exactly `optical_now`. So reaching `trigger` already guarantees one half of
`current_loss`, and once no acoustic service has responded for a while the
acoustic belief falls below the boundary too, completing it.

Measured over 5,299 PREDICTIVE steps across the ten interactive recordings:

| Predicate | Steps | Share |
|---|---|---|
| forecast non-empty | 5224 | 98.58% |
| optical warning raised | 24 | 0.45% |
| `raw_trigger` | 2613 | 49.31% |
| in dead reckoning | 526 | 9.93% |
| `fallback_required` | 81 | 1.53% |
| **`trigger`** | **61** | **1.15%** |
| `current_loss` | 2286 | 43.14% |
| **`predicted_trigger`** | **0** | **0.00%** |
| **`action.preemptive`** | **0** | **0.00%** |

Of the 61 steps that reach `trigger`, **61 are blocked by
`optical_now and acoustic_now`** — every single one. (18 are additionally
blocked by `velocity_now`.) The exclusion is structural, not statistical.

The forecaster itself works: forecasts are produced on 98.6% of steps. The
machinery is sound and the gate is unreachable. PREDICTIVE is therefore a
correct implementation of a contradictory specification — it can only pre-empt a
loss once the loss has already occurred, at which point there is nothing left to
pre-empt.

This confirms and explains the development null
(`STUDY3_DEVELOPMENT_NULL_FINDINGS.md`): `preemptive_actions == 0` is not
evidence that prediction has no value, only that this gate can never open.
Whether prediction helps remains **untested**.

The natural repair is to evaluate the forecast against modes that are *still
viable* — trigger when a currently-selected absolute mode is forecast to be lost
within the prediction horizon, rather than requiring the vehicle to be in dead
reckoning first. **Not implemented, not tuned.**

## 4. Other defects found

None beyond the two already reported. Specifically checked and clean:

- the full regression suite passes (419 tests);
- all 180 Part E1 packets and all 27 interactive replays reproduce byte-identical
  results under the fixed implementation;
- `truth_side_best_viable_mode` and `physical_acceptable_modes` agree on the
  cases exercised;
- no truth leakage assertion fired in any replayed run.

The `ambiguous` class at 49.4% is large and is **not** a finding — it is mostly
switches involving optical or DVL, for which the service-response test does not
apply, plus switches inside the 30 s truncation window at run end. It is
reported rather than reclassified.

## 5. Adverse and null findings preserved

- REACTIVE makes more unnecessary switches (196) than useful ones (141) across
  the material examined.
- 104 legitimate adaptation opportunities were missed, with a median duration of
  exactly one hold period.
- PREDICTIVE's pre-emption branch is dead code in practice.
- The exposure figure in the first interactive report was overstated by roughly
  a factor of two and has been corrected.

Nothing here was used to change the controller, and no correction cycle was
started.
