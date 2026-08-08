# Comparator and fairness contract

**Status:** DRAFT — authored 28 July 2026. Binding at the 7 August design freeze.
**Companion:** `../experiments/PROTOCOL.md`, `MODE_MANAGER_SPEC.md`,
`EVALUATION_METRICS_SPEC.md`.

This workspace has a documented history of comparator crippling. In June 2026 a 98.6 %
headline improvement was traced to a baseline that omitted gravity compensation and drag
while the proposed method had both, so the figure measured physics correctness rather
than method quality; a second defect scaled DVL noise with turbidity, degrading a sensor
that is physically turbidity-independent, in the baseline's disfavour. Both were caught
internally, but only after they had been written up.

This document exists so that Paper 2's comparators are defensible **by construction**
rather than by a fairness paragraph written afterwards.

---

## 1. Comparator set

| ID | Method | Role |
|---|---|---|
| `P` | Proposed automatic mode-aware manager | the claim |
| `C1` | **Best fixed policy** — one static configuration, tuned with equal budget, applied in all conditions | **primary comparator**; the lower bracket |
| `C2` | Residual-only reactive policy — innovation/NIS gating and covariance response, no mode concept, no mission actions | is an explicit mode abstraction needed? |
| `C3` | Continuous covariance adaptation driven by optical quality — the measurement-weighting approach | the "is this just measurement gating?" control |
| `C4` | No absolute aiding — dead reckoning on IMU+DVL+depth | performance floor; makes the aiding contribution legible |
| `C5` | **Oracle mode manager** — the *same policy* as `P`, with clairvoyant availability | **upper bracket / ceiling** |

### 0.9 `C1` is a frontier, not a configuration

Reducing the static configurations to a single "best" baseline discards information the
comparison depends on, and the development campaign showed exactly how it fails.

Navigation quality and **survey productivity** — the rate at which seabed area is actually
imaged, which scales with altitude and with ground speed — are separate axes. The
predeclared aggregate `J` measures only the first. Selecting one baseline by `J` therefore
ignores the second, and in the development campaign the top two static configurations were
separated by **0.3% of `J`, one failed run in eighty**, while differing by a **factor of two
in mission time**. The tie-break between them was arbitrary in any meaningful sense.

It was also actively harmful. The slower configuration accumulates roughly 0.11 m more
cross-track error, because the vehicle spends twice as long being pushed by the ambient
current between waypoints. A controlled sweep isolates it — at fixed altitude, 0.25 m/s
gives 0.891 m and 0.50 m/s gives 0.792 m, and altitude changes nothing — so the arbitrary
tie-break injected a constant apparent improvement into every downstream comparison,
**including scenarios in which nothing was degraded and there was nothing to manage.**
Reported as a result, it would have been an artefact of a coin flip.

Paper 2 therefore reports the **Pareto frontier of all static configurations** on
(`J`, productivity), and asks whether an adaptive policy lies outside it. Concretely:

- every static configuration is published, best to worst, with both coordinates;
- the primary claim is that **no static configuration attains the proposed method's
  navigation outcome at any productivity whatsoever** — a claim the published table
  falsifies immediately if it is untrue;
- the cost of the method's actions is read directly off the productivity axis rather
  than argued.

Nothing predeclared changes. `J` is untouched and still reported exactly as declared.
Productivity is the product of secondary metrics `S2` and `S3`, both of which the
metrics specification already required to be reported, and which it already described as
the honesty counterweight to `P2`. No configuration is selected away, so a reader who
prefers a different operating point can find it in the table.

*Recorded for transparency: the frontier presentation was adopted on 29 July 2026, after
the development campaign exposed the tie-break artefact and before the freeze. It replaces
a presentation, not a measurement.*

### 1.0 What `C5` must be, and what it must not be

`C5` is the proposed manager itself: the same action space, cost model, budget, mode
machine, and hysteresis. Exactly one thing differs. Where `P` must *infer* whether a
candidate configuration would yield a fix, using the learned availability model on
observables alone, `C5` is told the answer — evaluated against the true water profile and
the true fault schedule, over the manager's own projection horizon.

That single-difference construction is what gives `oracle_recovery` a meaning. The
statistic is then unambiguously *the fraction of the achievable benefit of better
information that inference from observables recovers*, because information is the only
variable.

An oracle written as a **separate hand-tuned heuristic** that happens to receive
privileged inputs is not a ceiling, and the first implementation here was exactly that.
Such a comparator can be beaten by the proposed method for reasons that have nothing to
do with information — a differently shaped decision rule, a worse cost trade — and when it
is beaten it does not reveal a bug, it just silently destroys the bracket it exists to
provide. The bracket must be guaranteed by construction, not hoped for.

The oracle's knowledge is **forward-looking**, over the same `DECISION_HORIZON_S` the
manager already projects across. Restricting it to the present instant was tried and
produced a degenerate bracket: ground truth about current conditions beat the learned
model by under 0.005 m, because observables already reveal the present almost perfectly.
The headroom a mode-aware manager could exploit is not what is happening now but what is
about to happen — a clairvoyant reconfigures *before* degradation arrives, whereas `P`
can only act once evidence has accumulated. The horizon is taken from the manager's own
parameters rather than chosen, so the bracket cannot be widened by tuning the oracle.

### 1.1 Why the brackets matter

`C1` and `C5` together are the strongest available answer to "did you cripple the
baseline?". The proposed method must land **between** them, and the paper reports the
fraction of the oracle's benefit that automatic inference recovers:

```
oracle_recovery = (outcome(C1) - outcome(P)) / (outcome(C1) - outcome(C5))
```

A method that beats its baseline but recovers only a small share of the oracle ceiling
is reported honestly as such. A method that appears to *exceed* the oracle is treated as
evidence of a defect in the comparator or the metric, investigated, and reported — never
as a result.

`C3` is the comparator that a reviewer will demand: it is the measurement-weighting
approach that the earlier work in this workspace already published. If `P` cannot beat
`C3` on mission-level outcomes, Paper 2 has no system-level contribution — this is the
comparator counterpart of ablation `A1` and falsification condition `F4`.

---

## 2. Shared substrate

Every method in the set shares, bit-for-bit, the following. Divergence in any of them is
a defect, not a design choice.

| Component | Requirement |
|---|---|
| Sensor measurement realisation | identical per seed; verified by digest comparison (test `T5`) |
| Estimator core | the same vendored filter, same digest (`MODE_MANAGER_SPEC.md` §6) |
| Process noise and physics model | identical |
| State initialisation | identical |
| Latency, jitter, and stale-measurement policy | identical |
| Controller structure and gains | identical; only the commanded speed cap / capture radius may differ, and only where a method's action space includes them |
| Mission, waypoint list, geofence, altitude band | identical |
| Numerical safeguards (Joseph form, normalisation, guards) | identical |

**No comparator receives degraded, delayed, or withheld measurements.** Faults are
injected into the shared sensor stream before any method sees it.

---

## 3. Equal tuning budget

- **B1.** Every method with free parameters — including `C1`, `C2`, and `C3` — is tuned
  by the **same automated procedure**, over its own declared search ranges, on the
  **same development seeds**.
- **B2.** Every method receives the **same number of candidate evaluations**. The count
  is fixed before tuning begins and recorded in the freeze record.
- **B3.** No method is hand-tuned. No comparator's parameters are set by intuition,
  inherited from an earlier project, or chosen as a "reasonable default" while the
  proposed method is optimised.
- **B4.** `C1`'s static configuration is selected as the best-performing single
  configuration on the aggregate development outcome — the genuine "best you can do
  without condition awareness", not a midpoint or a geometric-mean placeholder.
- **B5.** Tuning logs (candidates, evaluations, selected vector) are retained for every
  method and published with the artifacts.

---

## 4. Anti-crippling rules

Each rule has a checked implementation, not just a statement.

- **R1 — physics parity.** Every method runs the identical vehicle model, gravity
  compensation, and hydrodynamics. A regression test asserts that removing the method
  layer leaves all comparators numerically identical. *(Addresses the June 2026 defect.)*
- **R2 — sensor-model parity.** Sensor noise, bias, and availability are functions of
  the scenario and seed only, never of which method is running. A test asserts the
  sensor stream digest is method-independent. *(Addresses the turbidity-scaled DVL
  defect.)*
- **R3 — no privileged input.** Only `C5` receives the fault schedule, and it is
  labelled an oracle everywhere it appears, including in every figure legend and table.
- **R4 — no post-hoc retuning.** After held-out inspection, no comparator is retuned,
  no comparator is added, and no comparator is dropped.
- **R5 — comparator failures are reported.** If a comparator behaves pathologically, it
  is diagnosed and fixed on development data before the freeze, or reported as a
  limitation. A pathological comparator is never left in place to flatter `P`.
- **R6 — controller parity.** Where a method's action space excludes guidance actions,
  it runs the nominal controller settings — not degraded ones. `A1` and `C3` navigate
  with the same nominal guidance `C1` uses.
- **R7 — reporting parity.** Every method is reported on every scenario family and every
  metric. No method is shown only where it wins, and no scenario is omitted.
- **R8 — the baseline is selected, and the selection is published.** `C1` is chosen as
  the best member of an exhaustive sweep over the manager's own 18-configuration action
  space, evaluated on the full development family before the proposed method is run. The
  complete sweep table is published, best to worst, in `results/static_sweep.csv`.

  This rule exists because "the baseline was weak" is the criticism that sank the earlier
  work in this workspace, and it cannot be answered by assurance. A reader who suspects a
  handicap can find the configuration they would have chosen in the published table and
  read off its score. If a configuration outside the sweep would have done better, the
  sweep was too narrow and that is a stateable, checkable objection rather than a matter
  of trust.

  The sweep does double duty: its per-seed best, chosen with hindsight, is one of the two
  terms bounding `C5` (§1.0).

### 4.1 A note on what these rules cannot catch

Every rule above was satisfied, with passing tests, by a version of this study in which
the acoustic beacon returned a range on every 0.1 s simulation tick. Parity held —
every method received the same over-generous sensor. What the parity rules cannot detect
is a *shared* modelling error that makes the whole comparison vacuous: 10 Hz range-only
aiding bounds position error by itself, so every comparator scored identically and the
study's apparent finding, that optical management does not matter, was an artefact of the
sensor model rather than a result.

The guard against this is not a fairness rule. It is the discrimination criterion in
`PROTOCOL` §5.1 — stated in terms of `C1` and `C4` alone, so it cannot be tuned toward a
favourable outcome — together with the principle that **a metric identical across every
method is evidence of a broken experiment before it is evidence of equivalence**.

---

## 5. Statement required in the manuscript

The paper states plainly, with a table, that all methods share sensor realisations,
estimator, physics, controller, initialisation, and tuning budget; that only `C5`
receives privileged information and is labelled an oracle; and that the proposed method
is expected to fall between `C1` and `C5`. Where the proposed method loses, that is
reported in the same table as where it wins.

---

## Errata — divergences from the implementation

**Appended 3 August 2026, after the design freeze. The text above is unchanged.**

### E1. R8 states an 18-configuration sweep; it is 108

The acoustic technique and measurement-admission axes were both added during
development. `MODE_MANAGER_SPEC` errata E1 records why. Every reported campaign
sweeps 108 configurations, and the complete table is published as
`results/static_sweep_development_v5.csv`.

### E2. R5 was not being honoured for one comparator

R5 requires a comparator that behaves pathologically to be diagnosed and fixed on
development data before the freeze, or reported as a limitation. `residual_only`
switches optical channel **38.6 times per run on average and up to 185**, against
0.13 for the proposed method: it has no hysteresis by construction and oscillates
under fluctuating quality. This was neither fixed nor reported until the
manuscript audit on 3 August.

It is now reported in the paper rather than repaired. Repairing it would mean
granting that comparator the hysteresis whose value is part of what the paper
claims, which would be the opposite of the fairness this rule protects. The
paper states that part of the proposed method's margin over `residual_only` is
the cost of that oscillation and should be discounted accordingly.

### E3. §5's required statement was missing from the manuscript

§5 requires the paper to state, *with a table*, what every method shares. The
manuscript asserted parity in scattered prose and carried no such table until
3 August. It is now Section 5.1, `tab:parity`.
