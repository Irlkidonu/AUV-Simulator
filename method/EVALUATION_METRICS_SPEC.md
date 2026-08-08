# Evaluation metrics and statistics specification

**Status:** DRAFT — authored 28 July 2026. Binding at the 7 August design freeze.
**Companion:** `../experiments/PROTOCOL.md`, `MODE_MANAGER_SPEC.md`,
`COMPARATOR_SPEC.md`.

Metric definitions, the aggregate outcome, and the statistical procedure are fixed
before any held-out execution. Rule `N5` of the protocol governs the whole document:
**mission-level metrics are primary; estimator error is a diagnostic and never supports
the primary claim.**

---

## 1. Primary metrics (claim-bearing)

All are computed by the evaluator against ground truth, after the run.

| ID | Metric | Definition | Direction |
|---|---|---|---|
| `P1` | Failed-mission rate | fraction of runs that do not visit every waypoint within tolerance before the mission time limit, or that terminate in an abort state | lower better |
| `P2` | RMS cross-track error | RMS perpendicular distance from the **true** vehicle position to the **commanded** path, over the mission | lower better |
| `P3` | Safety-violation rate | violations per mission of the geofence polygon or the altitude band, computed from true position | lower better |

`P2` is measured against true position, not the estimate. A method that believes it is
on-path while actually off-path is penalised — which is precisely the failure mode a
localization-only evaluation cannot see, and the reason the earlier ground-truth-feedback
controller produced meaningless navigation numbers.

### 1.0 Two implementation traps, and why the definitions above are worded as they are

The first implementation of these metrics passed all its tests and still measured almost
nothing. Both failures were faithful-looking shortcuts that quietly deleted the effect
under study, and both are recorded here because the wording above is what rules them out.

**`P1` says "visit every waypoint within tolerance", and "visit" means the vehicle, not
its estimate.** The first implementation scored completion from the guidance waypoint
index, which advances when the *estimate* reaches the waypoint. Since guidance is what
drives the estimate there, every method completed every mission by construction: the
failed-mission rate was exactly 0.000 for all eight comparators across 400 runs. A metric
with no variance cannot discriminate, and `J` silently collapsed to `P2` alone. Completion
is therefore scored on true position against a survey tolerance, checked continuously.

The tolerance must be **smaller than half the line spacing** (2.5 m against a 6.0 m
spacing). Otherwise a vehicle that has drifted a full spacing sits on the neighbouring
leg and is credited with covering the one it missed.

**`P2` says "the commanded path", and that means the segment currently being flown.** The
first implementation took the distance to the *nearest* segment of the whole lawnmower
pattern. That inverts the metric exactly where it matters: a vehicle whose estimate has
drifted one line spacing lies precisely on an adjacent leg and scores near-zero
cross-track error while surveying the wrong ground. The correct reference is the segment
from the previously captured waypoint to the current target, and nothing else.

Both faults share a signature worth stating plainly: **a metric that is method-independent
is not evidence of equivalence, it is evidence of a broken metric.** When every comparator
including the dead-reckoning floor reports the same number, the first hypothesis is not
that they perform identically.

### 1.1 Aggregate primary outcome

A single predeclared scalar, fixed at freeze:

```
J = w1 * norm(P1) + w2 * norm(P2) + w3 * norm(P3)
```

with per-scenario-family weights that make no family dominate, normalisation constants
computed from **development** data only, and the compound family `E7` weighted no lower
than any single-fault family. The weight vector and normalisation constants are written
into the freeze record. `J` is the quantity falsification condition `F1` is evaluated on.

Concretely, `J` is computed **per scenario family and then averaged with equal weight**,
never pooled over runs. Pooling is not a neutral choice here. Three of the five families
are single-fault conditions in which, correctly, there is nothing for a mode manager to
do; pooling lets them outvote the compound family three to one and dilutes a real effect
below the noise floor. Equal family weighting is the predeclared rule, it satisfies both
constraints above, and it is fixed before any held-out execution.

The same caution applies to **any ratio of aggregates**, and `oracle_recovery` is the one
that matters. It must be formed per scenario and averaged, never as a ratio of pooled
means. A scenario in which the oracle happens to be *worse* than the fixed policy
contributes a negative denominator, which flips the sign of the pooled ratio: in the
development campaign this produced a recovery of 2.12 and a spurious "the method beat the
oracle" alarm while every individual scenario was correctly bracketed. Scenarios whose
bracket is degenerate — where perfect information buys nothing — are reported as
degenerate and excluded from the average, never folded in.

---

## 2. Secondary metrics (reported, not claim-bearing)

| ID | Metric |
|---|---|
| `S1` | Waypoint capture rate; mean and p95 capture time |
| `S2` | Path-length overhead vs. the commanded path |
| `S3` | Mission-time overhead vs. the nominal-condition mission |
| `S4` | Recovery latency: fault clearance → mode `M0` restored **and** vehicle back within cross-track tolerance |
| `S5` | Terminal true position error at mission end |
| `S6` | Time spent in each mode, per scenario family |

`S2` and `S3` are the honesty counterweight to `P2`: a manager can always reduce path
error by crawling. Speed reductions and diversions must be paid for visibly.

---

## 3. Mode-decision metrics

The mode inference is evaluated as a detection problem against the injected fault
schedule. The schedule is evaluator-side only (`MODE_MANAGER_SPEC.md` §7, test `T3`).

| ID | Metric | Definition |
|---|---|---|
| `D1` | Detection latency | fault onset → first entry into the appropriate conservative mode, per fault type |
| `D2` | False-alarm rate | conservative-mode entries per hour on nominal scenario `E1` |
| `D3` | Missed-detection rate | injected faults never producing an appropriate mode entry |
| `D4` | Dwell distribution | time-in-mode histogram, per mode |
| `D5` | Chatter | mode transitions per minute; predeclared threshold feeds falsification `F3` |
| `D6` | Mode precision / recall | per-mode, against the schedule-derived reference labelling |
| `D7` | Transition attribution | distribution of the winning evidence term at transitions |

`D7` supports the explainability requirement: the paper reports *why* the manager
switched, which is what distinguishes an inference mechanism from a tuned threshold.

---

## 4. Diagnostic metrics (explicitly not claim-bearing)

Reported in a clearly labelled diagnostics subsection:

position and velocity RMSE, median, p95, maximum; NEES and NIS coverage against
consistency bounds; per-sensor measurement acceptance, rejection, false-acceptance, and
false-rejection rates; estimator uncertainty at recovery; update and end-to-end latency;
callback CPU time where measurable.

**No sentence in the abstract, introduction, conclusion, or any headline figure caption
may state an improvement in terms of these quantities.** This is the specific rhetorical
failure Paper 2 exists to correct.

---

## 5. Statistical procedure

- **M1 — paired analysis.** For a fixed seed, every method receives an identical sensor
  realisation (`COMPARATOR_SPEC.md` §2), so comparisons are paired per seed. Report
  paired differences, not independent-sample tests.
- **M2 — uncertainty.** Bootstrap confidence intervals over the per-seed paired
  differences, at a confidence level fixed at freeze. Report the interval, not only a
  point estimate or a p-value.
- **M3 — per-run transparency.** Publish the per-run outcome table. The earlier
  workspace practice of reporting only a cross-run mean improvement percentage is not
  used.
- **M4 — no seed selection.** All held-out seeds are reported. Outliers are analysed and
  discussed, never dropped.
- **M5 — multiplicity.** The primary claim is evaluated on `J` alone. Secondary,
  mode-decision, and diagnostic metrics are descriptive, and are not used to construct
  an alternative headline after the fact.
- **M6 — effect sizes with meaning.** Report absolute values alongside relative changes.
  A "60 % improvement" between two sub-decimetre errors is reported in metres first.
- **M7 — negative results retained.** Scenario families where the proposed method does
  not win appear in the same tables and figures as those where it does.

---

## 6. Figure plan

| Figure | Content |
|---|---|
| F1 | System diagram: sensors → estimator → mode manager → guidance/mission → vehicle, with the ground-truth boundary drawn explicitly |
| F2 | Mode timeline vs. injected fault schedule for exemplar runs of `E3`, `E5`, `E7` |
| F3 | Primary outcomes by scenario family, all methods, with `C1` and `C5` brackets marked |
| F4 | Bracket recovery: where `P` sits between fixed policy and oracle |
| F5 | Detection latency vs. false-alarm rate; chatter trade-off |
| F6 | Ablation panel, with `A1` (covariance-only) highlighted as the navigation-contribution test |
| F7 | Sensitivity sweep over the nuisance parameters of `PROTOCOL.md` §7, including null values |
| F8 | Qualitative Gazebo figure — illustrative only, labelled as not contributing statistics |

Figure F4 and the `A1` panel of F6 are the two figures a sceptical reviewer will look
for. Neither is optional, and neither may be cut under `PROTOCOL.md` §11.
