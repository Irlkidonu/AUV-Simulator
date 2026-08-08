# Mode-aware navigation manager specification

**Status:** DRAFT — authored 28 July 2026. Binding at the 7 August design freeze.
**Companion:** `../experiments/PROTOCOL.md`, `COMPARATOR_SPEC.md`,
`EVALUATION_METRICS_SPEC.md`.

The manager is the paper's contribution. It must (a) infer its operating mode from
onboard observables without any privileged input, and (b) act on that mode through
**guidance and mission decisions**, not only through measurement weighting. Requirement
(b) is what makes this a navigation paper; falsification test `F4` in the protocol is
the check that it was actually met.

---

## 1. Inputs — observables only

The manager consumes exactly the following. Anything not on this list is a protocol
violation of rule `N2` and fails the isolation test `N4`.

| Class | Signal | Source |
|---|---|---|
| Availability | per-sensor message age vs. expected period; DVL **bottom-track** and **water-track** flags; acoustic-fix age | sensor interfaces |
| Consistency | per-sensor normalised innovation squared, windowed mean and exceedance rate | estimator |
| Uncertainty | position-covariance trace and its growth rate; predicted time-to-threshold | estimator |
| Optical evidence | image-derived optical quality score **and its trend across the identification window** | perception front end |
| Flow | estimated current speed and current-covariance trace | estimator (12-state) |
| Identified environment | turbidity / acoustic-noise / current class and the weakest of the three confidences | environment classifier |
| Geometry | acoustic beacon range and elevation adequacy for the current position estimate | estimator + beacon map |
| Self-state | current mode, time in mode, recent transition history | manager |

Two DVL flags rather than one, because the capability difference is real: a vehicle
that has lost bottom track but retains water track still measures its motion through
the water and still bounds its velocity error. Treating that as equivalent to total
loss collapsed a distinction the vehicle can observe and should act on, and made the
total-DVL-loss scenario indistinguishable from the bottom-lock one.

The optical **trend** is present because availability prediction was otherwise
memoryless. The manager commits to a configuration for the decision horizon, so a
quality of 0.4 on a falling trend and the same 0.4 on a rising trend imply opposite
answers to the question actually being asked.

The identified environment is an **inference**, never a commanded value. Labels are
produced from image statistics, from the acoustic gate's own rejection behaviour, and
from the filter's current state. They are wrong a measurable fraction of the time, and
the measured rates are reported rather than assumed.

**Explicitly forbidden inputs:** ground truth in any form; the commanded fault schedule;
the commanded optical condition or turbidity level; scenario identifiers; evaluator
quantities; wall-clock scenario phase boundaries.

> The invalidated earlier system derived its optical quality as `q = 1 - turbidity`,
> i.e. it inverted a value the experiment itself had set. Optical quality here is
> computed from image content only, and the commanded condition is never published to
> any node in the estimation or decision path.

---

## 2. Mode inference

### 2.1 Evidence to mode

Each mode `M0`--`M5` receives a scalar evidence score from the observables in §1. The
scores are combined into a mode decision through a rule that must satisfy:

- **fail-closed:** invalid, stale, or numerically unsafe inputs drive the decision
  toward the more conservative mode, never toward `M0_NOMINAL`;
- **monotone in capability loss:** losing a modality can never move the decision to a
  strictly less conservative mode;
- **explainable:** the winning evidence term is logged at every transition, so the
  paper can report *why* the manager switched, not just that it did.

### 2.2 Stability machinery

Chatter was a documented failure of the earlier system (30 mode flips per second from an
unnormalised quality signal with no hysteresis). The following are mandatory:

- **asymmetric hysteresis** — entering a more conservative mode is easier than leaving
  it;
- **minimum dwell time** per mode, tuned on development seeds;
- **debounce** — an entry condition must persist over a confirmation window;
- **confirmed re-acquisition** — leaving `M5_RECOVERY` requires two consecutive
  consistent measurements from the returning modality before it is trusted at full
  weight, mirroring the two-sample bottom-lock confirmation that resolved Paper 1's
  reject–drift–reject collapse.

Chatter is measured, not assumed away: transitions per minute is a reported metric with
a predeclared threshold in falsification condition `F3`.

### 2.3 Permitted transitions

`M0 ↔ M1 ↔ M2`, `M0 ↔ M3`, `{M1,M2,M3} → M4`, `M4 → M5`, `{M1,M2,M3} → M5`, `M5 → M0`.
Direct `M4 → M0` is forbidden: recovery from a critical state must pass through
confirmed re-acquisition.

---

## 3. Action space

Three tiers. A manager restricted to tier 1 is ablation `A1` and is **not** a navigation
contribution.

The space is **54 configurations**: three optical channels x three altitudes x two
speeds x three acoustic techniques. The acoustic axis was added because a paper
claiming the vehicle selects its navigation *technology* must let it do so; without it
the vehicle chose only among optical channels. The three techniques differ in what they
measure, not merely in how well:

| Technique | Measures | Fix period | Limitation |
|---|---|---|---|
| Single beacon | range only | 2 s | constrains a circle, not a point |
| LBL | full position, 4 seabed transponders | 6 s | three round trips per fix; needs a surveyed array |
| USBL | range and bearing, surface transceiver | 2 s | error proportional to slant range; needs a surface vessel |

**Every axis must be verifiably live.** Commanded altitude was inert for part of
development -- line-of-sight guidance built its direction between two waypoints at
equal depth, so the vertical command was identically zero and configurations at 1.0 m,
2.0 m and 3.0 m produced bit-identical outcomes on every scenario and channel. Altitude
is the exponential lever in this study, since the light makes a two-way trip of roughly
twice the altitude, so a third of the action space was dead while the full test suite
passed. `test_action_space_is_live.py` now fails if any axis stops changing outcomes.

### Tier 1 — estimation actions
- Which measurement streams are admitted.
- Measurement covariance policy per admitted stream.
- Re-acquisition gating after an outage.

### Tier 2 — guidance actions
- Commanded speed cap.
- Waypoint capture radius.
- Cross-track tolerance and path-adherence aggressiveness.
- Leg length before an absolute-fix opportunity is required.

### Tier 3 — mission actions
- Continue the survey as planned.
- Hold station until an absolute fix is obtained or a timeout expires.
- Return to the last position at which the estimate was well-supported.
- Divert toward acoustic-beacon geometry adequate for a fix.
- Abort the current leg and re-plan the remainder of the mission.

### Per-mode mapping (levels fixed at freeze)

| Mode | Tier 1 | Tier 2 | Tier 3 |
|---|---|---|---|
| `M0_NOMINAL` | all streams, nominal covariance | full speed, nominal capture | continue |
| `M1_OPTICAL_DEGRADED` | optical down-weighted | speed reduced, capture tightened | continue |
| `M2_OPTICAL_LOST` | optical excluded | speed reduced | continue; require fix opportunity within leg budget |
| `M3_VELOCITY_AIDING_LOST` | DVL excluded; absolute aiding prioritised | speed reduced; legs shortened | divert for fix if uncertainty projects past threshold |
| `M4_DR_CRITICAL` | dead reckoning with conservative gating | minimum speed | bounded hold, **only when a fix could arrive**; otherwise continue |
| `M5_RECOVERY` | confirmed re-acquisition gating | speed ramped back | re-plan remainder, then resume |

### 3.1 Holding is a priced action, not a safe default

Two properties of the tier-3 hold are easy to omit and expensive to omit. Both were
omitted in the first implementation, and between them they made the full manager score
*worse* than the ablation that has tier 3 switched off entirely — a result that reads as
"mission actions are harmful" and is nothing of the kind.

**The hold must be bounded.** §4 declares a hold timeout; without it the vehicle waits
until the mission time limit. An unbounded hold is not a conservative choice, it is a
mission failure reached slowly.

**The hold must be gated on a fix opportunity existing.** A vehicle holding station holds
against *its own estimate*. While that estimate diverges, the station-keeping controller
faithfully converts estimate error into physical displacement — it moves the vehicle to
keep a drifting number constant. That is the correct behaviour and a reasonable price for
a fix that is coming; it is a pure loss for one that is not. The manager therefore holds
only when optical aiding is currently available, or an acoustic fix arrived within the
fix-opportunity window. Both are observables; no privileged information is used.

**And a hold is not zero thrust.** Commanding zero velocity is not station-keeping, it is
drifting on the current. Because an unbounded hold ran to the mission time limit, the
displacement accumulated over the whole run rather than over the outage: in the coupled
turbidity/velocity-aiding scenario the measured RMS cross-track error was 4.6 m against
1.2 m for the fixed policy. The action that exists to *protect* the mission was the single
largest source of path error in the campaign.

*(Development-data observation, recorded to justify a design decision. It is not a result
and does not appear in the manuscript's results section.)*

### 3.2 What `RETURN_TO_LAST_GOOD_FIX` can and cannot do here

The action is in the repertoire above and is deliberately **not** used, for a reason that
is a property of this study rather than of the action.

Degradation here is scheduled in **time**, not in space. Turbidity is uniform across the
survey area and the faults are temporal windows, so a position at which a fix was obtained
ten seconds ago has no better prospect of yielding one now than any other position.
Returning to it would be a no-op dressed as a decision, and reporting it as an exercised
capability would be misleading.

A study with spatially structured degradation — a turbid plume, an acoustic shadow behind
terrain, a beacon with limited coverage — would need this action, and the water and beacon
models would have to carry spatial structure for it to mean anything. That is stated here
as a limitation of the present design, not deferred silently.

---

## 4. Tunable parameters

All are selected on **development seeds only** (`PROTOCOL.md` §8, rule D3) using the
equal-budget procedure in `COMPARATOR_SPEC.md` §3, and are frozen on 7 August.

Evidence thresholds and windows; hysteresis margins; minimum dwell per mode; debounce
and confirmation window lengths; speed caps and capture radii per mode; uncertainty
threshold and projection horizon for the `M4` trigger; hold timeout; leg fix-budget.

### 4.1 The mission-cost budget is mode-scaled

Configuration selection is constrained by a mission-cost budget: a configuration costing
more than the budget is not selectable, however good its perception would be. That budget
is **scaled by the active mode** rather than held fixed.

A single fixed budget cannot express both of the behaviours the system needs, and forces
a choice between them. Set it low enough that a healthy vehicle does not buy perception it
does not need, and a vehicle about to lose its navigation solution cannot afford the
altitude and channel changes that would save it. Set it high enough for the second, and
the manager over-reacts while nothing is wrong — which shows up directly as lost survey
swath and wasted power in the nominal scenario, where the correct behaviour is to do
nothing at all.

Scale factors, applied multiplicatively to the base budget:

| Mode | Scale | Rationale |
|---|---|---|
| `M0_NOMINAL` | 1.0 | nothing is wrong; buy nothing |
| `M1_OPTICAL_DEGRADED` | 1.6 | worth paying to keep a fix alive |
| `M2_OPTICAL_LOST` | 2.2 | the fix is already gone |
| `M3_VELOCITY_AIDING_LOST` | 2.2 | drift is now unbounded in velocity |
| `M5_RECOVERY` | 1.6 | pay to confirm re-acquisition, then stand down |
| `M4_DR_CRITICAL` | 3.0 | every affordable action is on the table |

The ordering is monotone in conservatism by construction and is not itself tuned; only
the base budget is. Ablation A1 pins the vehicle at nominal altitude and speed, so the
scaling has no effect there — which is the intended reading of that control.

The complete parameter vector, with the search ranges and the number of candidate
evaluations consumed, is written into the freeze record.

---

## 5. Ablations

| ID | Removes | Question it answers |
|---|---|---|
| `A1` | **Tiers 2 and 3** — mode inference retained, but the manager may only retune covariance | **Is this a navigation paper?** The decisive control for falsification `F4`. |
| `A2` | Tier 3 only — guidance actions retained, mission actions removed | How much comes from mission-level decisions specifically? |
| `A3` | Hysteresis, dwell, and debounce | What does stability machinery buy, and at what chatter cost? |
| `A4` | Acoustic aiding S5 from the action space | Does the non-optical modality create real navigational choice, or decoration? |
| `A5` | Innovation/uncertainty evidence, leaving availability and optical evidence only | Which observables carry the mode signal? |

`A1` and `A2` are non-negotiable and are protected from the scope cuts in
`PROTOCOL.md` §11.

---

## 6. Interface to Paper 1

Paper 2 needs a localization estimate. Paper 1's redesign is on version five, has failed
four consecutive campaign attempts, and has never executed held-out. Paper 2 **cannot**
take a live dependency on it.

- **I1.** The estimator is **vendored**: a copy taken at a recorded SHA-256 digest,
  placed under `src/`, with the digest and provenance recorded in the
  freeze record.
- **I2.** Paper 1's implementation is never modified in place, per `AGENTS.md`.
- **I3.** Paper 2 does not import, cite, or reuse any Paper 1 result, seed, campaign,
  or figure as evidence. The interface is code, not results.
- **I4.** A later Paper 1 version does not invalidate Paper 2. If Paper 1 changes after
  the Paper 2 freeze, Paper 2 reports which digest it used and moves on.
- **I5.** The vendored estimator is identical across the proposed method and **every**
  comparator (`COMPARATOR_SPEC.md` §2).

---

## 7. Isolation tests

These implement rules `N1`--`N4` and are part of the freeze record.

- **T1.** Static node-graph inspection: no ground-truth topic appears in any
  subscription of the controller, manager, estimator, or perception front end.
- **T2.** Runtime interception: a sentinel ground-truth publisher records every consumer;
  the test fails if any decision-path node appears.
- **T3.** Fault-schedule isolation: the schedule object is constructed in the evaluator
  process and is unreachable from the manager's address space; verified by import-graph
  inspection and by a runtime attribute probe.
- **T4.** Determinism: a scenario re-run from the same seed reproduces the mode-event
  stream byte-for-byte.
- **T5.** Measurement-realisation parity: for a fixed seed, every method receives an
  identical sensor stream; verified by digest comparison across methods.

---

## Errata — divergences from the implementation

**Appended 3 August 2026, after the design freeze. The text above is unchanged.**

A pre-registration is worth nothing if it is edited to match what was built, so
nothing above has been altered. What follows records where the implementation
diverged, so that a reader comparing this document against the released code is
not left to guess which is authoritative. **The code is authoritative; this
section says where it differs.**

### E1. The action space is 108 configurations, not 54

§3 declares three optical channels x three altitudes x two speeds x three
acoustic techniques = 54. A sixth axis was added during development: the
measurement admission strategy, gating against covariance weighting, which
doubles the space to **108**. It was added because whether a surprising
measurement should be rejected or admitted with an inflated covariance depends
on whether the present errors are systematic drift or one-sided outliers, which
is a condition the vehicle can infer — so it is properly an action rather than a
tuning constant. Every reported campaign sweeps 108.

The same stale count appears in `COMPARATOR_SPEC` R8, which says 18, and in
`PROTOCOL` §6.2, which says 54.

### E2. Tier 3 declares an action that does not exist

§3 lists *"divert toward acoustic-beacon geometry adequate for a fix"* among the
mission actions, and the per-mode mapping assigns it to `M3`. **It was never
implemented.** `MissionAction` has five members: continue, hold for fix, return
to last good fix, abort leg, and surface for GPS. The word `divert` survives only
in source comments.

Nothing in the paper claims this action exists; its action-space description
lists the five that do. The divergence is recorded because this document ships
with the artefact and would otherwise contradict the code it describes.

### E3. Tier 3 omits the terminal action, which is central to the paper

`SURFACE_FOR_GPS` — abandon the survey, ascend, hold at the surface for a
satellite fix — was added after this document was written and is not listed in
§3. It is the bottom of the escalation ladder and the subject of an entire
results section. Where the per-mode mapping gives `M4_DR_CRITICAL` a *"bounded
hold, only when a fix could arrive"*, the implementation escalates past that to
surfacing when the position estimate has been unsurveyable for a declared dwell.

### Why these were not caught earlier

All three are the same failure: this document was written on 28 July, the
implementation moved past it over the following week, and nothing compared the
two until the manuscript was being audited on 3 August. The freeze record digests
the *code*, which is the right thing to digest, but it does not check the code
against its own specification, and no test does either. That is a real gap in the
method and is stated as one.
