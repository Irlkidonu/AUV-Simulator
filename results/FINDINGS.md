# Development campaign — findings

**The held-out campaign has now run** (3 August 2026, once, against a verified
freeze record). Sections 1--7 below are the development record; **section 0f is
the held-out result**, and where the two disagree the held-out one is binding.
Two development findings reported in sections 1--7 did not replicate and are
retracted there.

> **Sections 1--6 were rewritten on 2 August 2026** against campaign `v5`, the
> first campaign in which the escalation ladder was verified working before the
> run rather than discovered broken after it. The superseded text has been
> removed; the defects that invalidated it are recorded in sections 0a--0c.

---

## 0a. Four defects found while adding current adaptation

Adding an ocean-current estimator removed the error that had been masking these.
All four were found by measurement, and all four are corrected in the source.

### 0.1 Dead reckoning never drifted

Terminal position error was 0.07 m after 98 m of survey and 0.09 m after 1,034 m
— flat with distance. The DVL carried white noise only, with no scale-factor
error and no mounting misalignment, so integrated position error grew as √t
rather than linearly with distance travelled. No real vehicle achieves that, and
it left absolute aiding nothing to contribute, which is the premise of the paper.

The companion paper's `SENSOR_SIMULATION_SPEC` models both terms, so the omission
was specific to Paper 2. Corrected by drawing a scale factor (0.3%) and a
residual yaw misalignment (1°) once per scenario. Dead-reckoning drift is now
**0.14% of distance travelled**, constant across 98 m, 254 m and 514 m surveys,
and inside the range instrument datasheets quote.

### 0.2 Cross-track error was measuring the capture radius

Holding everything else fixed and varying only the capture radius:

| capture radius | RMS cross-track | max cross-track |
|---|---|---|
| 1.50 m | 0.766 m | 1.47 m |
| 0.75 m | 0.328 m | 0.72 m |
| 0.30 m | 0.080 m | 0.26 m |

Maximum cross-track error equals the capture radius to two decimals. The paper's
primary path-quality metric was reporting a configuration constant, identical for
every method. Dead-reckoned and optically aided flight differed by under 2% of it
while their terminal position errors differed threefold.

### 0.3 The cause was pure-pursuit guidance

Steering at the next waypoint means flying the chord from wherever the leg was
entered, so the entry offset decays linearly along the whole leg instead of being
driven to zero. Excluding the turns from scoring did not fix it. Replaced with
lookahead line-of-sight path following, the standard marine path-following law,
which drives cross-track error to zero and leaves the residual reflecting
estimate error and imperfect current compensation.

Cross-track error remains sensitive to the capture radius even under LOS — the
dead-reckoned-to-aided ratio is 2.20, 1.52 and 1.23 at radii of 0.50, 0.75 and
1.00 m — while terminal position error does not move at all (0.42 m against
0.09 m throughout). **Terminal position error is therefore the metric robust to
this choice, and cross-track error must be reported beside it, never alone.**
The capture radius is set to 0.5 m on platform grounds (one vehicle length), and
the sensitivity is reported rather than tuned away.

### 0.4 The compound and decisive scenarios were no longer compound

Adding DVL water track gave every policy a velocity reference that survived
bottom-lock loss. `compound_schedule` and `coupled_turbidity_dvl_schedule` (E7,
E8) faulted bottom track only, so the vehicle stayed aided throughout: dead
reckoning completed the compound scenario with 0.22 m position RMSE, and on E8
the proposed manager became indistinguishable from both the fixed policy and the
ablation that takes no action at all (0.293 vs 0.293).

Both now fault **both** DVL modes, on the physical ground that these cells
represent an instrument failure rather than loss of the seabed — a failed DVL
returns neither mode. Bottom-lock-only loss remains available separately as the
degraded-but-not-lost case the capability modes distinguish. With this corrected,
the decisive-case, F4 and improvement-cost gating tests pass again.

### 0.5 A comparator result worth reporting

Across nominal seeds, `covariance_only` records lower cross-track error than
every other method — 0.182 against 0.292, 0.192 against 0.318, 0.244 against
0.353 on three of six seeds, and within 0.012 on the rest. The pattern follows
the scenario's DVL misalignment draw: a large residual mounting rotation produces
a steady lateral dead-reckoning drift, and correcting it needs the optical fixes
that a hard innovation gate rejects and covariance weighting admits at reduced
weight.

This is a real result about gating versus weighting under systematic velocity
error, and it belongs in the discussion. The proposed manager is bit-identical to
the fixed policy on every one of those seeds, so falsification condition F2 —
which is a statement about the manager, not about every comparator — is not
violated.

---

## 0b. Twelve further defects found on 31 July 2026

All found by running code that had never been executed, or by measuring a
quantity that had only ever been asserted. Every one is corrected in the source
and covered by a regression test. They are recorded because the pattern matters
more than any individual bug: **each was a thing that looked implemented and did
nothing**, and none was visible to a passing test suite.

### The manager could not see its own strongest decision axis

Six defects formed a chain, each hiding the next. They were found by asking why
the compound scenario failed, and the answer turned out to be six layers deep.

| # | defect | evidence |
|---|---|---|
| 5 | `innovation_exceedance_rate` was never computed | 0.0 in every run of the study; also silently disabled the mode-classification branch that reads it |
| 6 | `covariance_growth_rate` was a two-valued constant | `0.0 if aided else 0.05` — identical in still water and a strong current |
| 7 | the acoustic technique was absent from the objective | neither `_predicted_availability` nor `MissionCosts.evaluate` read `config.acoustic`; the manager chose `single_beacon` in **800 of 800** decisions of an instrumented compound-scenario run (single seed), on the axis whose mean position error over the static sweep spans **6.9×** (USBL 0.244 m, LBL 0.307 m, single beacon 1.686 m, each a mean over 36 configurations and all 150 scenarios) |
| 8 | the acoustic floor was borrowed from the optical channel | claimed sigma ~= 0.15 m, i.e. 0.02 m^2, against a **median** USBL position error of 2.093 m in the compound family (36 configurations) — roughly **70× optimistic in variance**, which is why the manager declined to descend for optical recovery |
| 9 | acoustic staleness read a present-tense observable as a counterfactual | `acoustic_fix_age_s` reports whether the technique *in use* just delivered; it was gating all three techniques, making them identical in **696 of 800 decisions (87%)** of one instrumented compound-scenario run |
| 10 | **the selection was discarded one line after being made** | the rebuild that attaches the mission action omitted `acoustic` and `fusion`, so both reverted to dataclass defaults on every decision |

Defect 10 was the keystone. The manager had been selecting USBL correctly the
whole time: at the decision inspected, the top-ranked candidate was
`camera_offaxis+usbl@3.0m` at objective 0.6490, and the configuration returned
was `single_beacon`. Three separate attempts to fix the value model upstream
failed because the result was being thrown away downstream.

Compound-scenario position error fell from **11.578 m to 2.299 m** once the axis
was live (mean over the family's ten seeds), and the manager began genuinely
selecting across techniques — USBL 560 decisions, LBL 176, single beacon 59 in
the same instrumented single run that had previously shown 800/800.

⚠️ **Aggregation labels are given because their absence is itself a defect this
project committed.** In the companion-paper analysis, figures taken from one run
were reported as corpus aggregates and per-run medians were described as pooled
rates; an independent check caught both. Every count above comes from a single
instrumented run and is labelled as such, and every rate states what it is
averaged over. A number whose aggregation is unstated cannot be checked.

A seventh, related: the configuration switching margin was an absolute 0.20 m^2,
calibrated when the projection was dominated by an unaided branch of order
1.6 m^2. Once the acoustic channel entered the objective the whole projection
moved to order 0.1 m^2 and the same margin silently froze **every** decision. It
is now proportional, and therefore scale-free.

### The metric credited surveys that never happened

`MissionEvaluator` counted a waypoint as surveyed on **horizontal distance
alone**, ignoring depth. A vehicle at any altitude directly above a waypoint was
credited with having surveyed it.

This was latent until the terminal self-preservation action was added, at which
point a vehicle that abandoned the survey, ascended seventeen metres and drifted
across the area on GPS scored **1.000 coverage and zero mission failures**. Fixed
by requiring the vehicle to be within the optical reach the study already
declares. The correction moves results **against** the proposed method — in the
affected fixture, coverage 1.000 -> 0.750 and mission failure 0.000 -> 1.000 —
which is the direction an honest correction to a favourable metric must move.

### The ROS demonstrator had never been executed

The title has claimed a ROS 2 / Gazebo demonstration since the first draft. The
node graph compiled and imported cleanly and had never once been run. Running it
produced four defects in sequence:

| # | defect | evidence |
|---|---|---|
| 11 | odometry read as world-absolute | Gazebo publishes pose *relative to spawn*; a vehicle at survey altitude reported 20 m instead of 3 m |
| 12 | world and mission disagreed on the survey origin | spawn `(0, 0, -17)` against first waypoint `(-10, -9, -17)` — a **13.45 m** offset reported as position error for the whole run |
| 13 | acceleration differentiated by the wrong interval | 50 Hz odometry divided by the 10 Hz control period, ~5x wrong, feeding every prediction step; covariance fell **61 -> 2.01 m^2** when corrected |
| 14 | `VelocityControl` will not move the vehicle vertically | commanding `{x: 0.2, z: 0.3}` over gz transport gives x = 0.19999999999953 and z = 0.0 **exactly** |

Defect 14 is **not fixed**. Buoyancy conflict, joint and gravity overrides, the
ROS bridge, and link-level versus model-level control were each tested and ruled
out. The remaining fix is to drive the vehicle kinematically by setting model
pose, which matches the fidelity the world file already declares. Until then the
demonstrator shows the horizontal survey, live camera-derived optical quality and
mode selection, but **not the altitude action**.

None of this touches the paper's evidence: the campaign does not go through
Gazebo, and the Gazebo world contributes no numbers.

### What this says about the method

Nothing, directly — every defect was in instrumentation, plumbing or scoring
rather than in the mode-aware idea. What it says about the *study* is that a
passing test suite certified a manager that could not see its strongest decision
axis, and a metric that rewarded surveys conducted from the surface. The
regression tests added alongside each fix assert that the axis *moves the
outcome*, not merely that it is present, because presence was never the failure
mode.

---

## 0c. Four further defects, 1--2 August 2026 --- all self-inflicted

> **On the numbering.** Sections 0a and 0b record four and twelve defects
> respectively, and section 0c resumes at 16 rather than 17: the individual
> entries in 0b are grouped thematically rather than numbered one by one, and the
> sequence was picked up off by one. The count is twenty-two across the four
> sections (4 + 12 + 4 + 2); the labels 16--21 are identifiers, not a running
> total. Left as it is rather than renumbered, because the entries are referenced
> by those labels elsewhere.

Recorded separately from section 0b because these were introduced *by the
repair work itself*, and the pattern in them is different. The twelve in 0b were
latent in code that had never been exercised. These four were created while
fixing those, and three of them are the same mistake: **an edit that landed
somewhere other than where it was believed to land.**

### 16. Duplicate method definitions shadowed the live code

Scripted edits produced a duplicated block of roughly two hundred lines in
`manager.py`. Python binds the *later* definition, so four methods --
`_acoustic_reachable`, `_acoustic_floor_m2`, `_mission_action`,
`_fix_opportunity` -- existed twice, and every edit to the earlier copy changed
nothing at run time.

**Three verification cycles were run against code that was never executing**, and
conclusions drawn from each. The criterion under test was inspected by reading
the file, which showed exactly what was intended; only `inspect.getsource` on the
*loaded object* revealed that a second definition was winning.

A further edit compounded it: `lines[nxt:]` with `nxt` unset evaluates to
`lines[None:]`, which Python returns as the whole list rather than raising, so
the entire file was appended to itself -- 2,408 lines with three copies of one
method. Recovered by locating the intact original inside the duplication.

### 17. The blackout criterion was unsatisfiable

Replacing the too-eager channel-silence test with geometric reachability produced
a condition that could never be true: the LBL array spans the survey area by
design, so *some* acoustic technique is always reachable in principle. Terminal
self-preservation was never commanded once across 150 runs, and the tier-3
ablation was consequently bit-identical to the full manager.

Both criteria were wrong in opposite directions, and neither asked the right
question. "Could a transponder answer?" and "did the last one answer?" are
proxies for what actually matters: **has the position estimate degraded past the
point where the survey means anything?** The criterion is now the trace of the
position covariance against a threshold derived from the mission's own survey
tolerance, and it is self-clearing -- any accepted fix drops it below threshold.

### 18. The spent-hold early return preceded the blackout assessment

Once the hold budget expired the function returned immediately, so the blackout
dwell never accumulated and the vehicle could not escalate however far its
estimate had drifted. A spent hold means "stop waiting for a fix", not "stop
judging whether you can still navigate"; the second question outranks the first
and must be asked before the first can end the evaluation.

### 19. The terminal action was not terminal

Tier-3 decisions are re-evaluated every tick, so a momentary improvement dropped
the mode out of the critical state, reset the dwell, and returned the vehicle to
surveying. Measured in the compound family: surfacing was commanded at
t = 103.5 s and held for **two seconds**, against an ascent requiring
**sixty-six**. The vehicle decided to preserve itself roughly thirty times per
run and arrived never -- reported as *commanded* in 30% of runs and *reached* in
0%.

Latched once committed. A vehicle that has concluded it cannot navigate does not
resume surveying because one fix arrived; by that point the survey is already
abandoned and reversal leaves it mid-water with neither a survey nor a recovery.

### What was added to stop this recurring

`test/test_terminal_action_fires.py` asserts outcomes rather than presence: that
the action fires on a degraded estimate, that the criterion is *reachable at
all*, that it stays committed through a recovery, that the A2 ablation removes
it, and that **no method in the manager is defined twice**. That last test would
have caught defect 16 in seconds.

The general lesson, stated plainly because it was learned three times in one
evening: **verifying the file is not verifying the behaviour.** Every claim in
this record that rests on reading source rather than executing it should be
treated as unconfirmed.

---

## 0d. Two defects in the demonstrator, and one limitation withdrawn, 2 August

Found while closing out the demonstrator before the freeze. None of these
touches a reported number — the demonstrator produces no statistics — but two
of them had been written into the paper as limitations, and one of those was
not true.

### 20. The world file is not well-formed XML

`worlds/mode_aware_survey.sdf` contained six `--` sequences inside XML comments,
which the specification forbids. Gazebo's parser accepts them; Python's `expat`
rejects the file outright at line 48. The world had therefore never been opened
by any strict parser, which is also why nothing had ever checked the scene
against the mission it depicts. Replaced with em dashes.

The interesting part is not the defect but what it was hiding: the moment the
file parsed, a test could compare the SDF against `SurveyMission`, and that test
is now what holds the two together.

### 21. Estimated optical quality reads zero for the first ten frames

Before the first camera frame arrives, the feedback node publishes its initial
value of zero and the status display shows it as a quality reading. It is
indistinguishable on screen from a genuinely black scene. Ten samples of 743 in
a 300 s run, all at the start, all before rendering has produced anything.

Related and fixed: the covariance trace reaches the 1e-5 range under continuous
aiding and the fixed-point display format rendered that as `0.0000`, which looks
identical to an estimator that has stopped publishing. Small values now print in
exponent form.

### Withdrawn: "scene lighting does not cover the whole survey area"

This claim was in the package README and in the paper's limitations. **It is
false.** It was observed while the vehicle still spawned at the world origin
rather than at the mission's first waypoint, so the frames that read zero were
taken from outside the survey area, not from an unlit part of it. The spawn
defect is recorded in §0b; the lighting claim was collateral from it and was
never re-checked after that fix.

Measured on 2 August over a 300 s headless run that traversed waypoints 1--7,
the full 20 m x 18 m area:

| Quantity | Value |
|---|---|
| Quality samples after the first frame | 743 |
| Minimum estimated quality | 0.177 |
| Mean | 0.324 |
| Maximum | 0.468 |
| Samples reading zero | 0 |
| Position error, mean / max | 0.163 m / 0.340 m |

The paper's limitation has been replaced with the measurement. This is the
first entry in this record that *removes* a stated weakness, so it is worth
being explicit about the direction of the error: for several days the paper
disclosed a defect that had already been fixed, on the strength of an
observation nobody had repeated.

### What was added to stop this recurring

`test/test_demonstrator_scene.py` parses the SDF and asserts that the vehicle
spawns at the mission's first waypoint, that every waypoint is inside the lit
volume, that the launch file sources the spawn position from the mission rather
than repeating the literal, and that the scene's own fog is a minor term beside
the water model the study varies. The first of those would have caught the spawn
defect, and the file could not have been written at all before defect 20 was
fixed.

`scripts/capture_demonstrator_figure.py` drives a running demonstrator through
three water conditions and records what the feedback node estimated from each
frame: 0.300 at c = 0.2, 0.017 at c = 0.8, 0.000 at c = 1.6, with the inferred
mode moving from optically degraded to optically lost. That the estimate falls
to zero at high turbidity is also the answer to a fair objection about the
figure — if the vehicle's own lamp geometry dominated the frame, near-field and
self-illuminated, no amount of water would take the estimate to zero.

---

## 0e. One protocol deviation, found after the freeze, 3 August

Found while auditing `EVALUATION_METRICS_SPEC.md` against the code, with the
held-out campaign already running. Recorded here rather than fixed, because the
tree is frozen and fixing it would invalidate the record the held-out execution
was gated on.

### 22. The aggregate's normalisation constants were never pre-registered

`EVALUATION_METRICS_SPEC` §1.1 requires the normalisation constants of the
aggregate outcome `J` to be *"computed from development data only"* and
*"written into the freeze record"*. Neither happened. `aggregate_outcome`
derives them at runtime from whatever rows it is handed, and `run_campaign.py`
calls it without passing any, so the held-out campaign will normalise `J` on
held-out data. Its own docstring asserts the opposite, which is how the
divergence survived: the file said the right thing while the call site did the
other one.

**What is and is not affected.**

The weight vector *is* cryptographically fixed — it is `analysis.DEFAULT_WEIGHTS`,
equal weights on all three components, and `analysis.py`'s SHA-256 is in the
freeze record. Only the normalisers were left to runtime.

Normalisation is a positive rescaling applied identically to every policy, so it
cannot reverse a comparison between two policies on a single component. It can
change the relative contribution of the three components and therefore the
ranking of the composite. So it matters for `J`, and it matters for `C1`, which
is selected by minimising `J` over the sweep.

**Tested, not assumed.** On the development sweep, `C1` was selected under both
the sweep-self normalisers and the development-comparator normalisers:

| Normaliser source | C1 | J |
|---|---|---|
| Sweep, self-normalised (what the code does) | `camera_coaxial+usbl@3.0m/0.25mps/gate/continue` | 1.6748 |
| Development comparators (what the spec asks for) | `camera_coaxial+usbl@3.0m/0.25mps/gate/continue` | 1.3999 |

Identical winner, and the top four are identical in both orderings. This also
explains a long-standing cosmetic discrepancy: the campaign log prints C1's
`J = 1.675` while Table 1 of the paper reports the fixed policy at `J = 1.400`.
Both are correct; they are the same policy scored on two rulers.

**What was done about it.** `results/DEVELOPMENT_NORMALISERS.json` records the
development constants, the SHA-256 of the campaign file they came from, and an
explicit statement that this is a post-freeze derivation from pre-freeze data
and *not* a pre-registered constant. `experiments/analyse_held_out.py` reports
the held-out aggregate twice: `J_self` as the campaign log computes it, and
`J_dev` on the development ruler, which is the only version comparable to the
numbers already in the paper. Any claim that held-out `J` moved relative to
development `J` uses `J_dev`.

The runs themselves are untouched. Normalisation is applied when scoring, not
when simulating, so the held-out campaign remains valid and the correct
statistic is recoverable from its output.

**The lesson, which is the same one as defect 16.** A docstring is not a
contract. This is the second time in this project that a file stated a property
its own call sites did not honour, and both times the statement was believed
because it was written down near the code. The check that caught it was reading
the specification and the call site side by side, which nothing automated does.

### 23. Three pre-registered reporting obligations were not being met

`PROTOCOL` §6.1b, declared 31 July 2026, fixed three rules about the instability
of `C1` under the aggregation statistic: keep `J` defined with the mean, report
**both** `C1` candidates and evaluate the proposed method against each, and
report the instability itself as a finding. The manuscript did the first and
none of the other two — the median-selected baseline appeared nowhere.

Recomputed on the v5 sweep, both scored on the pre-registered mean aggregate:

| Selected under | Configuration | J |
|---|---|---|
| Mean (pre-registered) | `camera_coaxial+usbl@3.0m/0.25mps/gate` | **1.400** |
| Median | `camera_coaxial+usbl@1.0m/0.50mps/weight` | 2.330 |
| Proposed | — | 1.994 |

The proposed method loses to the first and beats the second.

**This does not rescue F1 and must not be presented as though it did.** `J` is
defined with the mean, so the mean-selected configuration *is* `C1`; the
median-selected one ranks 58th of 108 under `J`, and beating a baseline that is
58th is a weak statement beside losing to the one that is first. The obligation
to report it was fixed before the campaign precisely so that a favourable number
found later could not be adopted as though it were the headline.

Two details differ from what §6.1b recorded on 31 July, and the difference is
the defect fixes in between. The identities have moved — §6.1b names
`lidar+single_beacon@1.0m/0.25mps/weight` as the median choice, where v5 gives
`camera_coaxial+usbl@1.0m/0.50mps/weight` — and so has the rank, 41/108 then
against 72/108 now. **The finding itself is unchanged and if anything stronger:**
each candidate still ranks poorly under the other's statistic, and they still
differ on multiple axes. The stale identities are left in `PROTOCOL` rather than
edited, because a pre-registration that gets updated to match later results is
not a pre-registration.

Added to the paper as Section 5.2 with `tab:c1`.

### 24. The paper claimed the scenario families were fixed before any result

The protocol section asserted that *"the aggregate primary outcome, its
weighting, the scenario families and the falsification conditions were fixed
before any result existed."* The first, second and fourth are true. **The third
is false**, and `PROTOCOL` §5 says so plainly two pages earlier: eight families
were declared originally, and seven were added during development — the current
axis `E9`–`E12`, the acoustic-noise axis `E13`–`E15`, and `E8`, which §5.1
records was added *"after the first development campaigns could not separate the
comparators."*

The additions are defensible and were made on development data, which is what
development data is for. Claiming they were pre-registered is not defensible,
and it is the sort of overstatement that discredits the parts of the record that
are true.

Replaced with a section that separates the two kinds of quantity. The families
define **where** the comparison is made and were extended, on development data,
with published structural reasons. The aggregate, the weighting and the
falsification conditions define **what counts as winning**, and those were fixed
first and left alone — including when F1 turned out to be triggered.

Also corrected: the paper said the protocol records its amendments *"with
dates"*. Exactly one amendment carries a date (§6.1b, 31 July). The family
extensions are described but not dated. Now stated as the weakness of the record
that it is.

### 25. The pre-registered decisive case was reported only as a control

`PROTOCOL` §5.1 nominates `E8` as the case the study turns on, with an argument
settled before the campaign that does not reference which method wins: a
capability manager can only contribute where a capability change is both needed
and available, and `E8` is the only cell where both hold. The manuscript
mentioned `E8` in one row of the per-family table and as the *control* column of
the dwell-sensitivity table, and never reported it as the decisive case at all.

The result, which had been sitting in the campaign output unreported:

| Policy | Cross-track (m) | Failed | Coverage |
|---|---|---|---|
| Proposed | **0.083** | **0.00** | **1.000** |
| Fixed C1 | 0.104 | 0.00 | 1.000 |
| Covariance only | 28.073 | 0.40 | 0.900 |
| Ablation A1 (tier 1 only) | 55.415 | 0.80 | 0.762 |
| Dead reckoning | 61.190 | 1.00 | 0.600 |

Three orders of magnitude between the proposed method and the
measurement-weighting comparators, in the family the protocol designated for the
question, delivering the same verdict as F4 does in `E7`. Tier 3 contributes
nothing here — `A2` matches the full manager exactly, because `E8` is
recoverable and the terminal action correctly declines to fire. Added as
Section 5.4 with `tab:decisive`.

### 26. A comparator chatters, and rule R5 requires saying so

`residual_only` switches optical channel **38.6 times per run on average, up to
185**, against 0.13 for the proposed method. It has no hysteresis by
construction, so it oscillates under fluctuating quality. `COMPARATOR_SPEC` R5
requires a pathological comparator to be diagnosed and fixed before the freeze
or reported as a limitation; it was neither.

Reported rather than repaired, and the reasoning is stated in the paper: part of
the proposed method's margin over `residual_only` is the cost of that
oscillation and a reader should discount it, but chattering is the honest
consequence of the design that comparator represents, and removing it would mean
granting it the hysteresis whose value is part of what this paper claims.

---

## 0f. The held-out campaign, 3 August 2026 — two findings retracted

Executed once, against a verified freeze record, on seed root 20,400,000 with
20 seeds per family: 32,400 sweep runs and 2,400 comparator runs. The block was
marked spent at 06:15 UTC with the results digest recorded. Seed sets verified
disjoint.

**F1 is triggered, as on development.** `J` 2.173 against the tuned fixed
policy's 1.554, with the ordering of all eight policies preserved exactly.
Protocol contingency X3 applies and the paper is now a characterisation and a
negative result. Nothing was retuned, no seed was reselected.

### 27. The fourteen-of-fifteen family advantage did not replicate

Development: the proposed method had lower cross-track error than the fixed
policy in **14 of 15** families, by 6–34%. Held-out: **7 of 15**, and in the
twelve non-discriminating families 5 of 12, with margins of roughly ±10% in both
directions.

Checked and ruled out: this is not the baseline changing identity between
blocks. Recomputed on held-out data against the *development*-selected
configuration, the count is also 7 of 15. The development margins in the flat
families were a property of those seeds.

### 28. The ~40% bracket-recovery claim did not replicate, and is withdrawn

Development: mean recovery of 0.433 of the hindsight-to-clairvoyance gap across
the families it won, reported in the paper as *"recovers roughly 40% of the
distance between a policy tuned with complete hindsight and one handed the
truth"* — the central quantitative claim of the draft.

Held-out: **−0.09** over non-degenerate families, with 7 of 15 negative. On
average the manager sits marginally on the wrong side of the fixed baseline.

The figure is withdrawn. No restatement of it survives anywhere in the paper.
This is the single most valuable thing the held-out block bought, and the
clearest available argument for reserving one: the claim was in the manuscript,
in bold, in the abstract slot, and it was wrong.

### 29. `EPS_NOMINAL` was never defined, so F2 cannot be decided

`PROTOCOL` §1.1 defines falsification condition F2 as nominal performance
degrading *"by more than the tolerance `EPS_NOMINAL` fixed at freeze"*. **The
tolerance appears nowhere** — not in the protocol, not in the metrics
specification, not in the source. It is referenced once and defined never.

It matters now, where it would not have mattered before, because the sign
flipped: nominal cross-track is 17% *better* than the fixed policy's on
development (0.0794 m against 0.0958 m) and 10% *worse* on held-out (0.1099 m
against 0.1003 m), with identical coverage and zero failures on both.

Reported as **undecidable**, not as passed. Any threshold chosen now would be
chosen after seeing the number, which is the thing pre-registration exists to
prevent — and a 10% regression is small enough that a tolerance picked today
would almost certainly be picked to clear it.

### What did replicate

| Result | Development | Held-out |
|---|---|---|
| F1 | triggered | triggered |
| F4 | not triggered | not triggered, larger margin |
| Decisive family E8, proposed | 0.083 m, 0% failed | 0.103 m, 0% failed |
| E8, tier-1-only ablation | 55.4 m, 80% failed | 64.1 m, 90% failed |
| Discriminating families won | 2 of 3 | 2 of 3 |
| Mission time vs fixed | 54% | 54% |
| Productivity vs fixed | 2.2x | 2.12x |
| Terminal action selectivity | E7 only, 3/3 surfaced | E7 only, 5/5 surfaced |

The system-level claim — that the contribution is in the guidance and mission
tiers rather than in measurement weighting — is the one that survived, and it
survived on the family the protocol nominated for it in advance.

### 30. I compared two different statistics and it favoured the method

Caught in the final verification pass, after the held-out rewrite was already
written and committed.

Section 5.10 claimed F3's tail component had *reversed* on held-out data — 12.15 m
for the proposed method against 72.79 m for the fixed policy, "six times better".
That was the **95th percentile** of per-run maximum cross-track, compared against
a development figure (6.505 m against 4.023 m) that was the **mean**. Two
different statistics, presented as one comparison, in the direction that
flattered the method.

On the consistent statistic the result is the opposite and is entirely
unremarkable:

| Campaign | Proposed | Fixed | Adverse by |
|---|---|---|---|
| Development, mean peak | 6.505 m | 4.023 m | +62% |
| Held-out, mean peak | 7.564 m | 4.481 m | +69% |

F3's tail component is adverse on both campaigns, by similar margins. Corrected
at all three sites.

The p95 inversion is real but it is not about the method: the held-out-selected
baseline fails 80% of compound runs, so its worst runs are catastrophic and its
upper percentile moves from 1.59 m on development to 72.79 m on held-out.
Quoting it would have reported a *baseline collapse* as a *method gain*. The
paper now carries this as an explicit warning rather than silently using the
mean.

Worth recording plainly: this is the same class of error as the "78 m" exclusion
in defect 4 of this session's manuscript audit, made by me, four hours after I
had written that one up. The mechanism was not carelessness about arithmetic —
every number was correct — it was pulling a figure from an analysis script whose
statistic differed from the manuscript's without checking which. The guard that
caught it was recomputing every held-out claim from the CSV rather than from the
script's output.

### 31. The hindsight baseline is stronger than expected, which is a finding

The held-out sweep selects a **different** C1: `camera_coaxial+lbl@3.0m/0.25mps/
weight` against development's `camera_coaxial+usbl@3.0m/0.25mps/gate`. Different
acoustic technique, different fusion mode.

But the development-selected configuration, evaluated on held-out data, ranks
**3rd of 108** and costs 3.2% more `J` than the held-out optimum. Its identity is
unstable; its performance is not. The tuned fixed policy is therefore not an
overfitting artefact, which makes losing to it a *more* meaningful loss, and is
recorded here because the temptation is to read baseline instability as baseline
weakness.

---

## 0g. Study 2 development, 4 August 2026 — three defects, all latent in study 1

Found while adding terrain-relative navigation. All three were present in the
frozen study-1 artefact and none was caught by a test suite that stayed green.

### 32. The switch-margin hysteresis was partially inert

`_is_incumbent` identified the configuration in use by optical channel, altitude
and speed — and ignored the acoustic and fusion axes, which had been added
during the E7 defect chain. With three acoustic techniques and two fusion modes,
**six distinct candidates answered to "the incumbent"**, and `incumbent_objective`
became whichever the search loop happened to evaluate last.

It stayed hidden while all six carried finite objectives: the incumbent's score
wobbled between ticks but never enough to defeat the 15% switch margin. Adding a
fourth technique whose objective is infinite when its terrain is unavailable made
it visible at once — a comparison against an infinite incumbent is true for every
rival, so the margin permitted a switch on every decision and the optical channel
oscillated at 1 Hz, 24 swaps in 30 s against a limit of 6.

**Study 1's mode-chatter statistics were computed with this defect present.**
They were low (3.17 transitions per run) so the reported conclusion does not
change, but the mechanism that was supposed to produce them was not fully
working.

### 33. `acoustic_fix_age_s` was a flag, not an age

The observable reporting "seconds since the last accepted acoustic measurement"
took exactly two values: `0.0` when a fix arrived on the current tick and `60.0`
otherwise. Acoustic fixes arrive once per interrogation cycle of 2–6 s while
decisions are taken every 0.5 s, so **a perfectly healthy channel reported
maximal staleness on three ticks in four**.

Measured, once a rule finally read it as an age: **74.9% of nominal-family
decisions** reported the acoustic link silent. Corrected to a real elapsed time,
that falls to **0.0%** in nominal and **4.7%** in the compound family, where it
tracks the actual outage window.

The consequence for study 2 was large and immediate: compound-family cross-track
error fell from 8.26 m to **1.11 m** on the correction alone. In study 1 nothing
read the field as an age, so the defect was inert — which is exactly why it
survived a campaign, a freeze and a held-out execution.

### 34. The acoustic outage disabled the echo sounder

`FaultKind.ACOUSTIC_OUTAGE` returned "no measurement" for every technique before
the technique was examined. The fault models loss of the transponder link — a
failed beacon, a blocked path, a departed vessel — none of which affects a
hull-mounted echo sounder looking straight down at three metres.

With terrain matching added, this disabled it in precisely the family it was
added for, and the manager correctly never selected it: an axis present in the
action space and dead in the only place it could have mattered. The same defect
class as the acoustic axis in §0b, found the same way — by asking why a
capability that should have been used was not being used.

### What was added to stop these recurring

`test_action_space_is_live.py` now asserts that all three infrastructure
dependency classes are represented in the candidate set, so a technique whose
failure mode duplicates another's cannot be added without noticing. The
anti-artefact test in `test_pipeline.py` now counts **infrastructure** as a
mission currency: the manager consumes roughly four times the infrastructure the
fixed policy does (0.201 against 0.050 in the cost model's units), and reading
that as "the improvement came for free" was the test's own blind spot rather than
a defect in the method.

### A note on the rate

Thirty-four defects now, of which nine were found in the two days after the
study-1 held-out block was spent. The rate is not falling, and the honest reading
is not that this study is unusually defective but that most studies are not
looked at this hard. Every one of the nine was found by executing something, by
reading a specification against its implementation, or by asking why a capability
that should have been used was not being used. None was found by a test.

---

## 0h. Three defects found by campaign v6, 4 August 2026

All three predate study 2. Campaign v6 ran to completion and was then discarded
without being reported, because two of the three invalidate it.

### 35. Waypoint capture had no along-track check

Capture advanced the waypoint index on proximity alone: the estimate within
0.5 m of the target. A vehicle whose position error exceeds the capture radius
can pass the waypoint without its estimate ever entering that circle. The index
never advances, line-of-sight guidance keeps steering along a leg already
finished, and the vehicle flies that heading until the mission times out.

Measured on `E8` seed 20001009: two legs completed normally, then x held at
9.9 m while y ran from 3.6 m to **169.7 m** over five minutes — 160 m beyond a
survey box 18 m across — with its own position estimate accurate to about a
metre throughout. It knew where it was the entire time.

Two runs of 190 behaved this way, and those two carried the whole aggregate gap
to `C1`. Corrected by advancing when the projection onto the leg passes its far
end, which cannot be defeated by a poor estimate the way a proximity test can.
Part of the shared guidance law, so it applies to every policy equally.

**This defect inflated study 1.** Dead reckoning at 73 m in the compound family
was substantially the vehicle leaving the survey area, not navigation error.
Corrected, dead reckoning fails the same families at 2–8 m. The failure *rates*
that define discrimination are unchanged and six families still discriminate,
but the headline magnitudes in study 1 should be read as partly an artefact of
this.

### 36. The prior map was a cost, not a condition

Terrain-relative navigation was modelled with its prior bathymetric map priced
as an infrastructure cost. That understates the dependency: a technique whose
infrastructure is merely expensive is available everywhere. All ten best fixed
configurations in v6 used terrain matching, worth **+1.018 J** to a static
policy, and `C1` became a terrain configuration — the dominance `E16` was built
to prevent and did not.

The map is now a property of the area, as the surface asset and the transponder
array already were. `E19` is an unprepared area: nobody laid transponders there,
so nobody surveyed it well enough to chart it either. The relief is present; the
chart is not.

### 37. The admission axis was invisible to the objective

`config.fusion` appeared in three places: the configuration's name string, the
rebuild that attaches the mission action, and the incumbent test. It was read
**zero** times in the value model, the cost model and the availability model.

Gating and weighting therefore scored identically, `argmin` returned whichever
the candidate list offered first, and the manager selected gating in **4,631 of
4,631 decisions** across twelve hard runs. Half the action space was unreachable
in practice — while the static sweep's best configuration used weighting. The
method was handicapped against its own baseline.

This is defect 7 on a different axis, and it survived for the same reason:
`_combined_uncertainty` repeated the acceptance mixture inline while
`_projected_uncertainty` held an identical copy that nothing called. A term
added to the copy would have changed nothing. There is now one implementation.

Priced from the physics the estimator's own docstring already stated, and now
selective: 22 of 2,328 decisions, concentrated in the family where the gate is
actually rejecting measurements. Where exceedance is near zero, gating loses
nothing and weighting has no upside.

### Still dead, recorded rather than fixed

`return_to_last_good_fix` and `abort_leg` are selected in no run examined.
`surface_for_gps` no longer fires at all, because the guidance defect that
created unrecoverable states is fixed. The action space therefore contains three
tier-3 actions with no evidence behind them, and the manuscript should not claim
otherwise.

### The question that found two of these

*Which actions does the manager actually select?* Counting selections per axis
across a dozen hard runs took two minutes and found a dead half of the action
space that five campaigns had not. It should be run before every freeze.

---

## 1. Result summary, campaign v5

150 scenarios across 15 families, 8 policies, 108 static configurations swept to
select the fixed baseline. Development seeds only.

| policy | failed | coverage | cross-track (m) | position error (m) | speed (m/s) | J |
|---|---|---|---|---|---|---|
| oracle *(not deployable)* | 0.01 | 0.996 | 0.247 | 0.291 | 0.443 | 0.236 |
| **fixed C1** *(hindsight-tuned)* | 0.05 | 0.965 | **2.108** | 0.122 | 0.249 | **1.400** |
| **proposed** | 0.06 | 0.968 | 3.441 | 0.252 | 0.490 | **1.994** |
| ablation A2 (no tier 3) | 0.06 | 0.968 | 4.707 | 0.253 | 0.497 | 2.176 |
| covariance only | 0.09 | 0.965 | 6.712 | 0.337 | 0.497 | 3.091 |
| ablation A1 (tier 1 only) | 0.12 | 0.949 | 8.746 | 1.236 | 0.497 | 4.160 |
| residual only | 0.12 | 0.945 | 9.090 | 1.215 | 0.497 | 4.262 |
| dead reckoning | 0.20 | 0.920 | 13.763 | 0.838 | 0.498 | 6.680 |

**On the pre-registered aggregate the proposed method does not beat C1**: 1.994
against 1.400. That is the primary declared outcome and it is reported as such.

It beats every other deployable comparator, and by margins that are not marginal:
1.9x on cross-track against covariance-only, 2.5x against tier-1-only, and 4x
against dead reckoning.

---

## 2. The aggregate is decided by one family in fifteen

The proposed method has lower cross-track error than C1 in **14 of 15 families**.

| family | proposed | C1 |
|---|---|---|
| E1--E6, E8--E15 *(14 families)* | 0.072 -- 0.120 | 0.096 -- 0.182 |
| **E7 compound** | **50.384** | **30.111** |

Fourteen families sit near 0.09 m and differ by 15--35% in the proposed method's
favour. One family produces errors around 500 times larger, and a mean over that
distribution reports the largest term. Both facts are true simultaneously and
neither is the whole picture, which is why both are reported.

`PROTOCOL.md` section 6 fixed the aggregate before any result existed. It has not
been replaced now that it is unfavourable, and section 6.1b records the specific
temptation: a robust statistic reverses the headline in the proposed method's
favour, and adopting it after observing that would be indefensible.

---

## 3. Within the family it loses, it is still second of eight

E7 pools every failure mode the study models: turbid water and loss of velocity
aiding at the same time.

| policy | cross-track (m) | position error (m) | failed |
|---|---|---|---|
| oracle *(clairvoyant)* | 2.84 | 3.04 | 0.20 |
| fixed C1 | 30.11 | 0.68 | 0.70 |
| **proposed** | **50.38** | **2.28** | 0.90 |
| ablation A2 | 69.38 | 2.30 | 0.90 |
| covariance only | 71.36 | 1.58 | 0.90 |
| dead reckoning | 73.63 | 3.66 | 1.00 |
| ablation A1 | 74.63 | 14.06 | 1.00 |
| residual only | 74.63 | 14.06 | 1.00 |

Two things are worth separating. The proposed method is **second of eight** here,
ahead of every estimator-only comparator by 30% or more, and its *estimate* is
six times better than the residual-only and tier-1-only comparators (2.28 m
against 14.06 m). What it loses is the comparison against a policy that keeps
surveying and mostly fails while it recognises the situation and stops.

The oracle fails 20% of E7 runs while being given the true water state. That is
the clearest available evidence that part of this family is not a policy problem
at all.

---

## 4. Falsification condition F4 is answered

F4 states that if the tier-1-only ablation matches the full manager, the
contribution belongs to measurement weighting and the paper has no system-level
claim.

| | cross-track in E7 |
|---|---|
| ablation A1 (tier 1 only) | 74.63 |
| ablation A2 (no tier 3) | 69.38 |
| **full manager** | **50.38** |

Tier 1 alone is barely better than dead reckoning (74.63 against 73.63). Adding
tiers 1--2 recovers to 69.38. **The full manager reaches 50.38, so tier 3 alone
accounts for a 27% improvement.**

This is the first campaign in which that separation exists. In every earlier run
`proposed` and `ablation_a2` were bit-identical, which meant mission actions were
contributing nothing -- the terminal action was implemented three times before it
ever executed (section 0c).

---

## 5. The terminal action fires selectively and completes

Surfacing was commanded in **3 of 150 runs** and reached the surface in **3 of 3**.

All three were E7. It did not fire in the other fourteen families, including
E8 (turbid water plus DVL loss) which resembles E7 but is recoverable and which
C1 completes without a single failure.

That selectivity is the point. Two earlier criteria failed in opposite
directions: channel-silence abandoned recoverable surveys on every E8 run, and
geometric reachability could never be satisfied and never fired at all. The
criterion now used -- has the position estimate degraded past the point where the
survey can be flown -- distinguishes "cannot navigate" from "this is temporarily
hard", which is the distinction the escalation ladder exists to make.

---

## 6. The speed confound, and why cross-track is not comparable here

The proposed method flies at 0.490 m/s and completes in 203 s. C1 flies at
0.249 m/s and takes 380 s. Cross-track error is not comparable across that
difference, which `PROTOCOL.md` section 6.1 fixed as a binding constraint before
these numbers existed.

A second non-comparability applies to E7 specifically and is declared in
section 5.4 of the manuscript. Cross-track measures deviation from an intended
path. Once a vehicle has *deliberately abandoned* that path to preserve itself,
the metric no longer measures policy quality -- a vehicle that has stopped
following the line has a small cross-track error for the same reason a
stationary one does. In that regime the declared score is whether the vehicle
recognised the condition and became recoverable, which is 3 of 3.

---

## 7. What remains open

- **E7 at 50.4 m with 0.90 failure is worse than C1 at 30.1 m and 0.70.** The
  escalation ladder improves it by 27% and does not rescue it. This is stated as
  a limitation, not a win.
- **The held-out root 20,400,000 is unspent.** C1 is selected with hindsight over
  all 150 development scenarios; whether it degrades on conditions it was never
  selected for, while a method that infers online holds up, is the paper's actual
  claim and has not yet been tested.
- **The static baselines have not moved across five campaigns** spanning nineteen
  defect fixes. Every change landed on the method under test and none on what it
  is measured against. C1 has been `camera_coaxial+usbl@3.0m/0.25mps/gate` at
  J = 1.675 every time.

