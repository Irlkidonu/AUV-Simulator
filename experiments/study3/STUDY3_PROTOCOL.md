# Study 3 protocol: predictive capability-aware multimodal adaptation

Status: **DEVELOPMENT DESIGN ACCEPTED; DEVELOPMENT EXECUTED — NOT FROZEN FOR HELD-OUT**  
Project: Paper 2 / platform-v2  
Date: 2026-08-11

## 1. Research question and hypotheses

**Research question.** Does predictive capability-aware multimodal adaptation
improve closed-loop navigation robustness relative to reactive
capability-aware and strong fixed strategies when navigation capabilities
change?

**Primary hypothesis.** In capability-transition conditions with observable
warning and a reachable useful action, PREDICTIVE reduces the consequences of
capability loss relative to REACTIVE without unacceptable nominal degradation.

**Secondary hypotheses.** (H2) PREDICTIVE and REACTIVE outperform the strongest
development-selected FIXED policy under nonstationary capability loss. (H3)
PREDICTIVE does not materially increase nominal intervention, mission duration,
safety violations or navigation error. (H4) PREDICTIVE has little or no
advantage in sudden/unpredictable loss and cannot restore capability in the
no-recovery boundary family. H4 is a mechanism check, not an adverse failure.

The tested causal chain is sensing/environment interaction -> measurements and
uncertainty -> capability belief -> prediction -> policy/selector -> applied
speed/altitude/mission/recovery action -> changed vehicle/sensing state ->
subsequent observations and navigation outcome.

## 2. Comparator definitions

All policies use identical sensor models, estimator, P5-v4 front end, acoustic
fixed-lag replay, action consequences, mission logic and computational budget.
They receive no scenario label, commanded degradation variable, future event or
ground truth.

- **FIXED:** one constant optical channel, altitude, speed, acoustic technique
  and fusion mode selected by the development-only sweep in Section 7. It never
  reads capability belief to reconfigure. Its estimator still accepts/rejects
  measurements normally and its terminal safety rule remains enabled; disabling
  safety would be an unfairly weak comparator.
- **REACTIVE:** the complete observable-only capability filter, current
  capability decision, selector and active recovery. Capability trends and the
  forecast are not supplied to decision logic; pre-loss action is forbidden.
  It acts only after a current usability probability crosses the same frozen
  boundary used by PREDICTIVE.
- **PREDICTIVE:** identical to REACTIVE except that the frozen ten-second
  observable trend forecast may trigger a reachable action before the current
  capability boundary is crossed.

REACTIVE and PREDICTIVE share posterior updates, thresholds, hysteresis,
selector costs, action space and action dynamics. Their only intentional
difference is permission to use the forecast before observed loss.

## 3. Scenario families

The seven transition families form the **primary analysis group**. Three
controls are reported separately and never pooled into its headline effect.

| Family | Changed capability and mechanism | Onboard evidence | Consequence without action | Reactive / predictive response | Necessity |
|---|---|---|---|---|---|
| S3_OPTICAL_GRADUAL | Increasing optical path/backscatter drives real P5-v4 from precise fixes to NO FIX | image quality trend, P5 acceptance, covariance, fix age | absolute-aiding gap and covariance growth | lower altitude/change optical channel after loss / before predicted loss | direct test of optical warning and action |
| S3_DVL_GRADUAL | Increasing altitude/terrain return weakening causes bottom-lock probability to decay before loss | lock probability, age, trend, water-track status | velocity drift; current becomes weakly observed | lower altitude or reduce speed after loss / before lock boundary | isolates predictive velocity-aiding protection |
| S3_ACOUSTIC_GEOMETRY_ASYNC | vehicle/support geometry worsens DOP while propagation, jitter and intermittent packets delay fixes | estimated position, DOP, SNR, validity/arrival times, silence | stale absolute aid and delayed corrections | reposition/hold/switch technique after invalidity / while DOP and age forecast failure | tests geometry plus asynchronous replay |
| S3_INFRASTRUCTURE_WARNING | surveyed support vessel departs gradually, then the acoustic asset becomes unavailable | estimated relative geometry, increasing range/DOP, packet silence | loss of selected acoustic technique | switch technique after silence / before support exits useful geometry | distinguishes prediction from oracle knowledge of departure |
| S3_RECOVERY | optical or acoustic aiding degrades and later returns through physically improving conditions | rising quality/lock/fix evidence | delayed reacquisition or unnecessary conservative action | confirm recovery after evidence / prepare but obey same reacquisition confirmation | tests recovery without privileged restoration time |
| S3_COMPOUND_OPTICAL_ACOUSTIC | overlapping optical loss and acoustic geometry/latency degradation | both observable streams and estimator covariance | intermittent or absent absolute aiding | reactive multimodal fallback / pre-emptive altitude/technique/hold action | tests prediction when alternatives are time-dependent |
| S3_COMPOUND_DVL_ACOUSTIC | sequential DVL loss and acoustic degradation | DVL trend, acoustic DOP/age, covariance growth | velocity drift plus sparse absolute correction | reactive fallback/recovery / preserve DVL or reposition early | tests coupled relative and absolute aiding loss |
| S3_NOMINAL (control) | stable multimodal operation | stable high-quality evidence | none | no intervention expected | measures unnecessary adaptation and regression |
| S3_SUDDEN (control) | abrupt optical or DVL loss with no observable lead | current availability changes only at loss | unavoidable transient | both adaptive policies can react; prediction should not lead | boundary against claims of foresight |
| S3_NO_RECOVERY (control) | optical, DVL and acoustic support become unavailable and no submerged action can restore them | loss, silence and covariance growth | mission failure unless terminal safety action fires | safe abort/surface only | establishes recovery boundary and safety behavior |

Scenario parameters set physical trajectories and sensor/environment mechanisms;
they never hard-code outcomes, observations, beliefs or actions. Primary
families must pass a development eligibility audit showing degradation,
advance observable signal, consequence of waiting and at least one reachable
action. A family failing that audit is repaired or removed before freeze, not
retained as non-discriminating volume.

External acoustic aiding is deployment-scoped. `S3_ACOUSTIC_GEOMETRY_ASYNC`,
`S3_COMPOUND_OPTICAL_ACOUSTIC`, and `S3_NOMINAL` contain surveyed LBL arrays;
`S3_COMPOUND_DVL_ACOUSTIC` contains USBL support; and
`S3_INFRASTRUCTURE_WARNING` begins with USBL support which subsequently departs.
`S3_DVL_GRADUAL` and `S3_RECOVERY` contain only a single range beacon, which is
not treated as an absolute position fix. `S3_OPTICAL_GRADUAL`, `S3_SUDDEN`, and
`S3_NO_RECOVERY` contain neither LBL nor USBL. Deployment labels remain
evaluator-side; policies may observe only technique-specific advertisements,
responses, geometry diagnostics, packet timing, and silence.

## 4. Closed-loop eligibility gate

Before freeze, an instrumented development trace must prove for every adaptive
action: (1) environment/sensor state changes measurement output; (2) the
diagnostic changes; (3) belief changes; (4) forecast changes when warning
exists; (5) policy selects an action; (6) actuator state changes on the next
simulation step; (7) subsequent measurements change because of that state; and
(8) navigation state/outcome responds. Merely logging an action fails the gate.

The existing `system_integration_v1` trace demonstrates this chain using the
runtime P5-v4 output adapter and applied selector actions. Study 3 is not yet
fully eligible: its runner must call the packaged image-to-fix P5-v4 algorithm,
not synthesize a P5-shaped result, and must demonstrate end-to-end equivalence
to the frozen P5-v4 confirmation implementation. This is an engineering blocker
to freeze, not authorization for new optical-method tuning.

## 5. Outcomes and estimands

### Primary outcomes

1. **Mission completion:** reached the final waypoint within the mission horizon
   with no terminal abort/surface and no safety-boundary violation (binary).
2. **Transition-window horizontal navigation RMSE (m):** RMS Euclidean error
   between estimated and true horizontal position from first evaluator-defined
   physical degradation onset until recovery or mission end. Truth is evaluator
   only and unavailable to policies. Reported conditionally among completed
   runs; completion is always reported beside it.
3. **Cumulative time without usable absolute aiding (s):** time for which neither
   an accepted P5-v4 fix nor a valid accepted acoustic absolute fix is available.
4. **Safety-violation occurrence:** any minimum-clearance, operating-envelope or
   maximum-allowed-position-uncertainty violation (binary).

The primary estimand is the family-equal paired PREDICTIVE-minus-REACTIVE effect
over the seven primary families: risk difference for binary outcomes and paired
mean difference for continuous outcomes. Family-equal weighting prevents a
large/easy family dominating. PREDICTIVE-vs-FIXED and REACTIVE-vs-FIXED are
secondary estimands. No weighted aggregate J is used.

### Secondary outcomes

Peak horizontal error; longest absolute-aiding gap; recovery time from physical
restoration to accepted aid; forecast lead time; correct pre-loss action rate;
unnecessary intervention count; mode dwell and chatter; Brier score and
reliability diagram for capability probabilities; modality availability and
use; action counts; mission duration; real-time factor and policy runtime.
Sensor, compute, hotel and propulsion energy are descriptive only and cannot
enter any policy objective or selection rule.

Non-completions are never silently dropped. Binary completion/safety analyses
include all pairs. Continuous navigation RMSE is reported both for paired joint
completers and as a sensitivity analysis assigning the physical failure-boundary
error for the post-failure remainder of the common mission horizon. Both are
shown; neither substitutes for completion.

## 6. Statistical analysis

Environmental randomness is paired by family/seed across all three policies.
Policy execution order is deterministically permuted within each pair. The
headline PREDICTIVE-vs-REACTIVE analysis uses 10,000 family-stratified paired
bootstrap resamples for continuous effects and paired risk differences with a
cluster bootstrap for binary effects. Report mean/median paired effects, 95%
confidence intervals, standardized paired effect size and per-family
win/tie/loss counts. Completion discordances also receive an exact McNemar test.

The four primary-outcome superiority tests use Holm correction at family-wise
alpha 0.05. Family-specific estimates and the three control-family analyses are
predeclared descriptive mechanism analyses with confidence intervals, not a
second multiplicity-generating claim set. Nominal non-inferiority margins are
5 percentage points for completion/safety, 0.25 m for RMSE, 2 s for unaided
time and one additional unnecessary intervention per mission.

The predictive mechanism is supported when at least one corrected primary
transition effect favors PREDICTIVE and no primary outcome or nominal control
exceeds its adverse margin. Failure to beat REACTIVE is reported as no evidence
for prediction, even if both adaptive methods beat FIXED.

The proposed 30 held-out paired seeds per primary family follow the development
planning target of detecting a practically meaningful 2 s paired reduction in
unaided time with paired SD 3.5 s: `(1.96+0.84)^2*3.5^2/2^2 = 24.0`; 30 allows
non-normality and missing paired completers. Development confirmation must
replace the assumed SD with measured paired SD and may increase—but never
decrease—the held-out count before freeze. Twenty seeds per control family are
for boundary precision, not headline power.

## 7. Development design and fixed selection

All work remains in registered Study 3 development roots.

1. **Scenario calibration:** 8 seeds x 10 families x 3 policies = 240 runs.
   Repair truth leakage, non-actionable transitions and trivial families.
2. **Strong fixed successive-halving sweep:** all 162 declared combinations
   (3 optical x 3 altitude x 3 speed x 3 acoustic x 2 fusion) receive 1 seed x
   10 families = 1620 runs; best 18 receive 4 seeds x 10 = 720; best 4 receive
   12 seeds x 10 = 480. Total 2820. Ranking is lexicographic: safety violation,
   completion, family-equal unaided time, family-equal RMSE, then mission time.
   Infrastructure-infeasible configurations remain failures rather than being
   silently excluded. Successive-halving ranks only candidates advanced from
   the preceding stage. The infrastructure-corrected development sweep at root
   31,800,000 selected `fixed_155`: lidar, 5 m, 0.5 m/s, USBL, covariance
   weighting. USBL produces measurements only in USBL-enabled contexts; FIXED
   does not oracle-switch elsewhere. The winner is frozen before held-out.
3. **Adaptive tuning:** four predeclared shared inference/recovery parameter
   bundles for PREDICTIVE across 10 seeds x 7 primary families (280), plus the
   identically parameterized REACTIVE reference across the same 70 pairs = 350.
   Selection uses the same lexicographic order; no outcome-specific post hoc
   comparator edits.
4. **Development confirmation/power:** 15 seeds x 10 families x 3 policies =
   450 runs. Estimate paired SD, confirm family discrimination, nominal margins,
   runtime/storage and freeze eligibility.

Total planned development executions: **3860**. Development may iterate before
freeze, but every correction uses a new versioned identifier and root; the
reported accounting includes all attempts.

## 8. Proposed held-out design

Reserved root `32,000,000` remains untouched. Seven primary families use 30
paired seeds x 3 policies = 630 executions. Three controls use 20 paired seeds x
3 policies = 180. Proposed total: **810 held-out executions**, one campaign
invocation after explicit authorization. Identical per-seed environment and
sensor streams are reused across policies; policy-influenced measurements then
diverge naturally through applied actions.

At the current headless integration rate the lower-bound estimate is under one
hour, but that path does not execute image-to-fix P5-v4. With P5-v4 at roughly
11--17 ms per image pair and a 0.5 s decision interval, a 120 s mission adds
about 3--5 s CPU per execution: approximately 4--7 h for 3860 development and
0.9--1.3 h for 810 held-out executions on one core, before orchestration
overhead. A real-time ROS/Gazebo campaign would instead require about 107 h and
27 h respectively. The final protocol uses a measured 30-run development
benchmark plus 30% contingency; runtime is not guessed after freeze.

## 9. Seed allocation and separation proof

Study 3 roots are: fixed sweep `31,100,000`, scenario calibration `31,200,000`,
adaptive tuning `31,300,000`, development confirmation `31,400,000`, and final
held-out `32,000,000`. Seeds derive from SHA-256 of root, family, index and stream
name; no arithmetic ranges cross roots. Workspace search finds all earlier
Study 1/2/platform-v2 roots below 30,000,000. The registry explicitly forbids
spent roots including 20,000,000/20,400,000/20,800,000 and 22.1--22.3 million.
The held-out root may appear only in design/lock records until authorized.

## 10. Freeze and one-shot evaluation procedure

After development review, generate an allowlisted freeze containing SHA-256 for
the Study 3 runner; platform-v2 package source; P5-v4 packaged implementation
and equivalence test; scenario generator and family manifest; all three policy
definitions; fixed winner; inference/prediction/recovery parameters; action
space; estimator/acoustic timing; outcome/statistics code; seed registry;
environment/package lock; and this protocol. Record Python, NumPy, OpenCV, ROS
and Gazebo versions, CPU, OS and expected result schema.

`verify-lock` must pass before execution and the runner must reject unregistered
roots, unknown policies, changed family counts, duplicate attempts and held-out
execution without a separate authorization record. Held-out runs once. An
interruption permits only same-root verified-packet resume. Results are immutable
and reported regardless of direction; no threshold, family, metric, comparator
or sample-count change follows inspection.

## 11. Truth-leakage safeguards

Policy input types exclude ground-truth pose, scenario/family identifiers,
commanded turbidity/noise/current, fault schedules, future transition time and
evaluator phase labels. Estimated pose is allowed for surveyed acoustic
geometry. Truth and physical-onset labels live in a separate evaluator process.
Tests inspect policy call signatures, serialize every policy input, reject
forbidden keys recursively, perturb hidden truth while holding observations
fixed and require identical decisions, and verify that paired policies receive
identical pre-action random streams. Forecasts must arise only from timestamped
observable history. Sudden-loss traces must have zero pre-loss forecast skill
within sampling uncertainty.

## 12. Reproducibility and provenance

Each execution records design/freeze/commit hashes; scenario and stream seeds;
policy/config hashes; action and observation traces; estimator inputs/outputs;
P5 stage metrics; DVL/acoustic validity and arrival times; runtime; termination;
and result checksum. Atomic per-run packets permit verified resume. A campaign
inventory proves expected family x seed x policy completeness and common-random-
number pairing. A clean-clone command builds the headless package, runs unit and
legacy-regression suites, verifies the lock, reproduces one development golden
trace byte-for-byte and refuses the held-out root without authorization.

## 13. Risks, blockers and fairness audit

Scientific risks: prediction may merely move interventions earlier without
improving navigation; belief probabilities are development-calibrated only;
P5-v4 has no turbid availability; action benefits may depend on transition
speed; completion may be too common for precise risk differences; and the
simulator lacks calibrated six-DOF vehicle coefficients, HIL and field evidence.
These are reported limitations, not reasons to solve excluded future work.

Engineering blockers before freeze: package the actual P5-v4 image-to-fix path
behind the runtime adapter and prove numerical equivalence; implement the three
policy wrappers with shared code and a forecast-only difference; implement
Study 3 scenario/action dynamics and evaluator separation; prove selector
actions change subsequent rendered/sensor observations; add recursive leakage
tests; implement fixed successive halving, paired statistics, atomic resume and
lock verification; benchmark runtime/storage; and pass legacy plus complete
platform regression.

Current fairness issue: FIXED presently lacks a Study 3 development-selected
winner, and the integrated development runner synthesizes P5-shaped results
rather than invoking the packaged image front end. Therefore no Study 3 policy
comparison is currently eligible or fair. The design remedies both before
freeze. No held-out evaluation is authorized by this document.
