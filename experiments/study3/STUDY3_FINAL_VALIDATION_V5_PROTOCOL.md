# Study 3 final DEVELOPMENT validation V5

Status: **AUTHORIZED; PRE-EXECUTION**
Date: 2026-08-11
Implementation commit: `9244a90d` (plus test-harness fix `b0af2d5b` and
evidence preservation `f9326267`, neither of which changes behaviour)
Development roots: smoke `31,899,000`; validation `31,900,000`
Held-out root `32,000,000`: **forbidden; not accessed by this protocol**

This protocol is written and committed **before execution**. Every threshold
below is derived from interface timing, an existing declared bound, or a
previously registered margin. **No threshold is taken from V3, V4 or
fairness-v1 effect sizes.** V3 (`31,850,000`), V4 (`31,870,000`) and
fairness-v1 (`31,880,000`) remain immutable historical DEVELOPMENT evidence and
are not reinterpreted.

## What is being validated

The candidate final system at `9244a90d`, comprising the six-mode observable
selector, the enforced interactive DVL crashout, terminal-safety precedence
after confirmed complete loss, the generated variable-environment framework,
the deployment-informed FIXED comparator and the corrected adaptation metrics.

## Approved claim framing

1. **Superiority** over the universal locked baseline `fixed_155`.
2. **Non-inferiority** to deployment-informed FIXED overall.
3. **Measurable online-adaptation benefit** specifically when the best viable
   mode changes after launch.

Deployment-informed FIXED is the fair comparator for (2): it selects one
technique from infrastructure declared deployed at launch and never switches.
Superiority over it is **not** claimed, because it presumes deployment
knowledge the vehicle does not have. Non-inferiority is the honest bar.

## Construction

**Part A — scripted transition families.**
Ten unchanged Study 3 families x seventeen fresh paired indices x three
policies (`fixed`, `deployment_fixed`, `reactive`) = **510 executions**, paired
by root, family, index and subsystem stream name. Redesign version 3.
Analysis scope is the seven primary families (119 pairs); the three controls
are reported separately and are not part of the gating aggregate.

**Part B — generated changing environments.**
`moderate_severe_variable_multimodal` x twelve environment seeds x the same
three policies = **36 executions**, traces retained. The environment
realization is generated once per seed and shared identically by all three
policies. Truth-side viability is computed by the evaluator only.

Total: **546 executions**, one invocation, no interim analysis.

## Predeclared gating criteria

Aggregates are paired means over the seven primary families for Part A, and
over all generated seeds for Part B. Intervals are 95% family-stratified
paired bootstrap. All eight must pass.

| # | Criterion | Threshold | Derivation |
|---|---|---|---|
| **C1** | Safety | REACTIVE uncontrolled safety-violation rate ≤ each FIXED comparator + **0.02** | Small absolute margin; safety is non-negotiable |
| **C2** | Completion | REACTIVE completion ≥ deployment-informed FIXED − **0.05** | The margin registered in V3 |
| **C3** | Overall RMSE vs universal `fixed_155` | paired mean **< 0**, 95% CI excludes 0 | Superiority claim (1) |
| **C4** | Transition RMSE vs universal `fixed_155` | paired mean **< 0**, 95% CI excludes 0 | Superiority during capability change |
| **C5** | Longest aiding gap vs universal `fixed_155` | paired mean **< 0**, 95% CI excludes 0 | Aiding-gap mechanism |
| **C6** | Non-inferiority to deployment-informed FIXED | upper 95% bound on RMSE difference ≤ **+0.10 m**, and completion lower bound ≥ **−0.05** | 0.10 m is the P5 spike's declared clear-water localisation accuracy unit; within one such unit is operationally equivalent |
| **C7** | Online adaptation | in Part B, among episodes where the truth-side best viable mode **changes after launch**, REACTIVE achieves an adequate contemporaneously-supported match in ≥ **70%** | Serialized probing gives one probe per 4 s with an 8 s evidence lifetime, so episodes shorter than a probe interval are unobservable in principle; 70% demands a clear majority while allowing that structural limit |
| **C8** | Adaptation latency | median adequate-match latency ≤ **12 s** | Three ordinary 4 s optical/probe opportunities |

**Adequate match** uses the corrected pilot definition: the selected mode is in
the acceptable set **and** has contemporaneous observation-side support. A
stale label is not a match. Simultaneously viable modes are an unordered
acceptable set and are not given a post-hoc preference. The terminal-commitment
sample is retained; later samples are excluded.

## Secondary, reported but non-gating

PREDICTIVE − REACTIVE; peak error; survey coverage; physical and unnecessary
interventions; logical mode transitions; per-family breakdown; control-family
behaviour; deliberate GPS safety aborts distinguished from uncontrolled safety
violations.

Prediction is not required to beat reaction.

## Decision rule

Exactly one outcome is assigned by the single analysis:

- **RECOMMEND FREEZE** — all eight gating criteria pass.
- **NO-FREEZE** — any gating criterion fails.

A partial or heterogeneous result is a NO-FREEZE. Freeze recommends only the
single held-out evaluation at `32,000,000`; **this protocol does not authorise
held-out execution.**

## Binding conditions

No threshold, scenario, policy, sensing model, comparator or recovery behaviour
may be changed after execution begins. The run completes irrespective of
partial results. There is **one** analysis; it is not repeated with adjusted
definitions. If the outcome is NO-FREEZE, no correction cycle is authorised by
this protocol.
