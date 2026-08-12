# Study 3 freeze decision V1 — researcher decision

Status: **FREEZE APPROVED BY RESEARCHER**
Date: 2026-08-11
Decision authority: C. Alexandris (researcher)
Prepared by: agent, on instruction

This is a **researcher scientific-acceptance decision**. It is not a
reinterpretation of the V5 protocol, not a change to any V5 criterion, and not
a correction of the V5 result. V5 stands exactly as executed and recorded.

## What V5 returned, unchanged

`STUDY3_FINAL_VALIDATION_V5_PROTOCOL.md` predeclared eight gating criteria and
the rule that any failure yields NO-FREEZE. The single analysis returned:

> **Verdict: NO-FREEZE. Failed criteria: C7_adaptation_rate.**

Seven of eight criteria passed. **C7 adaptation coverage failed at 59.8%**
(52 of 87 episodes) against its threshold of 70%.

| # | Criterion | Threshold | Observed | Outcome |
|---|---|---|---|---|
| C1 | Safety | ≤ comparator + 0.02 | 0.000 vs 0.000 / 0.000 | pass |
| C2 | Completion | ≥ deployment − 0.05 | 1.000 vs 1.000 | pass |
| C3 | Overall RMSE vs universal `fixed_155` | mean < 0, CI excludes 0 | −0.441 m [−0.471, −0.414] | pass |
| C4 | Transition RMSE vs universal | mean < 0, CI excludes 0 | −0.517 m [−0.552, −0.484] | pass |
| C5 | Longest aiding gap vs universal | mean < 0, CI excludes 0 | −35.87 s [−36.57, −35.13] | pass |
| C6 | Non-inferiority to deployment-informed FIXED | CI high ≤ +0.10 m | −0.020 m [−0.036, −0.004] | pass |
| **C7** | **Adaptation coverage** | **≥ 70%** | **59.8% (52/87)** | **FAIL** |
| C8 | Adaptation latency | median ≤ 12 s | 0.0 s (mean 2.65 s) | pass |

**These figures are not revised by this decision.** Any report of V5 states the
NO-FREEZE verdict and the 59.8% against 70%.

## Provenance of the 70% threshold

The 70% value in C7 was **selected by the agent** when drafting the V5 protocol,
derived from serialized probe timing (one probe per 4 s, 8 s evidence lifetime).
It was committed in the protocol before execution, which makes it binding on the
V5 analysis, and it was correctly applied.

It was **never submitted to the researcher for approval and was never adopted as
a scientific freeze requirement.** It is an agent-selected analysis threshold,
not a researcher-endorsed acceptance criterion. Recording this is the reason
this document exists: the distinction is between a protocol that was honoured
and an acceptance level that was never agreed.

## Researcher acceptance level for adaptation coverage

The researcher's scientific judgment, recorded here and applying from this point
forward:

> **For this work, adaptation coverage above 50% is sufficient. The observed
> 59.8% exceeds that level.**

This is stated as a scientific acceptance decision about what the evidence
supports. It is **not** applied retroactively to V5's pass/fail classification,
which remains 7/8 with C7 failed.

## Decision

On the overall V5 evidence, the researcher judges the system ready to freeze:

- superiority over the universal locked `fixed_155` is large and consistent
  across RMSE, transition RMSE and aiding gaps, with intervals excluding zero;
- non-inferiority to deployment-informed FIXED is not merely met but exceeded,
  which reverses the fairness-v1 equivalence finding;
- completion, safety and coverage are identical across all three policies;
- adaptation, when it occurs, is essentially immediate at a 0.0 s median;
- adaptation coverage of 59.8% is above the researcher's stated sufficiency
  level and is carried forward as a reported limitation.

**The system is frozen for one held-out evaluation.**

## Standing limitation to be reported

The manuscript and any report of this work must state that development
adaptation coverage was **59.8%**: roughly two in five post-launch changes of
the best viable mode were never matched. This is a limitation of the frozen
system, not a resolved question, and it is not softened by the researcher's
sufficiency level.

The PREDICTIVE and ROBUST_FUSION development null findings
(`STUDY3_DEVELOPMENT_NULL_FINDINGS.md`) are likewise reported, not omitted.

## What this decision does not do

It does not authorise held-out execution. Authorisation is a separate record,
`STUDY3_HELDOUT_AUTHORIZATION_V1.json`, which cites this decision so that the
failed criterion cannot be read past.

It does not alter V5, its protocol, its packets, its analysis or its verdict.

## Process note

The researcher has directed that subjective scientific acceptance thresholds be
proposed for approval rather than selected unilaterally. The C7 threshold was
selected unilaterally and this record exists in part because of that. Future
protocols must present acceptance levels for approval before commitment.
