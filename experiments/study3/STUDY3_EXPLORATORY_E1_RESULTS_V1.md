# Study 3 — post-freeze exploratory Part E1 results (V1)

Classification: **POST-FREEZE EXPLORATORY.**
Not held-out. Not confirmatory. Date: 2026-08-11.

**No acceptance threshold is applied and no verdict is assigned.** These numbers
cannot revise the held-out result or `STUDY3_FREEZE_DECISION_V1.md`. They are
reported regardless of direction.

## Execution

180 executions at exploratory root 33,000,000: 60 generated environment
realizations (seeds 33,001,000–33,001,059), each shared identically by FIXED,
DEPLOYMENT_FIXED and REACTIVE. Configuration
`moderate_severe_variable_multimodal`, unmodified. Horizon 180 s, dt 2 s, image
period 4 s, `redesign_version` 3 — inherited from V5 Part B.

The executed `plan_digest`
`f4d0f8323e35b8e02d079778af47bde56a5423e349fadceba7ee81ae083b94be`
is identical to the digest committed before execution in `8ced9356`.
`result_digest` `5016882007736f75bd44d0e92974d2bba654003e8d8f67ac35b7be6b43bc37d0`.
All 180 packet checksums verify. Zero drift across the 43 allowlisted frozen
files, checked before and after execution. Root 32,000,000 was not accessed.

`STUDY3_SEED_REGISTRY.json` was not modified: it is hash-pinned in the freeze
manifest, so the exploratory band is registered in
`STUDY3_EXPLORATORY_E1_DESIGN_V1.json` instead.

## Outcome means, 60 paired environments

| Metric | FIXED | DEPLOYMENT_FIXED | REACTIVE |
|---|---|---|---|
| completed | 0.2833 | 0.3000 | 0.3000 |
| safety_violation | 0.1833 | 0.2667 | 0.1333 |
| overall_rmse_m | 2.3117 | 2.4340 | **1.6896** |
| rmse_transition_m | 2.4049 | 2.5364 | **1.7607** |
| peak_error_m | 5.2993 | 5.8895 | **4.3698** |
| unaided_time_s | 74.5333 | 78.4000 | 69.9333 |
| longest_unaided_gap_s | 46.6667 | 48.5000 | **35.7667** |
| survey_coverage_fraction | 0.8386 | 0.8386 | 0.7923 |
| optical_fixes | 9.4333 | 8.9833 | 9.7667 |
| acoustic_fixes | 6.0167 | 5.7000 | 5.6500 |
| physical_interventions | 0.7167 | 0.7000 | 7.0167 |
| mode_switches | 0.0000 | 0.0000 | 9.3333 |

**These are not held-out conditions.** On the held-out scripted families every
policy completed every mission with zero safety violations. Here completion is
below one third for all three policies and safety violations occur in 13–27% of
episodes. The generated environments are far harsher than the scripted families,
and absolute levels are not comparable between the two blocks.

## Paired contrasts, 95% paired bootstrap over 60 environments

Negative favours the first-named policy on error metrics.

### REACTIVE − FIXED (universal `fixed_155`)

| Metric | Mean | 95% CI | w/t/l |
|---|---|---|---|
| overall_rmse_m | −0.6221 | [−1.3746, −0.0216] | 32/0/28 |
| rmse_transition_m | −0.6441 | [−1.4231, −0.0053] | 32/0/28 |
| peak_error_m | −0.9294 | [−2.3282, +0.2284] | 33/0/27 |
| longest_unaided_gap_s | −10.9000 | [−20.5008, −1.8667] | 28/13/19 |
| unaided_time_s | −4.6000 | [−13.3333, +3.9000] | 28/9/23 |
| completed | +0.0167 | [−0.0833, +0.1167] | 4/51/5 |
| safety_violation | −0.0500 | [−0.1500, +0.0667] | 7/49/4 |
| survey_coverage_fraction | −0.0463 | [−0.0937, −0.0023] | 24/29/7 |
| physical_interventions | +6.3000 | [+5.5000, +7.1167] | 0/0/60 |
| mode_switches | +9.3333 | [+8.3333, +10.3837] | 0/0/60 |

### REACTIVE − DEPLOYMENT_FIXED (deployment-informed)

| Metric | Mean | 95% CI | w/t/l |
|---|---|---|---|
| overall_rmse_m | −0.7445 | [−1.3851, −0.1723] | 38/0/22 |
| rmse_transition_m | −0.7757 | [−1.4421, −0.1755] | 38/0/22 |
| peak_error_m | −1.5197 | [−2.8301, −0.3680] | 37/0/23 |
| longest_unaided_gap_s | −12.7333 | [−20.7000, −5.0992] | 35/8/17 |
| unaided_time_s | −8.4667 | [−17.6333, +0.7000] | 36/5/19 |
| completed | +0.0000 | [−0.1000, +0.1000] | 4/52/4 |
| safety_violation | −0.1333 | [−0.2667, +0.0000] | 12/44/4 |
| survey_coverage_fraction | −0.0463 | [−0.1046, +0.0117] | 25/29/6 |

### DEPLOYMENT_FIXED − FIXED

| Metric | Mean | 95% CI | w/t/l |
|---|---|---|---|
| overall_rmse_m | +0.1224 | [−0.6818, +0.8461] | 28/0/32 |
| rmse_transition_m | +0.1316 | [−0.6685, +0.8869] | 28/0/32 |
| longest_unaided_gap_s | +1.8333 | [−8.2000, +11.1333] | 24/8/28 |
| safety_violation | +0.0833 | [−0.0167, +0.1833] | 3/49/8 |

## Adaptation

### V5 C7/C8 definition — the number comparable to 59.8%

Transcribed unchanged from `analyse_final_validation_v5.py::adaptation`.

| Policy | Episodes | Matched | Coverage | Median latency |
|---|---|---|---|---|
| FIXED | 287 | 0 | 0.0000 | — |
| DEPLOYMENT_FIXED | 277 | 0 | 0.0000 | — |
| **REACTIVE** | **284** | **181** | **0.6373** | **2.0 s** |

The two fixed policies never change mode (`mode_switches` = 0 for all 120 runs),
so zero coverage across roughly 280 episodes each is the expected reading and
serves as a check that the metric measures what it claims to.

For REACTIVE the development V5 figure was **59.8% (52/87)** on 12 seeds at root
31,900,000. Here it is **63.73% (181/284)** on 60 fresh seeds — 3.3× the episode
count, same definition, same environment configuration, different seed band.

### Corrected pilot adequate/exact definition — REACTIVE

Obtained by deterministic replay: all 60 REACTIVE runs were re-executed
in-process with a recording policy and every one reproduced its packet's
`trace_digest`, so the analysis observes the executed runs and no new evidence
was generated.

| Quantity | Value |
|---|---|
| Episodes | 343 (141 with an ambiguous acceptable set) |
| Adequate and observably supported | 262 / 343 = **0.7638**, median delay 0.0 s |
| Exact preferred, where evaluable | 129 / 202 = **0.6386**, median delay 4.0 s |

## Observations, stated without a verdict

1. **REACTIVE separates from both comparators here, including the
   deployment-informed one.** On the held-out scripted block the REACTIVE-minus-
   deployment-informed interval spanned zero (−0.0070 m, [−0.0197, +0.0061]). In
   these stochastic environments it does not (−0.7445 m, [−1.3851, −0.1723]).
   These are different environment classes and different seed bands; this is
   exploratory evidence about generated environments and is **not** a
   re-litigation of the held-out result, which stands as executed.
2. **The deployment-informed advantage disappears.** DEPLOYMENT_FIXED is not
   distinguishable from universal FIXED here on any error metric, and its mean
   safety violation rate is the highest of the three. A single technique chosen
   from the initial deployment is a weaker idea when infrastructure availability
   itself varies during the mission.
3. **Adaptation is not free.** REACTIVE costs +6.3 physical interventions and
   +9.3 mode switches per mission, and loses 0.046 of survey coverage against
   FIXED with an interval excluding zero.
4. **Adaptation coverage is in the same region as development**, 63.73% against
   59.8%, on five times the seeds. Roughly a third of post-launch changes of the
   truth-side best viable mode are still never matched.
5. **Absolute mission outcomes are poor for every policy** in this environment
   class — under a third of missions complete. Nothing here should be read as
   the system performing well in absolute terms; the contrasts are between
   policies under identical realizations.

## Scope

Part E2, the scripted-transition block, was not run. The families excluded by the
freeze (`E6` intermittent acoustic, `E12` rotating current, `E16`/`E17` terrain)
remain unreachable and are unaffected by this block.
