# Study 3 DEVELOPMENT null findings: PREDICTIVE and ROBUST_FUSION

These are preserved as scientific results, not omissions. Both policies were
registered comparators in the V3 and V4 development campaigns and neither
produced a measurable effect. Both are excluded from the held-out comparison
for the reasons recorded here, and both must be reported in the manuscript.

## ROBUST_FUSION is not a distinct treatment

`ROBUST_FUSION` applies covariance weighting to the locked fixed configuration.
The locked baseline `fixed_155` already uses `fusion_mode: "weight"`, so the
wrapper changes nothing.

| Campaign | Pairs | ROBUST_FUSION identical to FIXED |
|---|---:|---|
| `mode_comparison_v3` (31,850,000) | 170 | **170/170**, byte-identical trace digest |
| `mode_comparison_v4` (31,870,000) | 170 | **170/170**, byte-identical trace digest |

Identity holds at the level of the causal trace, not merely the reported
outcomes. This is a construction result: the comparator collapsed into the
baseline it was meant to test against. It cannot explain any observed effect,
and it cannot be strengthened without changing `fixed_155`, which is frozen.

## PREDICTIVE produced no measurable benefit over REACTIVE

`PREDICTIVE` is identical to `REACTIVE` except that a frozen ten-second
observable trend forecast may trigger a reachable action before the current
capability boundary is crossed.

| Campaign | Pairs | Identical on all reported outcome metrics | Pre-emptive actions taken |
|---|---:|---|---:|
| `mode_comparison_v3` | 170 | **170/170** | **0** |
| `mode_comparison_v4` | 170 | **170/170** | **0** |

Metrics compared: completion, safety violation, overall RMSE, transition RMSE,
peak error, unaided time, longest aiding gap, coverage, optical fixes, acoustic
fixes and mission duration.

**The mechanism did not fire.** `preemptive_actions` is zero for every
PREDICTIVE run in both campaigns. The causal traces do differ from REACTIVE,
because the forecast is evaluated and recorded, but no forecast ever crossed
its confirmation requirement into a pre-loss action under these families.

This is therefore a **null result about reachability, not about prediction's
value**. The honest statement is that the registered predictive extension was
never exercised by the registered scenarios: the forecast confirmation
requirement and the family set did not co-occur. It is not evidence that
anticipatory action cannot help, and it must not be reported as such.

The V5 final validation used the three-policy set and did not include
PREDICTIVE, so V5 adds no further evidence either way.

## Consequence for the held-out design

Neither policy is carried into the held-out comparison. Running a comparator
that is provably identical to another (ROBUST_FUSION) or provably inert
(PREDICTIVE) would consume one third of a single-use held-out block to
reproduce a known development result.

Both findings stand as development evidence. Establishing whether prediction
can help requires a scenario set in which its confirmation requirement is
reachable, which is future work under a new protocol and a new root.
