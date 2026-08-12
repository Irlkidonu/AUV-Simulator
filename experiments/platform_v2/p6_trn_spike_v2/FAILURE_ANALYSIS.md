# P6-v2 terminal result

Date: 2026-08-10  
Status: **FAIL — one permitted execution consumed; no rerun and no P6-v3 in
the current submission cycle**

The immutable result is `result.json`, SHA-256
`6cbeef64dec35cbbd0ad3063b7ae1549823cd1209e12b7e4aeb51e959fc56b64`.

## What the redesign fixed

- Repeated terrain: zero false convergences in 200 trials; all 200 rejected.
- Flat terrain: zero accepted fixes in 100 trials.
- Truncated searches: zero accepted fixes in 100 trials.
- Informative terrain: zero false convergences in every scoring stratum.
- Reference informative condition: 99% fix rate, 0.0275 m median error and
  0.0576 m 95th-percentile error.
- Runtime and covariance positive-definiteness passed.

The P6-v1 failure mechanism is therefore addressed: equally refined basins,
boundary completeness and profile consistency prevent confident periodic
aliases.

## Failed gates

1. Near-repetitive availability was 14%, below the predeclared 50% minimum.
   Thirty-five trials were rejected as ambiguous, 49 as profile-inconsistent
   and two at the search boundary. The safety mechanism does not merely reject
   exact aliases; it rejects too many maps with a weak but real uniqueness cue.
2. Pooled informative nominal-95% coverage was 501/507 = 98.82%. Its exact
   two-sided 95% interval was `[0.9744, 0.9956]`, which excludes 0.95. The
   calibration multiplier remained exactly 1.0, so the over-conservatism comes
   from the added window/sample covariance and sandwich terms rather than the
   calibration scale.

## Scientific interpretation

P6-v2 changes the failure mode from unsafe confident aliasing to safe but
excessive rejection and conservative confidence. That is meaningful progress,
but it does not satisfy the feasibility contract and is not integrated as a
navigation-aiding source. P6-v1 and P6-v2 remain adverse evidence. No threshold,
protocol, source or result is changed after execution.

Unrelated platform-v2 work may continue under its own gates. Further TRN
redesign requires a future identifier and new development root outside the
current submission cycle.
