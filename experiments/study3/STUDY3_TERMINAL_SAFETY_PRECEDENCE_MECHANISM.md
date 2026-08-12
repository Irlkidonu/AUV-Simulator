# Study 3 terminal-safety precedence mechanism check

## Scope and predeclaration

This DEVELOPMENT change addresses only terminal-safety precedence. It does not
change capability thresholds, the existing 4.0 m2 critical-covariance bound,
mode selection, recovery thresholds, sensor models, scenarios, prior evidence,
or held-out data.

The complete-loss confirmation interval is **8 s**. It was selected before the
post-change replay from interface timing: it spans the complete 8 s acoustic
service-evidence lifetime and two ordinary 4 s optical/probing opportunities.
It was not fitted to the recorded outcome.

## Observable-only rule

Terminal safety takes precedence only while every condition below is true:

1. no current optical fix evidence;
2. no accepted acoustic fix or responding position-giving acoustic service;
3. neither DVL bottom track nor water track is live;
4. optical, acoustic and velocity usability posteriors are each below the
   existing configured viability boundary (0.35 in this session); and
5. onboard position-covariance trace is at or above the existing recovery
   planner critical bound (4.0 m2).

The condition must persist for 8 s. Any viable observation or posterior resets
confirmation immediately. Inertial propagation is deliberately not counted as
a viable aiding capability: it has no bounded horizontal reference.

No true error, true pose, scenario/family label, disturbance control, event
time, future event, or future environment state enters the rule.

## Mechanism tests

Focused tests demonstrate that:

- seven seconds of complete unsafe loss do not trigger and the eighth does;
- optical evidence resets confirmation;
- position-giving acoustic evidence resets confirmation;
- either DVL track resets confirmation;
- any navigation-capability posterior at its viability boundary resets it;
- uncertainty below the existing bound resets it, proving uncertainty alone is
  insufficient; and
- the end-to-end interactive compound-loss path reports the explicit reason
  `complete_navigation_loss_critical_uncertainty` and physically surfaces.

The focused interactive and mode-selection set passes 18/18 tests.

## Exact saved-recording replay

The original recording was not modified. Its embedded checksum
`19a76dc94aac2954d328bb0ce6030f51ab61c8776bc1071a1142026351b54464`
and regenerated base-environment digest
`4f56e9bc54bfc782be7a628d508e3aa4707e3a3f9065a8b52f856a1abc707065`
both pass.

The first DVL crashout begins at 59 s. The complete rule is not yet satisfied at
60 s because velocity posterior probability remains 0.379, above 0.35. At 61 s
the last navigation posterior falls below viability and covariance trace is
5.247 m2; confirmation starts. It reaches 8 s at 68 s and terminal surfacing is
commanded with the new explicit reason. GPS is reacquired at 73 s and the
existing protocol terminates the mission.

| Metric | Crashout-only replay | Safety-precedence replay |
|---|---:|---:|
| Terminal entry | 129 s | 68 s |
| GPS reacquisition | 138 s | 73 s |
| Mission duration | 138 s | 73 s |
| Horizontal RMSE | 2.919 m | 0.537 m |
| Peak horizontal error | 6.252 m | 2.176 m |
| Pre-GPS error | 6.252 m | 2.176 m |
| Post-GPS error | 1.388 m | 0.755 m |
| Unaided time | 114 s | 49 s |
| Longest aiding gap | 93 s | 28 s |
| Coverage | 0.15795 | 0.11483 |
| Safety violation | no | no |

The result is appropriately conservative: recovery controls scheduled manually
for 75 s are never reached because the vehicle cannot observe that future
recovery. The price of earlier safety is lower survey coverage and termination
of a mission that would later have regained DVL. This adverse tradeoff is kept
explicit; no broader campaign or threshold tuning was performed.
