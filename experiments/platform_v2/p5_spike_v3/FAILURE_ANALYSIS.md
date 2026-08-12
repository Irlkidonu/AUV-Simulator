# P5-v3 immutable outcome and failure analysis

Date: 2026-08-10  
Outcome: **FAIL**  
Executions used: **1 of 1**

The corrected development attempt completed all 600 declared pairs without the
P5-v2 division-by-zero failure. Therefore, P5-v2 remains an execution failure,
whereas P5-v3 is the first interpretable scientific result for this detector and
gate configuration.

No false fix was accepted among 200 negative pairs, both ambiguity-rejection
criteria passed, runtime passed, and every computed covariance was positive.
However, localization availability was zero in every positive stratum. In the
clear T and Y strata, all 50/50 pairs passed detection, matching, the 20-inlier
minimum, reprojection, both hull-coverage tests, both quadrant tests, scale,
positive covariance, and the covariance-axis limit. All 50/50 failed both
predeclared scatter-eigenvalue requirements: the required value was
`0.04 * 192^2 = 1474.56 px^2`, while the observed maxima were `1306.30 px^2`
for T and `1254.29 px^2` for Y. Thus the strict conjunction rejected otherwise
accurate transforms. The corresponding ungated transform estimates had median
translation errors of 0.00154 m (T) and 0.00103 m (Y), but those values are
diagnostic only and are not localization successes under the frozen protocol.

The result cannot be made a pass by relabelling the execution or relaxing the
spatial-support threshold after inspection. Under the protocol, optical-localizer
development stops for this submission. The defensible conclusion is narrower:
the repaired front end detects and matches the development imagery and avoids
confident false fixes, but the frozen compound geometric gate does not establish
usable localization availability. P5-v3 is feasibility **FAIL**, not validation.
