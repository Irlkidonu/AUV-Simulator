# P5-v4 diagnosis, engineering decision and development outcome

Date: 2026-08-11  
Data class: development only  
Final held-out evaluation: not run

## Why the P5-v3 scatter gate failed

The `0.04*min(width,height)^2` threshold first appears in the P5-v2 protocol.
Its recorded purpose was to reject nearly collinear support, but no camera,
Jacobian, uncertainty or false-fix derivation was recorded. At 192x192 it equals
1474.56 px2, or a required minor-axis standard deviation of 38.4 px.

Diagnostic replay reproduced all 600 P5-v3 pairs. The largest observed inlier
span was about 134 px. Uniform points over a 134 px interval have variance only
1496.3 px2. Thus the gate required almost perfectly uniform filling of the
entire AKAZE-usable span on both axes and both images. It was not merely a
collinearity test. This explains why every T/Y fix failed it despite all 50/50
passing inlier count, reprojection, hull, quadrant, scale, covariance positivity
and uncertainty-axis checks. Their median ungated translation errors were
1.54 mm and 1.03 mm.

Across the replay's candidate transforms, safe fixes had median 61 inliers,
29.4% hull coverage, 112.5/121.6 px x/y span, 10 occupied 4x4 cells and point
condition 1.24. Dangerous/declared-adversarial candidates had medians of four
inliers, 0.75% hull, 60.7/65.4 px span, two cells and point condition 2.81.
Reprojection alone was not discriminative: its dangerous median was 0.42 px,
because tiny false hypotheses can fit exactly. Candidate horizontal NEES was
much more informative (safe median 2.45 versus dangerous median 87.4), although
uncalibrated covariance still under-covered its upper tail.

## Acceptance decision

P5-v4 reports scatter, hull, span, grid occupancy and point condition but does
not threshold each proxy independently. The accepted combination is:

- 12 or more inliers (six times the similarity model's algebraic minimum);
- at least 50% consensus and median reprojection below 2 px;
- no disjoint alternative hypothesis with at least half the primary support;
- physical scale in `[0.60,1.67]`;
- finite positive covariance, formed with a 0.5 px residual floor, with maximum
  horizontal sigma below 0.10 m;
- covariance inflation 2.30, fixed from the calibration root's horizontal-NEES
  95th percentile.

This is more direct than the former compound spatial proxy. On calibration
data, adding hard hull/span/grid/condition cutoffs rejected six correct
partial-overlap fixes and no additional negative. The retained rule rejected
all 200 negatives.

## Fresh development confirmation

The frozen root 22,140,000 completed once and returned **DEVELOPMENT PASS**:

- translation 100%, yaw 100%, scale 96%, partial overlap 74%;
- feature-poor same-map localization 100% with 2.67 mm median error;
- repeated-texture same-map localization 12%, all correct within the declared
  false-fix limits;
- 0/200 false fixes across non-overlap, repeated-displacement and independent
  feature-poor negatives;
- clear-condition horizontal 95% ellipse coverage 95.95%;
- partial-overlap median/p95 translation error 3.41/9.94 mm;
- median runtimes 11.0--16.8 ms by stratum.

At attenuation 0.6 and 1.2 the localizer accepted no fixes. This is safe loss of
availability, not demonstrated turbid-water localization. A separately
versioned mutual-matching experiment sharply reduced negative match survival
but did not recover turbid consensus and slightly worsened nominal errors; it
was therefore not selected.

P5-v4 establishes development feasibility of a conservative optical front end.
It is not a validated subsystem and supplies no final held-out claim.
