# P5-v4 optical-localizer development protocol

Status: development engineering; not held-out validation  
Protected evidence: P5-v1/v2/v3 remain immutable  
Held-out use: prohibited

## Motivation and acceptance design

P5-v3 showed that `0.04*width^2` was not a calibrated degeneracy boundary. It
requires a 38.4 px minor-axis standard deviation. AKAZE's observed usable span
is approximately 134 px; even points uniformly filling that detector support
have variance only `134^2/12 = 1496.3 px^2`, leaving almost no allowance for
nonuniform natural features. The threshold therefore tests near-uniform filling
of the detector's available field rather than merely rejecting collinearity.

P5-v4 replaces that proxy with direct, interpretable requirements:

- at least 12 RANSAC inliers and at least 50% inlier fraction;
- median reprojection error below 2 px;
- no second disjoint similarity hypothesis with at least half the support of
  the primary hypothesis;
- similarity scale in `[0.60,1.67]`;
- finite positive propagated covariance with maximum horizontal one-sigma axis
  below 0.10 m.

Hull coverage, horizontal/vertical span, 4x4-grid occupancy, point condition
number and the old scatter eigenvalue are all reported. They are not separate
hard gates. The centred similarity Jacobian already contains feature leverage;
with a 0.5 px residual-variance floor, weak or collinear geometry produces a
singular or large propagated covariance and fails the direct 0.10 m uncertainty
limit. Calibration showed that adding fixed hull/span/grid/condition cutoffs
rejected six correct partial-overlap fixes without rejecting an additional
negative candidate. The alternative-hypothesis test targets repeated texture
directly.

Twelve inliers give six times the two-correspondence algebraic minimum and 20
residual degrees of freedom for the four-parameter similarity. At the 2 px
ceiling, the translation standard-error bound is about 0.58 px; combined with
the 32 px two-axis span and explicit covariance gate, this is adequate for the
declared error domain. Calibration showed that going below 10 begins admitting
repeated-texture hypotheses, so 12 is retained as the safety boundary rather
than selected from availability.

The AKAZE threshold is `5e-5`, reducing the P5-v2 value by one octave in
response to a measured partial-overlap density limitation: at `1e-4`, only
19/50 partial-overlap calibration pairs supplied 20 inliers and the median was
13. The descriptor ratio is 0.80 in v4. The former 0.75 value discarded most
partial-overlap candidates before geometric verification. The modestly broader
candidate set is not trusted directly: it must still pass RANSAC, a 50% inlier
fraction, direct support, ambiguity, covariance and false-fix checks.

## Development sequence

1. Quantitative diagnosis replays only P5-v3 development imagery and writes
   separate diagnostic evidence.
2. Engineering/calibration uses root `22,130,000`. Detector parameters remain
   lower-threshold AKAZE from P5-v2/v3. Gate implementation may be corrected and
   uncertainty inflation estimated here. The fixed confirmation inflation is
   `2.30`, rounded conservatively above the calibration value `2.268` obtained
   as the 95th horizontal-NEES quantile divided by the chi-square(2) 95%
   boundary. Intermediate development outputs are
   retained rather than overwritten.
3. After the rule and covariance inflation are fixed, confirmation uses fresh
   development root `22,140,000`. Confirmation is not final held-out evidence.

Both roots contain 50 examples of translation, yaw, scale/altitude, partial
overlap, two turbidity levels, repeated texture and feature-poor texture, plus
100 non-overlap, 50 repeated-texture-negative and 50 independent-feature-poor
negative pairs. Results report stage success, errors, false fixes, geometry,
ambiguity, covariance coverage and runtime separately.

Useful availability means at least 90% translation, 80% combined yaw/scale and
60% partial-overlap success. Dangerous false-fix acceptance must be zero among
the 200 declared negative pairs and below 1% in every positive stratum. Median
and 95th-percentile clear translation errors must be below 0.10/0.25 m; yaw
below 1/3 degrees; scale below 2/5%. Horizontal 95% ellipse coverage must be
85--99%; median/p95 runtime must remain below 50/100 ms. Turbidity availability
must be non-increasing. Repeated and feature-poor conditions are reported rather
than definitionally required to reject: a correctly constrained fix is useful;
an ambiguous or inaccurate fix is not.

Any development pass establishes feasibility only. It does not validate the
subsystem and does not authorize a final held-out run.
