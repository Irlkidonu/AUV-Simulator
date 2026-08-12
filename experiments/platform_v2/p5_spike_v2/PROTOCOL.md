# P5 optical-localisation feasibility spike v2 — frozen proposal

Status: **PROTOCOL ONLY — DO NOT RUN YET**

Identifier: `p2v2_p5_spike_v2`  
Fresh development seed root: `22,110,000`  
Maximum executions: **one**

Spike v1 remains an immutable FAIL. Spike v2 is justified only by the recorded
failure-stage diagnosis: pose overlap was adequate, valid spatial
correspondences existed, and default AKAZE usually generated fewer keypoints
than the geometric gate could possibly accept.

## 1. Frozen front end

The only detector/descriptor used in the spike is AKAZE:

| Parameter | Frozen value |
|---|---:|
| descriptor | `DESCRIPTOR_MLDB` |
| descriptor size | 0 (full descriptor) |
| descriptor channels | 3 |
| detector threshold | `1.0e-4` |
| octaves | 4 |
| octave layers | 4 |
| diffusivity | `DIFF_PM_G2` |
| matcher | brute-force Hamming, no cross-check |
| nearest-neighbour ratio | 0.75 |
| transform | 2-D similarity: translation, yaw and uniform scale |
| robust estimator | RANSAC |
| RANSAC reprojection threshold | 2.0 px |
| RANSAC confidence | 0.995 |
| maximum RANSAC iterations | 2000 |

No detector, threshold, ratio or RANSAC sweep occurs during the spike. ORB is
not run as a comparator. The `1e-4` threshold is fixed because v1 produced a
median of only 16–17 default-AKAZE keypoints, while the unchanged geometric gate
required 20 inliers. A diagnostic `1e-4` threshold produced a median of roughly
145 keypoints on the same images. The threshold therefore repairs a measured
sampling deficiency at the detector stage; it is not selected from spike-v2
outcomes.

## 2. Geometric-support gate

A similarity transform has four degrees of freedom and a two-point algebraic
minimum. That minimum is unsafe: two nearby correspondences give essentially no
rotation/scale leverage, and a cluster in one image corner can extrapolate a
precise-looking but wrong camera-centre displacement.

The v1 count threshold is therefore **not lowered**. A candidate localization is
geometrically verified only when all conditions hold:

1. at least 20 RANSAC inliers;
2. median inlier reprojection error below 2.0 px;
3. inlier convex-hull area at least 15% of image area in both images;
4. inliers occupy at least three of four image quadrants in both images, with
   at least three inliers in every occupied quadrant;
5. the smaller eigenvalue of the centred 2-D inlier-coordinate scatter matrix
   is at least `0.04 * min(width,height)^2`; this rejects nearly collinear
   support;
6. estimated similarity scale lies in `[0.60, 1.67]`, the range implied by the
   declared altitude ratios plus margin;
7. the propagated horizontal position covariance is positive definite and its
   largest one-sigma axis is below 0.10 m.

With a 192 px image and approximately 0.018 m/px at 3 m altitude, 20 spatially
distributed inliers at the 2 px residual ceiling bound the translation standard
error to roughly `2/sqrt(20) = 0.45 px`, or 8 mm before model and scale
uncertainty. The 0.10 m covariance ceiling leaves more than an order-of-
magnitude margin while rejecting geometrically weak transforms. Covariance is
computed from the similarity-transform Jacobian and the robust inlier residual
variance; it is not a fixed declared covariance.

## 3. Stage definitions

Every pair receives one outcome at each stage:

- **detection success:** at least 20 keypoints in each image;
- **match success:** at least 20 ratio-test correspondences;
- **geometric-verification success:** all count, reprojection, spatial coverage,
  conditioning and scale requirements above pass;
- **localization success:** verified transform with finite covariance and errors
  inside the declared validity domain;
- **false fix:** localization success when horizontal error exceeds 0.50 m,
  yaw error exceeds 5 degrees, scale error exceeds 10%, or the two images are a
  declared non-corresponding negative pair.

A missed fix and a false fix are always reported separately. A rejected
ambiguous transform is a missed fix, not a false fix.

## 4. Fresh deterministic dataset

V1's 50 pose pairs are diagnostic-only and are not members of the v2 scoring
set. V2 uses the fresh root `22,110,000`, named RNG substreams for world texture,
poses and sensor noise, and 50 pairs per positive stratum:

| Stratum | Conditions |
|---|---|
| T — translation | altitude 3 m, yaw fixed, translation 0.05–0.50 m, `c=0.2 m^-1` |
| Y — yaw | translation 0.05–0.30 m, yaw change uniformly 2–20 degrees, altitude 3 m, `c=0.2` |
| S — scale | altitude pairs spanning ratios 0.75–1.35 within 2.0–4.0 m, translation 0.05–0.30 m, yaw below 5 degrees, `c=0.2` |
| P — partial overlap | translation chosen for 35–70% common footprint area, altitude 3 m, yaw below 10 degrees, `c=0.2` |
| W1 — moderate turbidity | mixed T/Y/S geometry, `c=0.6 m^-1` |
| W2 — severe turbidity | mixed T/Y/S geometry, `c=1.2 m^-1` |
| R — repeated texture | sinusoidal/ridge texture with repeated spatial aliases, mixed T/Y geometry, `c=0.2` |
| F — feature-poor | low-relief, low-modulation texture, mixed T/Y geometry, `c=0.2` |

The same world extent and camera model are used for all policies within a
stratum. Pose sampling rejects footprints leaving the map. Ground-truth
transform is calculated from the generated camera poses, never estimated from
pixels.

Negative controls use 200 additional pairs:

- 100 non-overlapping patches separated by more than one full footprint;
- 50 repeated-texture alias pairs with no true overlap;
- 50 independently seeded feature-poor/noise-only pairs.

Positive and negative pair lists, seeds and stratum labels are generated and
checksummed before feature extraction begins.

## 5. Required reporting

Report per stratum and overall:

- keypoint count and detection-success rate;
- ratio-match count and match-success rate;
- inlier count, hull coverage, scatter eigenvalues and geometric-verification
  success rate;
- localization-success and missed-fix rates;
- false-fix count and rate with exact binomial confidence interval;
- horizontal translation error: median, 95th percentile and maximum;
- yaw error: median, 95th percentile and maximum;
- scale error: median, 95th percentile and maximum;
- covariance eigenvalues, 95% ellipse coverage and NEES;
- median and 95th-percentile single-core runtime.

The attenuation strata additionally report whether detection, verification and
localization success are monotonically non-increasing from `c=0.2` to 0.6 to
1.2. Repeated and feature-poor strata report rejection/ambiguity separately
from ordinary missed fixes.

## 6. Predeclared pass/fail criteria

Spike v2 passes only if every primary gate passes:

1. T translation localization success at least 90%;
2. combined Y and S localization success at least 80%;
3. P partial-overlap localization success at least 70%;
4. across clear-water T/Y/S successes, median horizontal error below 0.10 m and
   95th percentile below 0.25 m;
5. Y median yaw error below 1 degree and 95th percentile below 3 degrees;
6. S median relative-scale error below 2% and 95th percentile below 5%;
7. **zero false fixes among all 200 negative controls** and below 1% false-fix
   rate in every positive stratum; the zero-count requirement is stricter than
   the rate threshold and reflects the safety asymmetry;
8. repeated-texture ambiguity rejection at least 95%;
9. feature-poor rejection at least 95% when the geometric-support gate is not
   met; no credit is given for fabricating a localization;
10. localization-success rates are monotonically non-increasing over attenuation;
11. every reported covariance is positive definite, and clear-water T/Y/S 95%
    ellipse coverage lies between 85% and 99%;
12. median runtime below 50 ms and 95th percentile below 100 ms per pair on one
    CPU process;
13. the `study2_legacy` critical hashes, 27-run golden regression and complete
    legacy test suite remain green.

W1 and W2 have no minimum success-rate gate: attenuation is expected to remove
features, and honest rejection is preferable to a confident false fix. Their
false-fix, monotonicity and covariance gates still apply.

## 7. Execution and terminal rule

The protocol, implementation digest, pair manifest and output schema are frozen
before execution. The spike runs once. An interruption may resume only from
verified, already generated pair packets; parameters and pair identities do not
change.

If any primary gate fails, record **FAIL**, stop optical-localizer development
for this submission and proceed with the separately specified TRN track. There
is no P5 spike v3. If all gates pass, record **FEASIBILITY PASS** only. A pass
does not establish a validated optical-localization subsystem and does not
authorize building the full localizer until the result and covariance evidence
are reviewed.

