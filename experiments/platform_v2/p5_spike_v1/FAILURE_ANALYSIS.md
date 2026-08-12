# P5 spike v1 failure analysis

## Immutable outcome

`p2v2_p5_spike_v1` remains FAIL/STOP. It achieved 7.6 mm median displacement
error when accepted, 0% false fixes, 10.3 ms median runtime and only 20% clear-
water recovery against the predeclared 80% requirement. The requirement is not
changed and the result is not rerun or replaced.

## Failure-stage diagnostics

The 50 clear-water pose pairs were inspected without changing their seeds,
images, acceptance rule or reported result.

| Diagnostic | Value |
|---|---:|
| median translation magnitude | 0.287 m |
| camera footprint width at 3 m | 3.464 m |
| median AKAZE keypoints, first/second image | 16.5 / 16.0 |
| keypoint ranges | 1–87 / 2–86 |
| median ratio-test matches | 12 |
| median RANSAC inliers | 11.5 |
| pairs with fewer than 20 keypoints in either image | 32/50 |
| pairs with fewer than 20 ratio-test matches | 38/50 |
| pairs with fewer than 20 RANSAC inliers | 40/50 |

The displacement is only 8.3% of footprint width, so the images retain roughly
92% overlap per translated axis. There is no yaw or scale change in this spike.
Crop size, overlap and viewpoint geometry therefore do not explain the 80%
missed-fix rate. Rejection occurs before or at descriptor matching because the
default AKAZE detector usually supplies fewer features than the unchanged
20-inlier verification rule can possibly accept.

## Environment versus front end

Two post-run diagnostic probes were applied to the exact same clear-water image
pairs. They are diagnostics, not substitute results:

| Detector diagnostic | Verified pairs under the same 20-inlier/2 px gate | Median error when verified |
|---|---:|---:|
| original default AKAZE | 10/50 | 7.60 mm |
| AKAZE threshold `1e-4` | 50/50 | 6.71 mm |
| ORB, 1000 features, FAST threshold 10 | 44/50 | 13.0 mm |

Because the same rendered environment supports abundant, geometrically correct
matches under two transparent detector configurations, the current evidence
does not support the conclusion that the seabed representation fundamentally
lacks unique spatial structure. The immediate cause is a mismatch between
default AKAZE sensitivity and the low-modulation multiscale texture after the
radiometric model. The first spike treated an algorithm default as if it were a
sensor-independent choice.

This does not prove the environment is adequate. The diagnostic did not repeat
the false-fix controls, attenuation sweep, repeated-texture case, scale/yaw
change or runtime test for the alternative settings. In particular, lowering a
detector threshold can convert noise into keypoints and may raise false fixes in
degraded water. Those risks must be tested, not inferred away.

## One justified second spike

Exactly one `p2v2_p5_spike_v2` is scientifically justified after migration.
Before it runs, freeze a new seed root and retain the original primary criteria:
clear-water recovery at least 80%, median error below 0.10 m, false-fix rate
below 5%, median runtime below 50 ms and monotonically non-increasing recovery
with attenuation. Use AKAZE threshold `1e-4` as the single proposed front end;
the value is fixed from the failure-stage diagnosis, not searched in spike v2.

Spike v2 must add predeclared strata for yaw, altitude/scale change, partial
overlap, repeated texture, feature-poor texture and non-overlapping negatives.
Report keypoint, match and inlier distributions per attenuation. If any primary
criterion fails, optical localisation retires for this submission with no third
spike. Until such a passed spike exists, `image_localizer` is not a supported
fidelity and no optical-localisation result enters the manuscript.

