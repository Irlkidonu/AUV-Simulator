# P5 optical-localisation feasibility spike v3 — corrected execution protocol

Status: **FROZEN BEFORE EXECUTION**  
Identifier: `p2v2_p5_spike_v3`  
Fresh development seed root: `22,120,000`  
Maximum executions: **one**

P5-v2 remains an immutable execution FAIL and is not a scientific result. V3
corrects exactly one software defect: a similarity transform returned by OpenCV
with non-finite coefficients or scale at or below `1e-8` is rejected as failed
geometric verification before camera-pose or covariance conversion. It is never
accepted, repaired, clamped or assigned an artificial covariance.

All scientific settings and gates are unchanged from frozen P5-v2:

- AKAZE MLDB, full descriptor, three channels, threshold `1e-4`, four octaves,
  four octave layers and PM_G2 diffusivity;
- Hamming matching, 0.75 ratio, similarity transform, 2 px RANSAC threshold,
  0.995 confidence and 2000 iterations;
- at least 20 inliers, median reprojection below 2 px, at least 15% hull area
  in both images, three quadrants with three points, scatter eigenvalue at least
  `0.04*192^2`, scale `[0.60,1.67]`, positive covariance and maximum one-sigma
  axis below 0.10 m;
- the same T/Y/S/P/W1/W2/R/F strata, 50 pairs each, and the same 200-negative
  composition, generated at the fresh root;
- T success at least 90%; combined Y/S at least 80%; P at least 70%; unchanged
  translation/yaw/scale error limits; zero negative false fixes; below 1%
  positive false fixes per stratum; at least 95% repeated and feature-poor
  rejection; monotonic attenuation; 85--99% clear-water 95% ellipse coverage;
  median runtime below 50 ms and 95th percentile below 100 ms;
- complete legacy compatibility.

The v3 manifest, wrapper, frozen P5-v2 dependency, regression tests and output
schema are checksummed before execution. Run once. If any scientific criterion
fails, record FAIL and stop optical-localizer development for this submission.
A pass establishes feasibility only and is not held-out validation.
