# P6 terrain-relative navigation feasibility spike — protocol

Status: design only. Implementation and execution begin only after migration to
the single AUV-Simulator codebase and compatibility-gate approval.

## Representations and observation

- Reference map: the existing continuous `seabed.height_at(x,y)` surface,
  sampled through a `BathymetryMap` interface at 0.10 m map cells.
- Observation: nadir single-beam range to bottom, converted to a depth profile
  using pressure depth. Noise is independent zero-mean Gaussian at 0.02 m,
  0.05 m and 0.10 m one sigma.
- Map uncertainty: separate 0, 0.02 m and 0.05 m correlated vertical-error
  fields with correlation length at least 1 m. Sensor and map error use separate
  named RNG substreams.
- Profile: 12 m trajectory, sampled every 0.25 m, yielding 49 samples. This
  spans several wavelengths of the existing 0.24 and 0.52 rad/m relief layers;
  a shorter profile cannot distinguish translation from local slope reliably.

## Initial uncertainty and search

- Horizontal initial error is sampled uniformly in a disk of radius 2 m.
- Search region is a 2.5 m radius around the initial estimate, leaving 0.5 m
  margin beyond the declared initialization error.
- Coarse-to-fine normalized least-squares correlation: 0.20 m coarse grid,
  then 0.025 m refinement within 0.25 m of the best coarse candidate.
- At least five trajectory headings span 0 to pi because a one-dimensional
  profile can be observable along one heading and ambiguous along another.

## Terrain cases

1. Informative: the existing multi-scale bathymetry where the 2-D profile
   information matrix is full rank.
2. Flat: constant depth. A correct matcher must return unavailable.
3. Repetitive: a sinusoidal ridge field with at least two correlation peaks of
   similar height inside the search region. A correct matcher must report
   ambiguity rather than choose one silently.

## Output and false convergence

Each match returns position, 2x2 covariance, normalized residual, best/second-
best likelihood ratio, minimum information eigenvalue, samples used and runtime.
A false convergence is a returned `success` whose horizontal error exceeds
0.50 m or whose truth lies outside the reported 99% covariance ellipse. Missed
fixes and false convergences are reported separately.

## Acceptance criteria and geometric justification

- Flat-terrain success rate: exactly 0%. Translation is unobservable when every
  map sample is identical.
- Repetitive-terrain false convergence: below 5%; ambiguous peak ratio must
  produce `unavailable`. The 5% cap permits finite-sample noise excursions but
  rejects a matcher that usually commits to an alias.
- Informative-terrain fix rate: at least 90% at 0.02 m sensor noise and zero map
  error. The 2 m initialization lies inside the search region and the 12 m
  profile spans multiple relief scales, so lower availability would indicate a
  matcher/representation failure rather than insufficient search support.
- Median error: below 0.10 m in that reference condition. This equals one map
  cell and four fine-grid steps; larger median error would not improve on the
  discretized representation.
- 95th-percentile error: below 0.25 m, the sample spacing. Errors larger than one
  along-track sample are operationally distinguishable from the correct match.
- Informative false convergence: below 1% in every noise/map-error stratum.
- Covariance: positive definite for every success; empirical 95% ellipse
  coverage between 90% and 99%. The interval permits finite-sample variation
  while rejecting both unsafe overconfidence and vacuous covariance.
- Runtime: median below 50 ms and 95th percentile below 100 ms on one CPU
  process. The manager decides every 0.5 s, so this reserves at least 80% of the
  decision interval for estimation and control.

Use a new development root. Freeze the implementation, strata and thresholds
before execution. A failed spike is reported and not tuned against the same
cases.

