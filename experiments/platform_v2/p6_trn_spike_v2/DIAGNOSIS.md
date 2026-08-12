# P6-v1 diagnosis for the P6-v2 redesign

This document is based on the immutable P6-v1 result and source. P6-v1 was not
rerun or modified. No held-out data were inspected.

## 1. Repetitive-terrain false convergence

P6-v1 accepted 47/50 repetitive-terrain profiles and 46 of those accepted
solutions were false convergences. Their horizontal errors cluster near integer
aliases of the one-metre terrain period: approximately 1.0, 1.4, 2.0, 2.2,
2.8, 3.2, 3.6 and 4.0 m. Their NEES values range from approximately 17,700 to
294,000. This is global data association failure, not poor nominal precision.

The implementation evaluates all hypotheses on a 0.20 m coarse grid but refines
only the single lowest coarse sample. It then compares that refined minimum with
unrefined coarse competitors. On periodic terrain, an equivalent peak sampled
between grid points receives a spuriously higher cost. Consequently, false
aliases passed with best/second cost ratios from 1.19 to 53.29. A ratio or
delta-chi-square test cannot be meaningful unless every competing basin is
refined to comparable numerical precision.

The search disk also creates a boundary effect. An alias near the search edge
can appear unique merely because its competing periodic neighbour lies outside
the evaluated domain. Uniqueness over a truncated hypothesis set is not
evidence of global uniqueness.

## 2. Why local observability did not protect the matcher

Accepted repetitive aliases had minimum local information eigenvalues between
approximately 3,919 and 10,548, much larger than the declared threshold of 5.
The local Jacobian is therefore strongly informative *within an alias basin*.
It cannot distinguish several spatially separated basins with the same profile.
Local Fisher information and global uniqueness answer different questions and
must be gated separately.

## 3. Covariance miscalibration

P6-v1 covariance is the inverse local information matrix plus fine-grid
quantisation. It is conditional on the selected basin being correct. It omits:

- between-basin uncertainty and data-association risk;
- search-boundary truncation;
- correlated model discrepancy not represented by the exponential inflation;
- variation across temporal/profile segments;
- selection effects caused by choosing the minimum among many hypotheses.

This explains the catastrophic NEES on aliases. In nominal informative terrain,
the issue is smaller: the zero-map-error, 0.02 m-noise stratum had one 99%
ellipse miss with only 0.081 m error, and the zero-map-error, 0.10 m-noise
stratum had two misses at 0.308 and 0.322 m. These point to modest conditional
under-dispersion at some noise levels, distinct from the alias failures.

The P6-v1 upper coverage gate was also poorly formulated. With 50 independent
trials and a truly calibrated 99% ellipse, observing 50/50 inside has probability
`0.99^50 = 0.605`. Declaring 100% coverage a failure is therefore not a valid
calibration test. P6-v2 uses a 95% ellipse, exact binomial confidence intervals,
and the NEES distribution on a larger pooled evaluation set.

## 4. Redesign target

P6-v2 does not target lower nominal RMSE. It targets:

1. equal refinement of all plausible spatially distinct minima;
2. explicit global-uniqueness and search-completeness checks;
3. agreement between overlapping profile segments;
4. covariance that includes segment variability and a calibration-only scale;
5. rejection when confidence is not supportable.

