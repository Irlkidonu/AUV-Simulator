# P6 terrain-relative navigation feasibility spike v2 — frozen protocol

Status: **FROZEN BEFORE EXECUTION**  
Identifier: `p2v2_p6_trn_spike_v2`  
Fresh development seed root: `22,220,000`  
Maximum executions: **one**

P6-v1 remains an immutable FAIL. V2 is justified by its recorded mechanism:
unrefined competing basins made periodic aliases appear unique, local Fisher
information was mistaken for global uniqueness, and covariance was conditional
on an often-wrong data association. V2 is not intended to improve already-good
nominal accuracy.

## 1. Frozen matcher changes

### 1.1 Equal multi-hypothesis refinement

The search radius is 3.5 m around an initial estimate whose error remains at
most 2.0 m. The additional 1.5 m is a completeness margin exceeding the
one-metre repeated-terrain period.

All strict local minima on the 0.20 m coarse grid are identified using their
eight-connected neighbourhood. Minima within 0.35 m are clustered. Up to the
32 lowest spatially distinct basins are each refined on the same 0.025 m grid
within 0.25 m. No refined candidate is compared with an unrefined candidate.

A solution is rejected as `search_boundary` unless its refined centre is at
least 1.0 m from the search boundary. It is rejected as `ambiguous` if any
other refined basin at least 0.50 m away has a chi-square cost difference below
13.816 (99.9%, two position degrees of freedom), or if the selected basin holds
less than 0.99 of normalized posterior mass among refined basins.

### 1.2 Observability and uniqueness are separate

The existing minimum local-information eigenvalue of 5 is retained. Passing it
only establishes local observability. Global uniqueness additionally requires
the basin and posterior-mass tests above. No amount of local curvature can
override an ambiguity rejection.

### 1.3 Temporal/profile consistency

The 49-sample profile is divided into two overlapping 31-sample windows
(samples 0--30 and 18--48). Each window is matched independently using the same
multi-basin procedure. A full-profile fix is accepted only if:

- both windows have a locally observable, globally unique solution;
- their position estimates differ by at most 0.25 m; and
- their squared disagreement under the sum of their provisional covariances is
  at most 9.210 (99%, two degrees of freedom).

This is an onboard consistency test: it uses only the measured profile and map,
never truth. A rejection is a missed fix, not a false fix.

## 2. Covariance and calibration

The provisional covariance is the sum of:

1. inverse local information at the full-profile solution;
2. fine-grid variance `step^2/12` per axis;
3. the sample covariance of full-profile and two window estimates, when all
   three exist;
4. a between-segment residual sandwich term using the three contiguous profile
   thirds.

No covariance is returned for an ambiguous association.

Calibration uses a disjoint calibration partition at the same development root:
40 informative trials per sensor-noise/map-error stratum. A single scalar
inflation is the maximum of 1 and `q95(NEES)/5.991`, where `q95` is computed
over accepted calibration fixes after ambiguity rejection. The scalar is then
frozen and applied unchanged to all scoring strata. Calibration truth is used
only for this scale, never for matching or acceptance. Calibration trials do
not enter pass/fail performance metrics.

On the scoring partition, report 50%, 90%, 95% and 99% ellipse coverage and
NEES quantiles. Calibration passes when the two-sided 95% Clopper--Pearson
interval for pooled informative 95% coverage contains 0.95 and median NEES lies
in `[0.5, 3.0]`. This replaces P6-v1's invalid requirement that every 50-trial
99% stratum contain at least one miss.

## 3. Fresh deterministic scoring set

Scoring uses new named RNG substreams, disjoint from calibration and P6-v1:

- 100 informative trials for every combination of sensor noise
  `{0.02, 0.05, 0.10} m` and correlated map error `{0, 0.02, 0.05} m`;
- five headings uniformly spanning `[0, pi)`;
- initial horizontal error uniform in area inside a 2.0 m disk;
- 100 flat-terrain trials;
- 200 repeated-terrain trials: 100 one-dimensional sinusoidal ridges and 100
  crossed periodic textures, with randomized phase, heading and initial error;
- 100 near-repetitive trials whose second basin is deliberately worse but
  plausible, to ensure the gate does not reject every structured map;
- 100 truncated-search controls placing an apparent best alias near the search
  boundary.

Reference profiles remain 12 m long at 0.25 m spacing. Sensor and map-error RNG
streams are separate. Pair identities and all seeds are materialized and
checksummed before scoring.

## 4. Required reporting

Report per stratum and pooled:

- fix, ambiguity-rejection, boundary-rejection and consistency-rejection rates;
- number of coarse basins and refined basins;
- posterior mass and first/second refined delta chi-square;
- median, 95th-percentile and maximum horizontal error;
- false-convergence count and exact binomial interval;
- window disagreement and temporal-consistency pass rate;
- covariance eigenvalues, NEES quantiles and 50/90/95/99% coverage;
- calibration inflation determined on the calibration partition;
- median and 95th-percentile single-core runtime.

## 5. Predeclared pass/fail criteria

V2 passes only if all criteria pass:

1. zero false convergences in 200 repeated-terrain trials;
2. at least 95% of repeated-terrain trials are rejected as ambiguous,
   inconsistent or search-incomplete;
3. zero accepted fixes in 100 flat-terrain trials;
4. zero accepted fixes in 100 truncated-search controls;
5. near-repetitive fix rate at least 50%, demonstrating that the method does
   not obtain safety by rejecting all structured terrain;
6. reference informative condition (`0.02 m` noise, zero map error) fix rate at
   least 85%; the reduction from P6-v1's 90% acknowledges the explicitly added
   safety tests and is not tuned to V2 results;
7. reference median error below 0.10 m and 95th percentile below 0.25 m,
   unchanged from P6-v1;
8. false-convergence rate below 1% in every informative scoring stratum;
9. every returned covariance is finite and positive definite;
10. pooled informative 95% ellipse coverage has a two-sided 95% exact binomial
    interval containing 0.95, and pooled median NEES lies in `[0.5, 3.0]`;
11. median runtime below 100 ms and 95th percentile below 200 ms. The doubled
    P6-v1 budget is declared from the cost of equally refining competing basins
    and remains below the manager's 0.5 s decision interval;
12. all legacy critical hashes, the 27-run golden regression and the complete
    test suite remain green.

## 6. Terminal rule

Protocol, implementation, calibration/scoring manifests, output schema and
digests are frozen before execution. Execute once. Interrupted execution may
only resume verified packets from this same root.

If any criterion fails, record **FAIL** and do not integrate this TRN matcher.
There is no P6-v3 during the current submission cycle. Continue unrelated
platform work only after recording the failure. If all criteria pass, record
**FEASIBILITY PASS** only; integration still requires review and does not
constitute held-out validation.
