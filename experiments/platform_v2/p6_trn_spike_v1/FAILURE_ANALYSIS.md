# P6 terrain-relative navigation feasibility spike — terminal result

Date: 2026-08-10  
Status: **FAIL — one permitted execution consumed; no same-root rerun**

The frozen development spike at seed root `22,200,000` failed three predeclared
scientific criteria. The result is retained exactly as produced in
`result.json` (SHA-256
`6ac62e08bbbcdbb28a6ec1d32bb25a71efa129dbd563dfc6f05de9e7a91ffa53`).

## Failed gates

1. Informative-terrain false convergence was not below 1% in every stratum.
   The nominal `0.02 m` noise / zero-map-error stratum had one covariance
   miss in 50 trials (2%), despite all horizontal errors remaining below
   0.082 m. The `0.02 m` noise / `0.05 m` map-error stratum included one
   accepted 2.03 m alias. The `0.10 m` / zero-map-error stratum had two
   covariance misses in 50 trials (4%).
2. Per-stratum 99% ellipse coverage did not remain in the predeclared
   `[0.90, 0.99]` interval. Several strata produced 100% coverage, indicating
   conservative covariance at this sample size, while the failures above show
   that the same model does not safely describe every accepted hypothesis.
3. Repetitive-terrain false convergence was 92% (46/50), against the below-5%
   requirement. The best/second-hypothesis test did not recognize aliases
   separated by integer terrain periods because the finite search lattice and
   spatial-separation rule did not reliably expose the equivalent peak.

## Findings that remain valid

The reference informative condition passed its availability and accuracy gates:
100% fix rate, 0.0277 m median error and 0.0627 m 95th-percentile error. Flat
terrain was rejected in 50/50 trials. Median and 95th-percentile runtime were
15.4 ms and 16.8 ms. These are characterization findings only and do not
override the terminal FAIL.

## Consequence

The current terrain matcher is not safe as a platform-v2 aiding source because
it can return confident aliases on repeated terrain. It must not be integrated
into navigation or used for a manuscript performance claim. In accordance with
the user's stop rule, downstream roadmap implementation stops at this failed
scientific gate. Any future redesign must use a new identifier, new development
root and an independently frozen protocol; this result is never overwritten or
rerun.
