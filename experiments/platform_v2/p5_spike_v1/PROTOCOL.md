# P5 optical-localisation feasibility spike — protocol

Identifier: `p2v2_p5_spike_v1`

This is a development feasibility experiment, not a held-out campaign. It uses
the new pose-dependent renderer and seed root `22,100,000`; neither spent Paper
2 held-out root is reachable.

For at least 50 pose pairs in clear water (`c = 0.2 m^-1`) at 3 m altitude:

- A1: repeated rendering is bitwise deterministic;
- A2: median horizontal displacement error is below 0.10 m;
- A3: at least 80% of overlapping pairs yield a verified fix;
- A4: false-fix rate is below 5% on non-overlapping pairs, where verification
  requires at least 20 RANSAC inliers and median reprojection error below 2 px;
- A5: median matching runtime is below 50 ms per pair on one process;
- A6: recovery rate is monotonically non-increasing over attenuation
  `c = 0.2, 0.6, 1.2, 2.0 m^-1`;
- A7: all legacy-protection tests remain green.

Failure of A2, A3 or A4 stops feature-front-end development. Thresholds are not
changed after inspecting the output.

