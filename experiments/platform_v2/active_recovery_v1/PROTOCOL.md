# Closed-loop active/predictive recovery development experiment

Status: **FROZEN BEFORE EXECUTION**  
Identifier: `platform_v2_active_recovery_v1`  
Development seed root: `22,240,000`  
Held-out: **no**

## Question

Can an observable quality trend trigger a physical altitude change early enough
to preserve optical aiding as a vehicle enters a spatial turbidity plume?

The experiment compares three policies on identical new development streams:
fixed altitude, reactive recovery, and predictive recovery. The active action
changes vehicle altitude at a bounded 0.25 m/s; quality is then recomputed from
the resulting two-way optical path. Merely changing a sensor label cannot
restore the fix.

## Conditions

- 30 paired seeds per policy, root `22,240,000`;
- 20 m straight survey at nominal 0.5 m/s and 0.1 s integration;
- attenuation rises spatially from 0.12 to 0.52 1/m around x=8 m;
- initial altitude 3 m, minimum altitude 1 m;
- availability requires observed noisy quality at least 0.25;
- prediction horizon 10 s; reactive receives no trend, predictive receives the
  onboard finite-difference trend;
- optical position noise 0.10 m; dead-reckoning bias is independently seeded;
- first-order kinematics are used because no defensible six-DOF coefficient set
  exists in the repository.

Report mission completion, aiding-loss duration, recovery latency, position
error, maximum covariance, altitude transitions and safety violations.

## Pass/fail

All must pass:

1. predictive completion at least 90%;
2. zero predictive safety violations;
3. predictive median aiding-loss duration is below reactive and fixed;
4. predictive median maximum position error is below fixed;
5. predictive acts before the first unavailable fix in at least 80% of seeds;
6. reactive altitude action is reached in at least 80% of seeds;
7. identical seeds reproduce byte-identical numeric results;
8. legacy hashes and tests remain green.

This development experiment is executed once. Failure is retained and not
tuned on the same seeds. It does not authorize a held-out claim.
