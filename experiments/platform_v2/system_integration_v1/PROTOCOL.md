# Platform-v2 system-integration development protocol

Identifier: `p2v2_system_integration_v1`  
Seed root: `22,300,000`  
Data class: development only; no held-out roots

The experiment exercises one closed loop joining the selected P5-v4 runtime
adapter, DVL lock/water-track signals, geometry/infrastructure-aware acoustic
signals, fixed-lag delayed acoustic replay, probabilistic capability belief,
capability trend prediction and active recovery.
Recovery decisions feed the declarative selector, and the selected
speed/altitude/mission action is applied on the following closed-loop step.

Five deterministic families are run with 12 fresh seeds each: optical
degradation/recovery, total DVL loss/recovery, acoustic infrastructure loss,
delayed asynchronous acoustics, and compound optical+DVL+acoustic transitions.
The manager receives only simulated onboard observables. Scenario phase labels
are retained by the evaluator and never passed into capability inference.

Development checks require: decreasing capability belief during every declared
loss; recovery after restored evidence where physically recoverable; prediction
before at least one optical or DVL loss; a physically matching recovery action;
accepted fixed-lag acoustic packets with nonzero delay; rejection of unavailable
infrastructure; finite covariance throughout; and exact deterministic replay.

This protocol does not tune or evaluate TRN, six-DOF coefficients, energy-aware
optimization, Study 2 legacy evidence, or any final held-out claim.
