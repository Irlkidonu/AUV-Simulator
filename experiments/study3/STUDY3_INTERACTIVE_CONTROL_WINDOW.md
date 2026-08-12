# Study 3 interactive real-time control window

## Status and scientific boundary

This is a DEVELOPMENT demonstration and mechanism-testing tool. It is not a
campaign runner and does not write into Study 3 evidence roots. It does not
change FIXED, REACTIVE, PREDICTIVE, P5-v4, scientific thresholds, scenarios, or
held-out data.

The window wraps the existing Study 3 closed-loop simulator. Every manual action
changes only truth-side `PhysicalState` fields. The ordinary image renderer,
DVL sampler, acoustic geometry/packet model, serialized service discovery,
capability filter, mode selector, estimator, and vehicle dynamics remain in the
loop. Policies receive only the resulting `PlatformStepInput` observations.

## Launch

From this repository after an editable install:

```bash
python3 -m pip install -e .
study3-control
```

From a built ROS 2 workspace:

```bash
source install/setup.bash
ros2 run uuv_mode_aware_navigation study3_control
```

Direct module invocation is also supported when the source package is on
`PYTHONPATH`:

```bash
PYTHONPATH=src/uuv_mode_aware_navigation \
python3 -m uuv_mode_aware_navigation.study3.control_window
```

Optional arguments are `--seed`, `--horizon-s`, and `--dt-s`. Defaults are a
900 s demonstration, 1 s simulation step, and development seed 31,895,000.

## Controls

- **START** runs deployment-informed FIXED, REACTIVE, or PREDICTIVE.
- **PAUSE/RESUME** freezes the simulation clock without advancing sensors.
- **RESET** terminates the current demonstration and starts a clean realization.
- **RECORD** arms the visible recording state; pressing **SAVE RECORDING** writes
  a checksummed JSON disturbance record. All actions since session start are
  retained so replay begins from the correct initial physical state.
- **REPLAY** loads that JSON and disables manual influence on the replayed
  environment. Select the desired policy before loading it.
- The speed selector changes wall-clock pacing only; it does not change the
  configured vehicle speed.

Manual physical controls cover:

- turbidity/visibility and complete optical failure/recovery;
- east/north currents;
- DVL bottom-lock probability, water-track probability, noise and crashout;
- ambient acoustic noise and complete acoustic failure/recovery;
- LBL deployment and geometry;
- USBL support-vessel presence/departure;
- compound optical+DVL, acoustic+DVL, and all-horizontal-aiding failures;
- one-click recovery to the benign starting environment.

The live panel displays true and estimated horizontal trajectories, horizontal
error, selected navigation mode and reason, contemporaneous optical/DVL/acoustic
evidence, selected acoustic technique, altitude/depth, estimator position-
covariance trace, mode transitions, mission action, and surfacing/GPS state.

## Deterministic recording and fair replay

Each manual event is stored with its effective simulator step, simulation time,
stable sequence number, control name and value. The record also contains the
complete base-environment configuration, seed, time step, horizon and base
environment digest. A SHA-256 checksum detects edits. Replay regenerates and
verifies the base realization before applying events at the same sensor steps.

The same recording can therefore be replayed separately with
`deployment_fixed`, `reactive`, and `predictive`. Exogenous disturbances are
identical. Vehicle-state-dependent physics (for example altitude-dependent DVL
bottom lock and geometry-dependent acoustics) remains correctly dependent on
each policy's resulting trajectory rather than being artificially copied.

## Mechanism verification

Automated tests verify:

1. controls affect truth-side sensor physics at the next sensor step;
2. LBL/USBL availability and acoustic failure still require serialized response
   evidence rather than exposing control labels;
3. compound failure/recovery actions are fully recorded;
4. recording replay reproduces every physical-state sample exactly and rejects
   checksum tampering;
5. evaluator-only future queries neither advance the live clock nor consume
   future replay events;
6. an end-to-end recorded compound failure removes real optical, DVL and acoustic
   evidence, causes the unchanged REACTIVE manager to pass through optical,
   acoustic, relative/dead-reckoning and terminal modes, changes the selected
   acoustic configuration, and physically commands surfacing.
