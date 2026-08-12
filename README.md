# AUV Simulator — mode-aware navigation under conditional sensing

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Simulator-Gazebo%20Harmonic-orange)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)
![License MIT](https://img.shields.io/badge/License-MIT-lightgrey)

Companion software and evidence for the paper on mode-aware adaptive navigation
for unmanned underwater vehicles.

---

## Overview

An underwater vehicle cannot rely on any single source of absolute position.
Every source is conditional, and each is conditional on something different:

* a **Doppler velocity log** needs bottom lock;
* **acoustic positioning** needs deployed infrastructure, workable geometry and
  a quiet enough channel;
* **optical/terrain aiding** needs water clarity and a surveyed patch beneath
  the vehicle;
* **dead reckoning** needs nothing, and drifts without bound.

The usual response is to keep one configuration and let the estimator weigh
whatever arrives. This project asks a different question: what if perceptual
capability is treated as a **managed resource** — something the vehicle reasons
about and reconfigures for — rather than as a fixed property of the platform?

The repository provides a deterministic simulator for that question, an
interactive playground for exploring it by hand, and the complete evidence
behind the results reported in the paper.

---

## Architecture

Navigation mode is selected online from **observable evidence only**. The
selector never receives ground-truth pose, scenario identity, infrastructure
truth, or any future schedule. It sees what the vehicle could actually measure:
acoustic service responses with their uncertainty and geometry, optical
availability and quality, DVL bottom-lock and water-track flags, and filter
covariance.

Six operational modes, in the order the selector prefers them:

| Mode | Selected when |
|---|---|
| `lbl_aided` | An LBL service responds with a usable position fix |
| `usbl_aided` | A USBL service responds with a usable position fix |
| `optical_dvl` | Optical aiding is usable and bottom lock is held |
| `optical_no_bottom_lock` | Optical aiding is usable without bottom lock |
| `relative_dead_reckoning` | No absolute fix is observable; velocity aiding remains |
| `terminal_degraded` | No submerged horizontal mode remains — surface for GPS |

Two properties matter for interpreting the results:

* **Absence of evidence is not evidence of absence.** A service that has simply
  not been re-probed is *stale*; one that was probed and returned no usable
  geometry is *refuted*. Only the second causes the vehicle to abandon a mode.
* **Switching is not free.** Changing acoustic mode costs a probe opportunity,
  so a candidate must be meaningfully better before it displaces the incumbent.
  The margin is derived from the information content of a position fix, not
  tuned against results.

See `experiments/study3/STUDY3_MODE_ARCHITECTURE.md` and
`experiments/study3/STUDY3_CORRECTION_SPECIFICATION_V1.md`.

---

## Implemented policies

All policies share one implementation and one locked configuration. They differ
only in what they may do with it — which is what makes the comparison fair.

| Policy | Behaviour |
|---|---|
| **FIXED** (universal) | One configuration for every condition, chosen in advance and never changed. |
| **Deployment-informed FIXED** | The same configuration, with only the acoustic technique set to the service known to be deployed at launch. No run-time adaptation, and no knowledge of later loss or recovery. This is the demanding comparator. |
| **REACTIVE** | Selects a navigation mode online from present observable evidence. |
| **PREDICTIVE** | REACTIVE plus an observable trend forecast; may act before a projected loss has occurred. |

---

## Installation

Ubuntu 24.04, Python 3.12.

**Headless** — the physics, the campaign runner and the full test suite need
neither ROS nor Gazebo. Nothing needs to be installed; put the package on the
path:

```bash
git clone https://github.com/Irlkidonu/AUV-Simulator.git
cd AUV-Simulator
export PYTHONPATH=src/uuv_mode_aware_navigation
python3 -m pytest src/uuv_mode_aware_navigation/test -q
```

To install instead, use a virtual environment — Ubuntu 24.04 marks the system
Python as externally managed (PEP 668), so a bare `pip install` there will
refuse:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev]'
```

**Environment used to produce the published evidence.** Python 3.12.3, NumPy
1.26.4, OpenCV 4.6.0. The optical front end requires an OpenCV build providing
`cv2.AKAZE_create`; some newer builds omit it, and the runners refuse to
execute rather than silently produce different results.

**As a ROS 2 package**, for the demonstrator and the interactive playground.
Requires ROS 2 Jazzy and Gazebo Harmonic:

```bash
mkdir -p ~/uuv_ws/src && cd ~/uuv_ws
git clone https://github.com/Irlkidonu/AUV-Simulator.git src/auv-simulator
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Quick start

The shortest path to a running simulation, no ROS required:

```bash
export PYTHONPATH=src/uuv_mode_aware_navigation
python3 -m pytest src/uuv_mode_aware_navigation/test -q     # 437 tests
```

To run the graphical demonstrator:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch uuv_mode_aware_navigation playground.launch.py
```

---

## Interactive playground

The Study 3 control window is the fastest way to develop intuition for what the
navigation manager does and why.

```bash
cd <workspace>/auv-simulator
source install/setup.bash
ros2 run uuv_mode_aware_navigation study3_control
```

**Change conditions while the simulation runs.** The window exposes controls for

* optical conditions — turbidity, and outright optical failure;
* ocean currents — east and north components;
* DVL capability — bottom-lock and water-track probability, noise scaling, and a
  hardware crashout;
* acoustic conditions — ambient noise level and LBL geometry quality;
* infrastructure availability — LBL and USBL presence, including a support
  vessel departing mid-mission;
* compound presets — simultaneous optical/DVL, acoustic/DVL, or total loss, and
  a one-click recovery of every control.

**Watch what the vehicle does about it.** The display reports the true and
estimated trajectory, horizontal error and filter covariance, the selected
navigation mode and the reason for it, the observable evidence behind that
choice, the selected acoustic technique, commanded altitude and speed, the
mission action, and any surfacing/GPS commitment.

### Recording and deterministic replay

Every control change you make is timestamped against the simulation clock and
can be saved as a checksummed recording, then replayed exactly — against the
same policy or a different one, which is how policies are compared on an
identical disturbance sequence.

```python
from uuv_mode_aware_navigation.study3.interactive import (
    run_interactive_session, save_recording)
from uuv_mode_aware_navigation.study3 import PolicyKind

# Replay a saved disturbance sequence against a chosen policy.
environment, completion = run_interactive_session(
    policy_kind=PolicyKind.REACTIVE,
    replay_record="experiments/study3/interactive_sessions/S6_usbl_departure_under_turbidity.json",
    pace=False)
print(completion["result"]["overall_rmse_m"])
```

The control window saves recordings through the same interface. Ten recorded
sessions ship with the repository in `experiments/study3/interactive_sessions/`,
covering optical degradation and recovery, bottom-lock and water-track loss,
currents, acoustic noise, LBL degradation, USBL vessel departure, compound
failures and a staged total loss ending in a GPS surfacing.

---

## Paper (Mode-Aware Adaptive Navigation using Multi-Modal SEnsing and OPtical feedback in simulation enviroment) evidence and reproducibility

Everything the paper reports is in `experiments/`, and every result is
checksummed.

### What counts as final evidence

The principal final evaluation is the **corrected-controller held-out block at
seed root 36,000,000**, executed exactly once:

```
experiments/study3/redesign_results/heldout_v2/          2010 immutable packets
experiments/study3/redesign_results/heldout_v2_analysis_generated.json   Part B
experiments/study3/redesign_results/heldout_v2_analysis_scripted.json    Part A
experiments/study3/final_tables/                          all final numbers
```

It has two parts, reported separately and **never pooled**:

* **Part A — scripted degradation families.** 810 runs. A robustness and
  reproducibility evaluation; predeclared as *not* required to show superiority.
* **Part B — generated changing environments.** 1200 runs over 400 stochastic
  environment realisations. The primary claim is decided here.

The decision rules were frozen *before* the block was authorised and are in
`experiments/study3/STUDY3_HELDOUT_V2_DESIGN.json`; their evaluation is in
`experiments/study3/final_tables/final_decision_record.json`.

An **earlier held-out block at root 32,000,000** is also published. It measures
the **pre-correction** controller and is retained for provenance. It was not
revised and not re-run. Do not combine the two.

### Studies 1 and 2

`RESULTS.md` maps every Study 1 and Study 2 number to the artefact, seed block,
command and checksum that produced it. Those studies characterise
configuration, failure families and architecture across 19 scenario families and
two exhaustive 144-configuration sweeps; Study 3 is the principal final
evaluation.

### Analysis scripts and machine-readable results

| Path | Contents |
|---|---|
| `experiments/study3/*.py` | Runners, analysers and the freeze locks |
| `experiments/study3/final_tables/*.csv`, `*.json` | Contrasts with 95% CIs, means, per-family summaries, adaptation, switching cost, safety and completion |
| `experiments/analyse_campaign.py`, `experiments/analyse_held_out.py` | Study 1–2 analysis |
| `src/uuv_mode_aware_navigation/results/` | Study 1–2 campaign CSVs |

### Verifying it yourself

```bash
# frozen-file manifests and packet integrity for both held-out blocks
python3 experiments/study3/verify_lock.py        # original 32M block
python3 experiments/study3/verify_lock_v2.py     # final 36M block

# regenerate every final table from the immutable packets
python3 experiments/study3/build_final_tables.py
```

Both locks end with `held-out output already present`. That is the one-shot
guard correctly reporting a **spent** block, not a failure — everything above
that line must pass. `build_final_tables.py` re-verifies all 2820 held-out
packet checksums as it runs.

Manifests and checksums live in
`experiments/study3/STUDY3_FREEZE_MANIFEST_V1.json` (pre-correction),
`experiments/study3/STUDY3_FREEZE_MANIFEST_V2.json` (final),
`experiments/study3/FINAL_TABLES_SHA256SUMS.txt`,
`experiments/platform_v2/SHA256SUMS`, and
`src/uuv_mode_aware_navigation/results/PRE_CAMPAIGN_BASELINE.sha256`.

`experiments/study3/README.md` is the full evidence index.

### Null and adverse results

They are published alongside the favourable ones. The final block shows no
superiority over deployment-informed FIXED on scripted families; PREDICTIVE
takes pre-emptive actions but shows no error benefit over REACTIVE and is worse
on aiding continuity; adaptation is not free, costing mode switches, physical
interventions and survey coverage; and absolute mission completion in generated
environments is below 0.44 for every policy. See
`experiments/study3/STUDY3_DEVELOPMENT_NULL_FINDINGS.md` and
`experiments/study3/STUDY3_SWITCHING_AND_PREDICTIVE_INVESTIGATION_V1.md`.

---

## Repository structure

```
src/uuv_mode_aware_navigation/   ROS 2 package: physics, optics, acoustics,
                                 estimator, mode selector, policies, campaign
                                 runner, interactive playground, 437 tests
  ├─ uuv_mode_aware_navigation/study3/   mode selector, policies, interactive
  ├─ launch/                             demo and playground launch files
  ├─ models/, worlds/                    demonstrator scenery
  └─ results/                            Study 1–2 campaign outputs

experiments/
  ├─ study3/                     principal final evaluation and its evidence
  │   ├─ redesign_results/       immutable result packets, incl. both held-out blocks
  │   ├─ final_tables/           final numbers as CSV/JSON
  │   └─ interactive_sessions/   recorded disturbance sequences
  └─ platform_v2/                sensing and platform characterisation spikes

benchmarks/    regression baselines used by the test suite
tools/         analysis helper used by the test suite
RESULTS.md     Study 1–2 result map
```

---

## Testing and verification

```bash
export PYTHONPATH=src/uuv_mode_aware_navigation

python3 -m pytest src/uuv_mode_aware_navigation/test -q      # 437 tests
python3 experiments/study3/verify_lock_v2.py                 # evidence integrity
python3 experiments/study3/build_final_tables.py             # regenerate tables

cd experiments/study3/final_tables \
  && sha256sum -c ../FINAL_TABLES_SHA256SUMS.txt             # table checksums
```

The suite runs on Ubuntu 24.04 with Python 3.12 and needs neither ROS nor
Gazebo.

---

## Citation

Machine-readable metadata is in `src/uuv_mode_aware_navigation/CITATION.cff`.

```bibtex
@software{alexandris_auv_simulator,
  author  = {Alexandris, Christos and Papageorgas, Panagiotis},
  title   = {{AUV Simulator}: a {ROS} 2 / {Gazebo} framework for mode-aware
             {AUV} navigation under conditional sensor and infrastructure
             availability},
  year    = {2026},
  url     = {https://github.com/Irlkidonu/AUV-Simulator}
  % version, DOI and archived release to be added on publication
}
```

The accompanying paper's citation and DOI will be added here once published.

---

## License

MIT — see `LICENSE`.

Third-party scenery assets bundled for the demonstrator carry their own terms,
recorded in `NOTICE`. None of them affect any reported result.
