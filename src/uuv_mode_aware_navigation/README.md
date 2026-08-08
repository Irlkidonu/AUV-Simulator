# uuv_mode_aware_navigation

Mode-aware adaptive navigation for underwater vehicles under multi-modal sensing
degradation.

Companion software for the study *Mode-Aware AUV Navigation under Conditional
Sensor and Infrastructure Availability: Simulation and Characterization*.

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Simulator-Gazebo%20Harmonic-orange)

---

## What this is

Most adaptive underwater navigation adjusts the **estimator** when conditions
degrade: the filter learns to trust the camera less. The vehicle keeps flying the
same path with the same sensors, and simply becomes less certain about where it
is.

This package does something different. It treats the vehicle's **perceptual
capability as a controllable resource**. A mode manager infers which navigational
capability state it is in, using onboard observables only, and then acts to
*restore* sensing rather than merely down-weighting it:

| Action | Physical mechanism |
|---|---|
| Enable off-axis lighting | Moves the lamp cone out of the near-field backscatter volume |
| Switch camera → laser profiler | Collimated beam and range gating suppress backscatter |
| **Reduce altitude** | Optical depth is `2·c·h`, so descending shortens the light path exponentially |
| Reduce speed | Longer integration, more feature overlap |
| Switch absolute-positioning technique | Each depends on different infrastructure, so they fail for unrelated reasons |
| Hold for an absolute fix | Trades path length for a bounded position solution |
| Surface for a satellite fix | Terminal: abandons the survey when no configuration can restore an observable position |

Each is paid for in mission currency — survey swath, time, power, collision
margin, and the infrastructure a technique depends on — so the manager is solving
a navigation problem, not a filtering problem. Results are therefore reported as
**mission outcomes**, not estimator RMSE.

The absolute-positioning axis carries the study's central asymmetry. Its four
members differ less in accuracy than in what each requires somebody to have
provided:

| Technique | Requires | Fails when |
|---|---|---|
| Ultra-short baseline | A transceiver held at the surface | The support vessel leaves station |
| Long baseline | Four surveyed seabed transponders | The array was never laid, or geometry degrades |
| Single beacon | One seabed transponder | Same, and it constrains only range |
| Terrain-relative | A prior bathymetric map and seabed relief | The seabed is featureless |

Because they fail for unrelated reasons and at unrelated times, a vehicle that
can tell *which* has stopped working is more capable than one carrying the best
of them. That is what the manager is for.

## Design commitments

These are enforced in code and tests, not just documented.

**No prescribed error terms.** Measurement noise, geometric bias, dropout, and
availability are *derived* from propagation geometry and water optical
properties. There is no hand-tuned bias vector anywhere in this package. Every
free parameter is either a published physical quantity or an explicitly declared
design choice.

**No privileged information.** The water state — turbidity index, beam
attenuation `c`, optical depth `τ` — is hidden. It is consumed by the physics
model and reported to the evaluator, and it never reaches the estimator, the mode
manager, or the controller. `ChannelResponse.navigation_view()` enforces that
boundary in code, and tests assert it.

**No crippled comparators.** Every method shares sensor realisations, estimator,
physics, controller, initialisation, and tuning budget. Only the oracle receives
privileged information, and it is labelled an oracle everywhere it appears.

**Determinism.** Same seed, same output, bit-for-bit. No global RNG is touched.

## Install

```bash
# Standalone (physics, campaign and tests — no ROS required)
pip install numpy pytest
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 -m pytest test/ -q          # 173 tests

# As a ROS 2 package, built into an isolated overlay so it cannot disturb
# anything else in the workspace
cd ~/auv-simulator
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --build-base .build --install-base .install --symlink-install
source .install/setup.bash
```

## Quick start

```python
from uuv_mode_aware_navigation.optics import (
    WaterState, CAMERA_COAXIAL, CAMERA_OFFAXIS, LIDAR, channel_response,
)

water = WaterState(c=1.20)          # degraded water, m^-1

for cfg in (CAMERA_COAXIAL, CAMERA_OFFAXIS, LIDAR):
    high = channel_response(water, altitude_m=3.0, config=cfg)
    low  = channel_response(water, altitude_m=1.0, config=cfg)
    print(f"{cfg.name:<16} τ@3m={high.tau:5.1f}  τ@1m={low.tau:5.1f}")
```

## Validation

The optical model ships with the validation suite from its specification.
Two tests are **gating**: if either fails, the affected claim is dropped from the
paper rather than rescued by retuning the physics.

```bash
PYTHONPATH=. python -m pytest test/ -q              # all
PYTHONPATH=. python -m pytest test/ -q -m gating    # those that gate a claim
```

Run a development campaign:

```bash
PYTHONPATH=. python3 scripts/run_campaign.py --seeds 10 --out results/dev.csv
```

| Test | Checks |
|---|---|
| V1 | Contrast falls monotonically with attenuation and altitude |
| V2 | Clear water is signal-limited; opaque water yields no confident fix |
| V3 | Off-axis lighting raises contrast **and** costs a non-zero bias — never a free win |
| **V4** | **Non-nesting envelopes**: a region exists where the laser works and the camera does not, and the camera retains a rate/power regime of its own |
| **V5** | **The altitude lever**: descending restores camera availability in the transition band |
| V6 | Determinism; the global NumPy RNG is never disturbed |
| V7 | Hidden state does not reach the navigation side |

Pipeline-level tests additionally enforce the information boundary by inspecting
the runner's own source: guidance is passed `estimator.position` and never
`vehicle.position`, `Observables` cannot carry water state or truth, and no
evaluator output reaches a decision. These are the evidence that this study
measures navigation rather than localization, and they are part of the freeze
record.

## The physics, briefly

Everything reduces to **optical depth in attenuation lengths**, `τ = c · L`,
where `L ≈ 2h` is the two-way path to the seabed. Each configuration has a
published maximum usable `τ`:

| Configuration | Usable range | Source |
|---|---|---|
| Camera, lamp adjacent (coaxial) | 1–2 attenuation lengths | [R3] |
| Camera, lamp separated (off-axis) | ~3 attenuation lengths | [R3] |
| Laser line scan | 5–6 attenuation lengths | [R2] |
| Range-gated pulsed laser | up to 7 attenuation lengths | [R3] |

Because that published ladder does not nest, there are genuinely different
water/altitude states where each configuration is the right choice — which is
what makes the manager's decision real rather than manufactured by parameter
choice.

The single normalisation constant the relative-units radiometry needs is not
chosen but *solved*, so that a coaxial camera reaches its contrast floor exactly
at its published limit. Off-axis and laser configurations then reach further
purely because their geometry admits less backscatter.

Full derivation, parameter provenance and limitations are given in the paper.
The method specifications the implementation is checked against
(`OPTICAL_PROPAGATION_SPEC`, `MODE_MANAGER_SPEC`, `EVALUATION_METRICS_SPEC`,
`COMPARATOR_SPEC`), cited throughout the source and tests, are available from
the corresponding author on request.

## Gazebo world

`worlds/mode_aware_survey.sdf` is the qualitative demonstration scene. It
contributes **no statistics** — every reported number comes from the headless
deterministic runner.

The vehicle carries both lamp positions used in the study, a coaxial lamp beside
the camera and an off-axis lamp on a 0.35 m baseline, so the geometry behind the
off-axis result is visible in the render rather than existing only in the maths.

What it shows is the loop closing on a *rendered camera frame* rather than on a
synthetic quality index: the water column degrades the render, the
optical-feedback node estimates water condition from pixels alone, and the mode
manager acts on that estimate.

### Opening it — one command

```bash
cd ~/auv-simulator
source /opt/ros/jazzy/setup.bash
source .install/setup.bash
ros2 launch uuv_mode_aware_navigation demo.launch.py
```

Arguments: `turbidity_c` (default `0.2`), `headless` (default `false`).

### Opening it — step by step

The launch file starts everything at once. When something misbehaves it is
easier to bring the pieces up separately, and this is the sequence that was used
to bring the demonstrator up for the first time.

```bash
cd ~/auv-simulator
source /opt/ros/jazzy/setup.bash && source .install/setup.bash

# Use a private ROS domain. The workspace hosts other UUV stacks that publish on
# overlapping /uuv/* topic names; without this the demonstrator will both read
# and corrupt their traffic.
export ROS_DOMAIN_ID=42

W=.install/uuv_mode_aware_navigation/share/uuv_mode_aware_navigation
M=$W/models

# 1. Simulator. Drop -s for the GUI; -s is server-only and still renders the
#    camera, it just opens no window.
gz sim -s -r -v 1 $W/worlds/mode_aware_survey.sdf &

# 2. Bridge: camera and odometry in, velocity command out.
ros2 run ros_gz_bridge parameter_bridge \
  "/paper2/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/model/bluerov2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  --ros-args -r /paper2/camera:=/camera/image_raw &

# 3. The five nodes.
ros2 run uuv_mode_aware_navigation water_column    --ros-args -p turbidity_c:=0.2 &
ros2 run uuv_mode_aware_navigation optical_feedback --ros-args -p model_path:=$M/optical_feedback.json &
ros2 run uuv_mode_aware_navigation mode_manager     --ros-args -p availability_model_path:=$M/availability.json -p decision_period_s:=0.5 &
ros2 run uuv_mode_aware_navigation vehicle          --ros-args -p turbidity_c:=0.2 &
ros2 run uuv_mode_aware_navigation status_display &
```

### What you should see

`status_display` prints a live panel: the inferred capability mode and the reason
for the current decision, the optical quality estimated from the camera image
alone, the selected channel with commanded altitude and speed, DVL bottom-lock
state and acoustic fix age, the covariance trace, and — clearly separated as
evaluator-only — the true position error.

Healthy operation looks like `M0_NOMINAL`, altitude holding at 3.00 m, a
covariance trace near zero, and the waypoint index advancing.

### Change the water and watch it respond

```bash
ROS_DOMAIN_ID=42 ros2 param set /vehicle turbidity_c 1.6
```

Optical quality collapses within a second or two, the mode escalates, and the
manager reconfigures. This is the demonstration the study exists to make.

### Checking the loop is genuinely closed

```bash
export ROS_DOMAIN_ID=42
ros2 topic hz /camera/image_raw              # frames arriving from Gazebo
ros2 topic echo /uuv/optical_quality --once  # quality estimated from those frames
ros2 topic echo /uuv/nav_mode --once         # the mode inferred from it
```

If `/camera/image_raw` is silent the bridge is not running. If it publishes but
optical quality stays at `0.000`, the rendered frame has no contrast — which
happens wherever the survey area is unlit.

### How the demonstrator moves the vehicle

Motion is integrated by the vehicle node using **the same kinematics the campaign
uses**, and Gazebo renders the result. Gazebo's `VelocityControl` plugin drives
only the horizontal axes — commanding `{x: 0.2, z: 0.3}` returns
`x = 0.19999999999953` and `z = 0.0` exactly — so depth would otherwise be
unavailable. Integrating in one place keeps the demonstrator and the campaign on
a single motion model, and removes a whole class of boundary mismatch between
them: frame conventions, spawn offsets and differentiation intervals.

The demonstrator is for seeing and steering the environment. It reports no
number in the paper; every published result comes from the headless campaign.

### Scene coverage

Illumination and texture cover the full survey area. Measured over a 300 s
headless run across waypoints 1–7 — the whole 20 m × 18 m box — the image-quality
statistic holds a **minimum of 0.177** and a mean of 0.324 over 743 samples, with
no dropout after the camera starts publishing.
`test_demonstrator_scene.py` ties the scene and the mission together so coverage
is checked on every run of the suite.

## Running the campaign

Every number in the paper comes from here, and none of it uses Gazebo.

```bash
cd src/uuv_mode_aware_navigation

# Verify the source is unmodified before spending hours on it.
# Silence means nothing changed; any output names the file that did.
sha256sum -c results/PRE_CAMPAIGN_BASELINE.sha256 | grep -v ': OK$'

# 190 scenarios x 144 static configurations, then the comparator campaign.
# Roughly five hours on four physical cores; progress every 500 runs with an ETA.
PYTHONPATH=. python3 -u scripts/run_campaign.py --seeds 10 --jobs 7 \
    --out results/campaign.csv > results/campaign.log 2>&1 &

tail -f results/campaign.log
```

### The held-out block

Seed root `20,800,000`, twenty seeds per family, executed **once**. Study 1's
block (`20,400,000`) is spent and stays permanently unreachable: every reserved
root is refused to `--root`, because a spent block that can be re-entered is a
development block and every number published from it would have to be reported
as one. The gate is
code, not a convention: `--held-out` refuses unless a freeze record exists and
still matches the tree, and refuses again once the block has been spent.
Reaching the same seeds through `--root` is refused outright, because that path
would consume the block without recording that it had been consumed.

```bash
PYTHONPATH=. python3 scripts/freeze.py --status   # what is covered, and whether spent
PYTHONPATH=. python3 scripts/freeze.py --verify   # does the tree still match the record
PYTHONPATH=. python3 scripts/freeze.py --write    # at the freeze; refuses if tests are red

# Roughly ten hours: twice the development seed count, so twice the sweep.
PYTHONPATH=. python3 -u scripts/run_campaign.py --held-out --seeds 20 --jobs 7 \
    --out results/held_out.csv > results/held_out.log 2>&1 &
```

`--reuse-sweep` is refused here. The sweep is what selects the tuned fixed
comparator `C1` and what bounds the hindsight oracle, so reusing one computed on
development seeds would carry a development choice into the held-out result and
would leave the oracle without a per-seed ceiling to be computed from.

### The demonstrator figure

Attaches to a demonstrator that is already running and steps it through three
water conditions, recording the frame and the quality the feedback node
estimated from it:

```bash
ros2 launch uuv_mode_aware_navigation demo.launch.py headless:=true &
PYTHONPATH=. python3 scripts/capture_demonstrator_figure.py --turbidity 0.2 0.8 1.6
```

## Architecture

```
optics.py       propagation physics -> channel availability, sigma, bias
   |
sensors.py      five modalities + discrete fault injection
   |
estimator.py    ONE twelve-state EKF, shared by every method
   |
modes.py        capability inference + transition stability
availability.py counterfactual "what would I see if I moved / switched?"
   |
manager.py      constrained one-step selection over the action space
comparators.py  C1-C5 + ablations, all sharing everything above
   |
mission.py      guidance (estimate only) + scoring (truth only)
campaign.py     deterministic scenario runner
analysis.py     paired statistics, bracket, aggregate J
```

The boundary that matters runs through `mission.py`: `Guidance` receives the
estimate and cannot reach truth; `MissionEvaluator` receives truth and feeds
nothing back.

## Reproducibility

Campaign artifacts are **not** committed — see `.gitignore`. Every reported
result is regenerable from a recorded seed, a frozen parameter set, and a
verified freeze record. Development and held-out seed sets are disjoint and the
held-out set is executed once, after the freeze.

## Citing

See [`CITATION.cff`](CITATION.cff).

## License

MIT — see [`LICENSE`](LICENSE).

## References

- **[R2]** *Performance considerations for continuous-wave and pulsed laser line
  scan (LLS) imaging systems.* J. Eur. Opt. Soc. 5, 10020 (2010).
- **[R3]** *Extended Range Underwater Optical Imaging Architecture.*
- **[R4]** Boss et al., *Particulate backscattering ratio at LEO 15*;
  Twardowski et al., Opt. Express 15(11), 7019.
- **[R5]** *Beam attenuation coefficient for different water turbidities.*
  Appl. Opt. 63(24), 6482 (2024).
