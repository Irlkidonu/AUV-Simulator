# AUV Simulator

A ROS 2 / Gazebo simulation framework for unmanned underwater vehicles, built
around mode-aware adaptive navigation: multi-modal sensor fusion, physically
modelled optical degradation, four absolute-positioning techniques with
different infrastructure dependencies, and an ocean environment you can fly
through by hand while breaking it.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Simulator-Gazebo%20Harmonic-orange)
![Tests](https://img.shields.io/badge/tests-176%20passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-green)

**Clone it, build it, and you are flying a vehicle through a reef in about two
minutes.** Turn the water opaque with a slider and watch the navigation mode
escalate. Take the surface vessel away and watch the vehicle fall back on
terrain. Hold a camera the manager wanted to abandon, and watch its position
estimate come apart in front of you.

### Looking for the data?

Campaign CSVs, execution logs, both 144-configuration sweeps, the freeze record
and the checksums are in
**[`src/uuv_mode_aware_navigation/results/`](src/uuv_mode_aware_navigation/results/)**.

**[`RESULTS.md`](RESULTS.md)** is the index: for every file, the command that
produced it, its campaign, seed block, SHA-256, and the reported claims it
supports. Check them all in one line:

```bash
cd src/uuv_mode_aware_navigation/results && sha256sum -c ARTEFACT_SHA256SUMS
```

---

## What is in the box

| | |
|---|---|
| **Vehicle** | six degrees of freedom under the keyboard or a control panel, or flying itself under the mode manager |
| **Sensing** | twelve-state EKF; Doppler velocity log with bottom lock and water track; pressure depth; IMU |
| **Positioning** | four absolute techniques with genuinely different failure conditions: surface-tended USBL, seabed LBL, a single beacon, and terrain-relative navigation needing no infrastructure at all |
| **Optics** | `exp(-2cr)` attenuation, three lamp geometries with real baselines, backscatter that grows with turbidity, and a live degraded camera feed the manager actually reads |
| **Environment** | 45,000-face seabed with relief, reef, wreck, ocean current, fish, jellyfish, kelp that leans downstream |
| **Failures** | five you can assert and release at will, plus nineteen scripted scenario families |
| **Evidence** | headless campaign runner, pre-registered protocol, full configuration sweep, 176 tests |

---

## The idea in one paragraph

Most adaptive underwater navigation adjusts the **estimator** when conditions
degrade: the filter learns to trust the camera less, and the vehicle keeps flying
the same path with the same sensors while becoming less certain where it is. This
framework treats perceptual capability as a **controllable resource** instead. A
mode manager infers which navigational capability state the vehicle is in, using
onboard observables only, and acts to *restore* sensing rather than merely
down-weighting it: change optical channel, change altitude, change acoustic
positioning technique, change measurement-admission strategy, or, when nothing
can restore an observable position, abandon the survey and surface for a
satellite fix.

You do not have to take that on trust. The environment is interactive: assert
each failure yourself, hold a configuration the manager would have abandoned,
and watch the mode escalate and the covariance grow while it happens.

---

## Design commitments

These are enforced in code and tests, not merely documented.

**No prescribed error terms.** Measurement noise, geometric bias, dropout and
availability are *derived* from propagation geometry and water optical
properties. There is no hand-tuned bias vector anywhere in this package. Every
free parameter is either a published physical quantity or an explicitly declared
design choice.

**No privileged information.** The water state — turbidity index, beam
attenuation `c`, optical depth `τ` — is hidden. It is consumed by the physics
model and reported to the evaluator, and it never reaches the estimator, the mode
manager or the controller. `ChannelResponse.navigation_view()` enforces that
boundary in code, and tests assert it.

**No crippled comparators.** Every method shares sensor realisations, estimator,
physics, controller, initialisation and tuning budget. Only the oracle receives
privileged information, and it is labelled an oracle everywhere it appears.

**Determinism.** Same seed, same output, bit for bit. No global RNG is touched.

---

## Where everything is

| Path | What it holds |
|---|---|
| `src/uuv_mode_aware_navigation/` | The ROS 2 package: physics, optics, estimator, mode manager, campaign runner, tests |
| `src/uuv_mode_aware_navigation/results/` | Campaign outputs, both configuration sweeps, freeze record |
| `experiments/` | Read-only analysis: development campaign, held-out comparison, configuration sweep |
| `RESULTS.md` | Every result file: command, seed block, SHA-256, and the claims it supports |
| `NOTICE` | Third-party assets, their licences, and every change made to them |

The method specifications the implementation is checked against — cited by
section number throughout the source and tests as `MODE_MANAGER_SPEC`,
`OPTICAL_PROPAGATION_SPEC`, `EVALUATION_METRICS_SPEC` and `COMPARATOR_SPEC` —
and the pre-registered protocol are available from the corresponding author on
request.

---

## Install

The physics, the campaign and the tests need **no ROS and no Gazebo**. Only the
interactive demonstrator does.

```bash
git clone https://github.com/Irlkidonu/AUV-Simulator.git ~/auv-simulator
```

**Standalone — physics, campaign and tests:**

```bash
pip install numpy pytest
cd ~/auv-simulator/src/uuv_mode_aware_navigation
PYTHONPATH=. python3 -m pytest test/ -q          # 176 tests, about three minutes
```

**As a ROS 2 package**, for the demonstrator. Requires ROS 2 Jazzy and Gazebo
Harmonic on Ubuntu 24.04. Do not skip the `source` lines: without them `ros2`
cannot find the package and the launch fails with a bare "package not found".

```bash
cd ~/auv-simulator
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --build-base .build --install-base .install --symlink-install
source .install/setup.bash
```

`--symlink-install` means later Python edits need no rebuild, and the isolated
overlay cannot disturb anything else in the workspace.

> **Use a private ROS domain** if anything else on your network publishes on
> `/uuv/*`. ROS 2 discovers peers across the whole subnet by default, so two
> people running this at once will drive each other's vehicles:
>
> ```bash
> export ROS_DOMAIN_ID=42
> ```

### A first look at the optics, in five lines

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

---

## Opening the simulator

Two entry points. They differ only in who is flying.

### Interactive — the one to start with

```bash
cd ~/auv-simulator
source /opt/ros/jazzy/setup.bash && source .install/setup.bash
ros2 launch uuv_mode_aware_navigation playground.launch.py
```

That opens **two windows**: Gazebo, with the vehicle's degraded camera docked in
its right-hand panel, and a graphical control panel. Give it around 30 seconds —
Gazebo loads a 45,000-face seabed and a reef before the scene appears.

Pick a failure to fly into by naming its family:

```bash
ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E7
```

**In the control panel**, press `TAKE MANUAL CONTROL`, then:

Six degrees of freedom, on the layout a drone pilot already knows: the left hand
translates, the arrows attitude. Translation and rotation never share a key, so
strafing cannot turn the vehicle and "forward" means the same thing before and
after a manoeuvre.

| key | does |
|---|---|
| `w` `s` | forward / back |
| `a` `d` | strafe left / right, square to the nose, **no turn** |
| `r` `f` | ascend / descend |
| `q` `e` | yaw left / right, on the spot |
| `↑` `↓` | pitch the nose up / down |
| `←` `→` | roll left / right |
| `space` | all stop |

Keys combine: hold `w` and `a` for a diagonal, which is normalised so it is not
faster than a straight run.

Attitude aims, it does not steer. Pitch and roll decide what the camera and the
lamps are pointed at; they never redirect travel, so tilting to look at
something on the seabed cannot turn your next `w` into a dive. Depth is on `r`
and `f` alone. `level the vehicle` on the panel puts roll and pitch back to zero
without moving anything, since nothing here rights the vehicle for you.

On the panel every axis is also a button, and the pad translates.

Breaking things is on the panel, or from another shell:

| topic | does |
|---|---|
| `/uuv/inject_fault` | `dvl`, `acoustic`, `optical`, `vessel_gone`, `no_map` |
| `/uuv/set_turbidity` | water clarity |
| `/uuv/set_current` | ocean current |
| `/uuv/force_channel` | hold an optical channel against the manager's choice |

Moving the turbidity or current sliders, or pressing any fault button, **pauses
the scenario schedule** so your setting is not overwritten — the panel says who
is driving the water. `load scenario` hands it back.

### Hands-off — the manager flies

```bash
ros2 launch uuv_mode_aware_navigation demo.launch.py
```

Same physics, no panel, terminal status display instead. Change the water from
another shell and watch the mode escalate:

```bash
ros2 topic pub --once /uuv/set_turbidity std_msgs/Float32 "{data: 1.6}"
```

### If nothing appears

* **Gazebo exits immediately, log says `uri ... could not be resolved`** — the
  launch file sets `GZ_SIM_RESOURCE_PATH` for you, so this means the package was
  not rebuilt after a mesh was added. Re-run the colcon build.
* **Camera view is black or the window is very slow** — offscreen rendering has
  fallen back to software. The launch files name the NVIDIA EGL vendor already;
  if your GPU is not NVIDIA, remove that line from the launch file.
* **Nothing at all, no error** — `source .install/setup.bash` was skipped, or you
  are in a different `ROS_DOMAIN_ID` from the one you launched with.
* **`not found: .../ros_gz_image/share/ros_gz_image/local_setup.bash`** — harmless.
  The overlay chains to a parent workspace entry that is no longer built. It
  prints once on `source` and affects nothing here.

Closing the Gazebo window shuts the whole session down, including the panel and
the camera view. Nothing is left running.

---

## What is in the world

The scene is not decoration. Most of it is there because the study is about
perception failing, and a vehicle needs something worth looking at before
losing sight of it means anything.

**Seabed.** A 45,000-face mesh with real relief, generated from
`uuv_mode_aware_navigation/seabed.py`. That module is the single definition:
`scripts/make_seabed.py` renders the mesh from it and the vehicle node samples
the same function for altitude, so the surface on screen and the one the
altimeter reads cannot disagree. Relief is gentle inside the survey box (about
0.3 m, keeping clearance at the −17 m survey line) and rises to roughly 8 m
further out, where the vehicle never flies. That matters beyond looks: terrain
matching over a plane is exactly the case that cannot work, so a flat floor made
the study's own subject invisible.

**Life.** Ten fish, twelve jellyfish and twenty-four plants, all driven by
`fish_school.py`:

* **fish** cruise, and bolt at 1.9 m/s when the vehicle comes within 3.2 m,
  turning directly away because that is the direction which opens the range
  fastest;
* **jellyfish** swim by bell contraction — a sharp pulse for the first fifth of
  each cycle, then a passive glide with a slight sink — and barely steer, so
  they go where the water goes;
* **weed** leans downstream in the current and sways, closed-loop on an
  integrated angle so it returns upright when the current drops.

Turn the current up in the control panel and the weed goes over before the
vehicle starts to crab. That is the intended order.

**Structure.** A wreck, a seabed pipeline with a free span, a support vessel on
the surface with its USBL transducer over the side, and an acoustic beacon. The
vessel is not scenery either: ultra-short baseline positioning interrogates a
transceiver held at the surface, and family E18 is the case where that vessel
leaves station.

**Rendering notes**, stated because they are easy to mistake for bugs:

* Gazebo's ogre2 engine **ignores `<fog>`**. The world declares it and it has
  never rendered. Depth falloff comes from the background colour instead, and
  the genuinely underwater view is the vehicle's camera, which the propagation
  model degrades properly.
* The vehicle is **posed from the simulated state** rather than driven by
  Gazebo's velocity-control plugin, which moves only the horizontal axes. One
  motion model therefore serves both the demonstrator and the campaign, and the
  rendered vehicle tracks altitude and attitude exactly.
* Weed **bends from its base**, whole-plant. Moving only the fronds needs a
  rigged mesh or a vertex shader, and the model has neither.
* The **sea surface does not animate**. Waves are geometry, not motion.

Illumination and texture cover the full survey area. Measured over a 300 s
headless run across waypoints 1–7 — the whole 20 m × 18 m box — the image-quality
statistic holds a minimum of 0.177 and a mean of 0.324 over 743 samples, with no
dropout after the camera starts publishing. `test_demonstrator_scene.py` ties the
scene and the mission together so coverage is checked on every run of the suite.

Third-party models and textures are listed in [NOTICE](NOTICE).

---

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

---

## Running the campaign

Every number in the study comes from here, and none of it uses Gazebo.

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

### Validation

The optical model ships with the validation suite from its specification. Two
tests are **gating**: if either fails, the affected claim is dropped rather than
rescued by retuning the physics.

```bash
PYTHONPATH=. python3 -m pytest test/ -q              # all 176
PYTHONPATH=. python3 -m pytest test/ -q -m gating    # those that gate a claim
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

---

## What the interactive scene is for, and what it is not

The Gazebo scene contributes **no statistics**. Every reported number comes from
the headless deterministic runner. What the demonstrator shows is the loop
closing on a *rendered camera frame* rather than on a synthetic quality index:
the water column degrades the render, the optical-feedback node estimates water
condition from pixels alone, and the manager acts on that estimate.

The campaign is not image-free either. With optical feedback enabled — which is
how the reported campaign runs — each decision renders a seabed patch through the
propagation model and estimates water condition back from it. What is absent is
Gazebo, not imagery.

So the scene is not a demo bolted onto a result. It runs the same propagation
model, the same sensor models, the same filter and the same manager the campaign
executes, and it imports the campaign's own scenario definitions rather than
restating them. A session cannot disagree with a campaign about what a scenario
is, because there is only one definition of it.

---

## How to check any of this yourself

Simulation results are easy to publish and hard to trust. Four things here are
meant to make this set checkable rather than merely readable.

**The outcome measure was fixed before any result existed.** A pre-registered
protocol declares the metrics, the seeds and the falsification conditions up
front, and every campaign is reported against them. Source files carry
`PROTOCOL.md` section references where a rule comes from the protocol; the
document itself is available from the authors on request.

**The baseline is not ours to choose.** The fixed policy the method is compared
against is selected by exhaustive sweep of the manager's own action space, and
both sweeps ship with the repository, best to worst. If you think the comparison
was against a weak configuration, open the file, find the one you would have
picked, and read off its score. The development sweep selected
`lidar+terrain_relative@1.0m/0.25mps/weight/continue`; the held-out sweep,
executed afterwards, returns the same winner.

**The comparators are labelled by what they know.** The tuned static baseline
knows before departure which configuration suits conditions it has never met, and
the oracle is handed the true water state; neither could be deployed on a real
vehicle, which is the point of using them. The oracle is a
privileged-information comparator rather than an upper bound — it shares the
manager's decision rule, so perfect information does not make it optimal for the
reported aggregate, and where the ordering departs from expectation that is
reported directly.

**Every number is traceable.** `RESULTS.md` maps each
reported result to the artefact, the seed block and the command that produced it,
with a SHA-256 for every file. The twelve modules that produce the campaign match
the freeze record byte for byte. Held-out execution is gated on that record and
the block was executed once.

The tier ablations show which part of the method carries the effect: restricted
to measurement admission alone it loses a factor of 33 on the aggregate outcome,
and a factor of 254 in an area with no acoustic infrastructure and no prior map.

---

## Held-out data

Held-out execution is gated on a verified freeze record and is not reachable from
the campaign script by an ordinary `--root`. Two roots were reserved,
20,400,000 and 20,800,000; both have been executed, once each, and the freeze
record marks the block spent. A third execution is refused.

```bash
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 scripts/freeze.py --status   # what is covered, and whether spent
PYTHONPATH=. python3 scripts/freeze.py --verify   # does the tree still match the record
```

`--verify` checks the whole tree, and reports the interactive demonstrator as
changed: the launch files, ROS 2 nodes, Gazebo world and scenery generators were
built after the campaigns were frozen. The demonstrator contributes no reported
number, so it continues to evolve while the campaign source stays fixed. To check
the campaign closure specifically, see
[`RESULTS.md`](RESULTS.md) §4.

---

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
comparators.py  fixed, oracle, estimator-only and tier ablations, all sharing
                everything above
   |
mission.py      guidance (estimate only) + scoring (truth only)
campaign.py     deterministic scenario runner
analysis.py     paired statistics, bootstrap intervals, Pareto fronts, aggregate J
```

The boundary that matters runs through `mission.py`: `Guidance` receives the
estimate and cannot reach truth; `MissionEvaluator` receives truth and feeds
nothing back.

---

## Citing

See [`CITATION.cff`](src/uuv_mode_aware_navigation/CITATION.cff).

## Third-party assets

Vehicle mesh, scenery models and photogrammetry textures come from outside this
project and carry their own licences, recorded in full in [NOTICE](NOTICE) along
with every change made to them and why. In short: the BlueROV2 mesh is
Apache-2.0, two scenery models are CC0, and four are Sketchfab Standard, which
permits commercial use and derivative works without attribution. Attribution is
given anyway.

## License

MIT. See [LICENSE](LICENSE).

## References

- **[R2]** *Performance considerations for continuous-wave and pulsed laser line
  scan (LLS) imaging systems.* J. Eur. Opt. Soc. 5, 10020 (2010).
- **[R3]** *Extended Range Underwater Optical Imaging Architecture.*
- **[R4]** Boss et al., *Particulate backscattering ratio at LEO 15*;
  Twardowski et al., Opt. Express 15(11), 7019.
- **[R5]** *Beam attenuation coefficient for different water turbidities.*
  Appl. Opt. 63(24), 6482 (2024).
