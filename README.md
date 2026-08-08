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

## Where everything is

| Path | What it holds |
|---|---|
| `src/uuv_mode_aware_navigation/` | The ROS 2 package: physics, optics, estimator, mode manager, campaign runner, tests |
| `src/uuv_mode_aware_navigation/README.md` | **Full run instructions**, architecture, physics notes |
| `experiments/` | Read-only analysis: development campaign, held-out comparison, configuration sweep |
| `PUBLICATION_ARTEFACT_MANIFEST.md` | Every result file: command, seed block, SHA-256, and the claims it supports |
| `src/uuv_mode_aware_navigation/results/` | Campaign outputs, including the full configuration sweep |
| `NOTICE` | Third-party assets, their licences, and every change made to them |

The method specifications the implementation is checked against — cited by
section number throughout the source and tests as `MODE_MANAGER_SPEC`,
`OPTICAL_PROPAGATION_SPEC`, `EVALUATION_METRICS_SPEC` and `COMPARATOR_SPEC` —
and the pre-registered protocol are available from the corresponding author on
request.

---

## Quick start

Requires ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04.

```bash
git clone https://github.com/<you>/AUV-Simulator.git ~/auv-simulator
```

Every command below has been run from a clean shell. Copy the
block, do not skip the `source` lines: without them `ros2` cannot find the
package and the launch fails with a bare "package not found".

```bash
# 1. Build into an isolated overlay. This does not touch anything else in the
#    workspace, and --symlink-install means later Python edits need no rebuild.
cd ~/auv-simulator
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --build-base .build --install-base .install --symlink-install
source .install/setup.bash

# 2. Tests. No ROS, no Gazebo, about three minutes.
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 -m pytest test/ -q          # 176 tests
cd ../..
```

> **Use a private ROS domain** if anything else on your network publishes on
> `/uuv/*`. ROS 2 discovers peers across the whole subnet by default, so two
> people running this at once will drive each other's vehicles. Export an unused
> `ROS_DOMAIN_ID` before launching:
>
> ```bash
> export ROS_DOMAIN_ID=42
> ```

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

Keys combine: hold `w` and `←` together and it climbs while turning. Moving the
turbidity or current sliders, or pressing any fault button, **pauses the scenario
schedule** so your setting is not overwritten — the panel says who is driving the
water. `load scenario` hands it back.

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
  not rebuilt after a mesh was added. Re-run step 1.
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
the paper's own subject invisible.

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

**Known limits**, stated because they are easy to mistake for bugs:

* Gazebo's ogre2 engine **ignores `<fog>`**. The world declares it and it has
  never rendered. Depth falloff comes from the background colour instead, and
  the genuinely underwater view is the vehicle's camera, which the propagation
  model degrades properly.
* The **rendered vehicle does not translate vertically** — the velocity-control
  plugin does not drive that axis. The simulated vehicle does, so altitude,
  slant range and every decision depending on them respond correctly.
* Weed **bends from its base**, whole-plant. Moving only the fronds needs a
  rigged mesh or a vertex shader, and the model has neither.
* The **sea surface does not animate**. Waves are geometry, not motion.

Third-party models and textures are listed in [NOTICE](NOTICE).

## Running the campaign

Every number in the paper comes from here, and none of it uses Gazebo.

```bash
cd src/uuv_mode_aware_navigation

# Verify the source is unmodified before spending hours on it.
# Silence means nothing changed; any output names the file that did.
sha256sum -c results/PRE_CAMPAIGN_BASELINE.sha256 | grep -v ': OK$'

# 150 scenarios x 108 static configurations, then the comparator campaign.
# ~5 hours on four physical cores; progress every 500 runs with an ETA.
PYTHONPATH=. python3 -u scripts/run_campaign.py --seeds 10 --jobs 7 \
    --out results/campaign.csv > results/campaign.log 2>&1 &

tail -f results/campaign.log
```

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

Simulation results are easy to publish and hard to trust. Three things here are
meant to make this set checkable rather than merely readable.

**The outcome measure was fixed before any result existed.** A pre-registered
protocol declares the metrics, the seeds and the falsification conditions up
front, and every campaign is reported against them. One consequence worth
knowing: the best fixed configuration turns out not to be stable under the
choice of aggregation statistic, and a robust statistic would have moved the
headline comparison in the proposed method's favour. The pre-registered mean is
reported anyway.

Source files carry `PROTOCOL.md` section references where a rule comes from the
protocol. The document itself is available from the authors on request.

**The baseline is not ours to choose.** The fixed policy it is compared against
is selected by exhaustive sweep of the manager's own action space, and the sweep
ships with the repository, best to worst. If you think the comparison was
against a weak configuration, you can open the file, find the one you would have
picked, and read off its score.

**The comparators bracket the method rather than compete with it.** The tuned
static baseline knows before departure which configuration suits conditions it
has never met, and the oracle is handed the true water state; neither could be
deployed on a real vehicle, which is the point of using them. They put a floor
and a ceiling around the result, and the tier ablations show which part of the
method carries the effect: restricted to measurement admission alone it loses a
factor of 33 on the aggregate outcome, and a factor of 254 in an area with no
acoustic infrastructure and no prior map.

**Every number is traceable.** `PUBLICATION_ARTEFACT_MANIFEST.md` maps each
reported result to the artefact, the seed block and the command that produced it,
with a SHA-256 for every file. Held-out execution is gated on the freeze record
and the block was executed once. Nothing was retuned after seeing an answer, and
the corrections that moved results against the proposed method are reported in
the paper rather than left out of it.

## Held-out data

Seed root 20,400,000 is reserved and unspent. Held-out execution is gated on a
verified freeze record and is not reachable from the campaign script:

```bash
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 scripts/freeze.py --status   # what is covered, and whether spent
PYTHONPATH=. python3 scripts/freeze.py --verify   # does the tree still match the record
```

---

## Third-party assets

Vehicle mesh, scenery models and photogrammetry textures come from outside this
project and carry their own licences, recorded in full in [NOTICE](NOTICE) along
with every change made to them and why. In short: the BlueROV2 mesh is
Apache-2.0, two scenery models are CC0, and four are Sketchfab Standard, which
permits commercial use and derivative works without attribution. Attribution is
given anyway.

## License

MIT. See [LICENSE](src/uuv_mode_aware_navigation/LICENSE).
