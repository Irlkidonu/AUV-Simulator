# Mode-Aware Adaptive Navigation for Underwater Vehicles

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Simulator-Gazebo%20Harmonic-orange)
![Physics: DART](https://img.shields.io/badge/Physics-DART-lightgrey)
![Tests](https://img.shields.io/badge/tests-173%20passing-brightgreen)

A **flyable underwater simulation environment** in which perceptual failure is
something you can cause on purpose and watch a vehicle reason its way through.

Software and data for the study *Mode-Aware Adaptive Navigation for Unmanned
Underwater Vehicles Using Multi-Modal Sensing and Optical Feedback in a
Simulation Environment*.

---

## Fly it

Three commands from a clean checkout. Copy and paste the block:

```bash
cd uuv-mode-aware-navigation
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --build-base .build --install-base .install --symlink-install
source .install/setup.bash
ros2 launch uuv_mode_aware_navigation playground.launch.py
```

That opens everything at once: the simulator, the vehicle's camera feed, and a
graphical control panel. Closing Gazebo closes the rest. Pick a failure to fly
into by naming it:

```bash
ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E7
```

**Click `TAKE MANUAL CONTROL` and fly.** Arrow keys steer, `w` and `s` change
depth, `i`/`k` pitch, `z`/`x` roll, and they combine: hold `w` and `Left`
together and the vehicle climbs while turning. The slider sets thruster surge
from nothing to 1 m/s. Press the button again to hand the vehicle back to the
manager mid-manoeuvre and watch what it does with what you left it.

Everything else on the panel is there to be broken: buttons to fail the Doppler
log, the acoustic fix, the optical channel, the surface vessel or the prior map;
a turbidity slider; a channel selector; and all nineteen scenario families in a
dropdown. The right-hand column shows what the vehicle believes while you do it
--- its capability mode, why it chose that mode, optical quality estimated from
the camera image alone, covariance, and, separately, the truth.

If you prefer the terminal, the same controls exist as a keyboard client and the
same telemetry as a text display:

```bash
ros2 run uuv_mode_aware_navigation teleop           # keyboard
ros2 launch uuv_mode_aware_navigation playground.launch.py hud:=true
```

Surge runs along the vehicle's nose, so pitching down and driving forward
descends without touching the heave axis.

The point is not the flying. It is that **the estimator, the sensor models and
the mode manager keep running and keep reporting while you fly**, so you can
drive into turbid water holding the coaxial camera the manager would have
abandoned, and watch the covariance grow.

### Pick a failure

Any of the campaign's nineteen scenario families can be replayed live:

```bash
ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E19
ros2 topic pub --once /uuv/set_scenario std_msgs/String "{data: E12}"
```

| | | |
|---|---|---|
| `E1` nominal | `E8` turbid + DVL loss | `E15` turbid and noisy |
| `E2` short DVL loss | `E9` current unobservable | `E16` featureless plain |
| `E3` long DVL loss | `E10` steady current | `E17` terrain recoverable |
| `E4` graded turbidity | `E11` building current | `E18` vessel departs |
| `E5` optical loss | `E12` rotating current | `E19` unprepared area |
| `E6` acoustic intermittent | `E13` acoustic noise | |
| `E7` compound | `E14` noisy + DVL loss | |

These are not re-declared for the demonstrator. The scenario director imports
the campaign's own definitions, so a family cannot drift between the table in
the paper and the thing you fly.

### What you are looking at

The camera window is the vehicle's own view **after** the propagation model has
degraded it — the image the optical feedback estimator reads, not a clean
render. Three channels are available, and they fail at different water
clarities, which is the whole argument in miniature:

| Channel | Lamp baseline | Usable to |
|---|---|---|
| `1` coaxial camera | 0.02 m | τ ≈ 1.5 |
| `2` off-axis camera | 0.35 m | τ ≈ 3.0 |
| `3` laser | 0.20 m | τ ≈ 5.5 |

Off-axis lighting is not a separate switch: it *is* the difference between a
lamp 0.02 m from the lens, which scatters the illuminated water column straight
back into it, and one on a 0.35 m baseline, which moves the brightest scattering
volume out of the field of view.

### Honest limits of the demonstrator

Worth knowing before you draw conclusions from a session:

- **The rendered model does not move vertically.** Gazebo's velocity-control
  plugin does not drive that axis (measured; see the note in
  `worlds/mode_aware_survey.sdf`). The *simulated* vehicle does move — altitude,
  slant range, optical availability and the manager's decisions all respond to
  heave correctly. Only the on-screen model holds its depth.
- **The sensor models are functions of altitude, water state and configuration,
  not of attitude.** Tilting does not by itself change what the Doppler log or
  the camera can do. Flying nose-down until the altitude closes does.
- **A session is a demonstration, not a measurement.** Nothing launched here
  writes a result file, and the campaign does not go through Gazebo at all, so
  flying cannot affect a reported number.

---

## The idea in one screen

An underwater vehicle cannot see its own position. Every source of absolute
aiding it has is conditional — and, crucially, **each is conditional on
something different**:

| Technique | Requires | Stops working when |
|---|---|---|
| Ultra-short baseline (USBL) | A transceiver held at the surface | The support vessel leaves station |
| Long baseline (LBL) | Four surveyed seabed transponders | The array was never laid, or the geometry degrades |
| Single-beacon ranging | One seabed transponder | Same — and it constrains range only, not position |
| Terrain-relative navigation | A prior bathymetric map, and relief to match against | The seabed is flat and featureless |
| Optical (camera / laser) | Water clear enough to see through | Turbidity rises past the channel's limit |

The usual response to degraded sensing is to adjust how much the *estimator*
trusts a measurement. That cannot change whether the measurement exists.

This package treats perceptual capability as a **resource the vehicle manages**.
It infers which capability state it is in from onboard observables alone,
predicts whether a configuration it is *not* currently flying would produce a fix
its filter would accept, and acts across three tiers — what it admits, where it
flies, and whether it continues the mission at all.

Because the techniques fail for unrelated reasons at unrelated times, a vehicle
that can work out *which* one has stopped answering is more capable than one
carrying the best of them. That is what this software is for.

## What is here

```
src/uuv_mode_aware_navigation/   ROS 2 package: physics, estimator, mode
                                 manager, comparators, campaign runner,
                                 Gazebo world, launch files, tests
  nodes/teleop_node.py           keyboard flight and fault injection
  nodes/scenario_node.py         replays a campaign scenario family live
  nodes/status_display.py        the terminal HUD
  launch/playground.launch.py    the interactive environment
  launch/demo.launch.py          the hands-off demonstration
protocol/                        the pre-registered protocol and the method
                                 specifications, as fixed before the campaigns
results/                         campaign outputs and the freeze record
analysis/                        scripts that turn a campaign CSV into tables
```

## Quick start

No ROS required for the physics, the campaign or the tests:

```bash
pip install numpy pytest matplotlib
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 -m pytest test -q
```

Then build, for anything involving Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --build-base .build --install-base .install --symlink-install
source .install/setup.bash
```

Two entry points, which differ only in who is steering:

```bash
# hands-off: the manager flies, closing the loop on a rendered camera frame
ros2 launch uuv_mode_aware_navigation demo.launch.py

# hands-on: a scenario is replayed and you can take the controls
ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E7
ros2 run uuv_mode_aware_navigation teleop     # second terminal
```

Change the water from outside either one:

```bash
ros2 param set /vehicle turbidity_c 1.6
ros2 topic pub --once /uuv/set_turbidity std_msgs/Float32 "{data: 1.6}"
```

A development campaign — roughly nine hours on eight cores:

```bash
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 -u scripts/run_campaign.py --seeds 10 --jobs 7 \
    --out results/dev.csv
```

## Command reference

Everything the repository can do, in the order you would normally do it.

### 1. Build

Only needed for anything involving ROS or Gazebo. The physics, the campaign and
the tests run without it.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --build-base .build --install-base .install --symlink-install
source .install/setup.bash
```

`--symlink-install` means edits to Python sources take effect without rebuilding.

### 2. Tests

```bash
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 -m pytest test -q          # whole suite
PYTHONPATH=. python3 -m pytest test -q -k optics    # one area
PYTHONPATH=. python3 -m pytest test -x -q        # stop at first failure
```

### 3. Launch the environment

```bash
# hands-off demonstration
ros2 launch uuv_mode_aware_navigation demo.launch.py
ros2 launch uuv_mode_aware_navigation demo.launch.py turbidity_c:=1.6
ros2 launch uuv_mode_aware_navigation demo.launch.py headless:=true

# interactive environment
ros2 launch uuv_mode_aware_navigation playground.launch.py
ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E7
ros2 launch uuv_mode_aware_navigation playground.launch.py scenario:=E19 turbidity_c:=1.4
ros2 launch uuv_mode_aware_navigation playground.launch.py camera_view:=false hud:=false
```

| Launch file | Argument | Default | Meaning |
|---|---|---|---|
| both | `turbidity_c` | `0.2` | starting beam attenuation, m⁻¹ |
| `demo` | `headless` | `false` | run Gazebo without its GUI |
| `playground` | `scenario` | `E1_nominal` | family to replay; `E7` also accepted |
| `playground` | `camera_view` | `true` | open the degraded camera feed |
| `playground` | `hud` | `true` | run the terminal status display |

### 4. Take the controls

In a second terminal:

```bash
ros2 run uuv_mode_aware_navigation teleop
```

| Key | Action | Key | Action |
|---|---|---|---|
| `↑` `↓` | surge fore / aft | `1` `2` `3` | optical channel |
| `←` `→` | yaw | `l` | off-axis lighting |
| `w` `s` | ascend / descend | `t` `T` | turbidity − / + |
| `i` `k` | pitch up / down | `d` | Doppler log fail / restore |
| `z` `x` | roll left / right | `a` | acoustic fix fail / restore |
| `space` | all stop | `o` | optical blackout fail / restore |
| `e` | manual ⇄ manager | `v` | surface vessel here / gone |
| `r` | reset to first waypoint | `m` | prior map have / none |
| `q` | quit | | |

### 5. Drive it from the command line instead

Every keyboard action is a topic, so a session can be scripted.

```bash
# who is flying
ros2 topic pub --once /uuv/control_mode std_msgs/String "{data: manual}"
ros2 topic pub --once /uuv/control_mode std_msgs/String "{data: auto}"

# fly: linear.x surge, linear.z heave; angular x/y/z roll/pitch/yaw rates
ros2 topic pub --rate 10 /uuv/teleop_cmd geometry_msgs/Twist \
    "{linear: {x: 0.4, z: 0.0}, angular: {y: -0.3, z: 0.2}}"

# force an optical channel, overriding the manager
ros2 topic pub --once /uuv/force_channel std_msgs/String "{data: camera_coaxial}"
ros2 topic pub --once /uuv/force_channel std_msgs/String "{data: camera_offaxis}"
ros2 topic pub --once /uuv/force_channel std_msgs/String "{data: lidar}"
ros2 topic pub --once /uuv/force_channel std_msgs/String "{data: ''}"   # hand it back

# water
ros2 topic pub --once /uuv/set_turbidity std_msgs/Float32 "{data: 1.6}"

# break something, then mend it
ros2 topic pub --once /uuv/inject_fault std_msgs/String "{data: 'dvl:on'}"
ros2 topic pub --once /uuv/inject_fault std_msgs/String "{data: 'dvl:off'}"

# switch scenario mid-run, and start over
ros2 topic pub --once /uuv/set_scenario std_msgs/String "{data: E12}"
ros2 topic pub --once /uuv/reset std_msgs/Bool "{data: true}"
```

Fault names: `dvl`, `acoustic`, `optical`, `vessel_gone`, `no_map`. `dvl` breaks
both Doppler modes together, because asking for "no DVL" means no velocity
aiding of any kind. `no_map` is not an instrument fault — it removes the prior
bathymetry that terrain-relative navigation matches against.

### 6. Watch what it is doing

```bash
ros2 topic echo /uuv/nav_mode                  # capability mode
ros2 topic echo /uuv/decision_reason           # why the manager chose it
ros2 topic echo /uuv/optical_quality           # estimated from the image alone
ros2 topic echo /uuv/position_covariance_trace # what the filter believes
ros2 topic echo /uuv/position_error            # truth; no decision reads this
ros2 topic echo /uuv/attitude_rpy              # roll, pitch, yaw in radians
ros2 topic echo /uuv/scenario_info             # active family and elapsed time
ros2 topic list | grep uuv                     # everything else
```

### 7. Run the nodes individually

Useful when working on one piece. Each is a console script:

```bash
ros2 run uuv_mode_aware_navigation vehicle
ros2 run uuv_mode_aware_navigation mode_manager
ros2 run uuv_mode_aware_navigation water_column
ros2 run uuv_mode_aware_navigation optical_feedback
ros2 run uuv_mode_aware_navigation status_display
ros2 run uuv_mode_aware_navigation teleop
ros2 run uuv_mode_aware_navigation scenario_director --ros-args -p scenario:=E7
```

The vehicle node integrates its own truth, so it runs and responds to teleop and
faults with no Gazebo present — which is the quickest way to check a change.

### 8. Campaigns

```bash
cd src/uuv_mode_aware_navigation

# development campaign; roughly nine hours on eight cores
PYTHONPATH=. python3 -u scripts/run_campaign.py --seeds 10 --jobs 7 \
    --out results/dev.csv

# the configuration sweep alone, without re-running the comparators
PYTHONPATH=. python3 -u scripts/run_campaign.py --sweep-only --jobs 7

# skip the sweep and use a baseline settled beforehand (PROTOCOL S2.5)
PYTHONPATH=. python3 -u scripts/run_campaign.py --fixed-config <name> --jobs 7

# the held-out block: refuses without a verified freeze record, and marks the
# record spent on success so a second execution is refused
PYTHONPATH=. python3 -u scripts/run_campaign.py --held-out --jobs 7 \
    --out results/held_out.csv
```

| Flag | Meaning |
|---|---|
| `--seeds N` | seeds per scenario family |
| `--jobs N` | worker processes; results identical to a serial run |
| `--out PATH` | output CSV |
| `--root N` | seed root; refuses reserved held-out roots |
| `--reuse-sweep` | reuse an existing sweep for the same scenarios |
| `--analytic-quality` | analytic index instead of a rendered frame; development only |
| `--fixed-config NAME` | use a named baseline, skipping the sweep |
| `--sweep-only` | run the sweep and stop |
| `--held-out` | execute the held-out block |

The static sweep over held-out scenarios is a separate tool, deliberately
outside the freeze gate because a static sweep flies no adaptive policy and so
cannot retune anything about the method:

```bash
python3 experiments/heldout_sweep.py --jobs 6
```

Long runs are best started detached and at low priority, so an interactive
session keeps its share of the machine:

```bash
nohup nice -n 5 python3 -u experiments/heldout_sweep.py --jobs 6 \
    > results/sweep.log 2>&1 &
tail -f results/sweep.log
```

Note that `nice` makes the work invisible to CPU monitors that report only
`user` time. `ps -eo pcpu,args` or the `nice` row of `top` shows the truth.

### 9. Freeze records

```bash
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 scripts/freeze.py --status    # human summary
PYTHONPATH=. python3 scripts/freeze.py --verify    # does the tree still match?
PYTHONPATH=. python3 scripts/freeze.py --write     # record the current tree
```

### 10. Analysis and export

```bash
python3 analysis/analyse_campaign.py \
    --campaign results/held_out_2.csv \
    --sweep results/static_sweep_held_out.csv \
    --normalisers results/DEVELOPMENT_NORMALISERS.json

# when no sweep exists for that block, suppress the oracle-relative numbers
python3 analysis/analyse_campaign.py --campaign results/held_out_2.csv \
    --no-hindsight-oracle

# rebuild the public repository from the working tree
python3 experiments/export_release.py --out ../uuv-mode-aware-navigation
python3 experiments/export_release.py --verify-only
```

## Reproducing the paper's numbers

Every reported figure comes from a campaign CSV in `results/`, and every campaign
CSV was produced by a source tree whose SHA-256 digests are recorded in
`results/freeze_record.json`. To check that the code here is the code that
produced them:

```bash
cd src/uuv_mode_aware_navigation
PYTHONPATH=. python3 scripts/freeze.py --verify
```

To regenerate the paper's tables from a campaign:

```bash
python3 analysis/analyse_campaign.py \
    --campaign results/held_out_2.csv \
    --sweep results/static_sweep_held_out.csv \
    --normalisers results/DEVELOPMENT_NORMALISERS.json
```

### Held-out data

Seeds are drawn from disjoint blocks. Development seeds are used for everything
— tuning, inspection, debugging, figures. Held-out blocks are executed once,
after the source is frozen, and are then closed permanently.

That is enforced in code rather than by convention: `run_campaign.py` refuses to
reach a reserved seed root through its ordinary `--root` argument, and
`--held-out` refuses unless a freeze record exists, still matches the tree, and
records the block as unspent. A spent block that could be re-entered would be a
development block, and every number ever drawn from it would have to be reported
as one.

## Design commitments

Enforced by tests, not by documentation.

**No privileged information.** Water state, fault schedules and true pose are
consumed by the physics and the evaluator, and never reach the estimator, the
mode manager or the controller. Tests assert the boundary by inspecting the
runner's own source.

**No crippled comparators.** Every policy shares sensor realisations, estimator,
physics, controller, initialisation and tuning budget. Only the oracle receives
privileged information, and it is labelled as such wherever it appears.

**No prescribed error terms.** Noise, bias and availability are derived from
propagation geometry and water optical properties rather than hand-tuned.

**Determinism.** Same seed, same output, bit for bit. The global RNG is never
touched.

## Third-party assets

The vehicle mesh and the scenery models come from outside this project and keep
their own licences. [NOTICE](NOTICE) lists every one with its source, its
licence, and the exact changes it needed to load in Gazebo.

**The downloaded scenery models are not in this repository.** They are about
191 MB of glTF, which git handles badly. The world loads without them: the
generated scenery, the seabed, the vehicle and every behaviour still work, and
you simply get a quieter reef. To restore the full scene, fetch the models named
in `NOTICE` into `src/uuv_mode_aware_navigation/models/external/<name>/`, then
apply the two fixes `NOTICE` records for each — Gazebo's Assimp loader cannot
split a `metallicRoughness` map, and nested texture folders are not installed by
colcon.

Everything under `models/meshes/*.obj` is generated and also not tracked. Three
scripts rebuild it deterministically from fixed seeds:

```bash
cd src/uuv_mode_aware_navigation
python3 scripts/make_seabed.py     # seabed relief, from uuv_mode_aware_navigation.seabed
python3 scripts/make_rocks.py      # eight irregular rocks
python3 scripts/make_scenery.py    # coral, fans, fish, wreck, pipeline, ship
```

## Citing

See [`CITATION.cff`](CITATION.cff).

## Licence

MIT — see [`LICENSE`](LICENSE).
