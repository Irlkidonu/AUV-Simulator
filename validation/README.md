# Validation evidence

Everything here supports a claim made in the top-level README. It is the
released simulator's evidence, not the development history that produced it.

## Re-running the checks

```bash
export PYTHONPATH=src/uuv_mode_aware_navigation:src/uuv_sim_physics

# Integrity of the reduced simulator: 251 files, byte-for-byte
sha256sum -c validation/regression/reduced_simulator.sha256

# The reduced simulator is deterministic; these rows must reproduce exactly
python3 validation/regression/golden_study3.py --check

# The full existing suite, with neither ROS nor Gazebo installed
python3 -m pytest src/uuv_mode_aware_navigation/test -q          # 437 tests

# The physics package
python3 -m pytest src/uuv_sim_physics/test -q                    # 113 tests

# Physics mode (needs the pinned Gazebo stack)
python3 -m uuv_sim_physics.validation.protocol                   # P1-P16
python3 -m uuv_sim_physics.control.maneuvers                     # T1-T7
python3 -m uuv_sim_physics.validation.sensor_validation          # sensor suite
python3 -m uuv_sim_physics.privileged                            # isolation audit
```

## `regression/`

| File | What it certifies |
|---|---|
| `reduced_simulator.sha256` | 251 files of `uuv_mode_aware_navigation`, unchanged by the physics release. Paths are repo-relative, so it verifies from a clean clone. |
| `golden_study3.py` / `.json` | 48 deterministic Study-3 rows, digest `1a89cd70…`. Engineering fixture at seed root 39,000,000 — **not** scientific evidence, and never to be mixed into a results campaign. |
| `freeze_integrity.json` | Hash-only verification of the Study-3 freeze manifests (V1 43/43, V2 49/49). Use this rather than `verify_lock_v2.py`'s exit code: that script also gates held-out execution, which has been spent, so it fails permanently and correctly. |
| `environment.json` | The environment the baselines were captured in. |

## `results/`

Measured outputs, not summaries.

| File | Contents |
|---|---|
| `physics_P1_P16.json`, `physics_P10_P15_P12.json` | Hydrostatics, hydrodynamics, actuation, numerics, contact, frames |
| `timestep_convergence_*.json` | Four timesteps × four dynamic quantities |
| `added_mass_window_sweep.json` | P10 ratio against fit window; ideal 0.8001, best estimate 0.8040 |
| `dock_contact_scenarios.json` | Five contact configurations |
| `heading_repeatability.json` | Three identical open-loop runs |
| `control_T1_T7.json` | Closed-loop manoeuvres |
| `reduced_vs_physics.json` | Same mission intent through both execution modes |
| `sensor_validation.json` | 24 sensor checks: rates, signs, frames, observability, optical |
| `provenance_example.json` | The record every physics run emits |

## `figures/`

| Figure | Shows |
|---|---|
| `01_equilibrium` | Zero-input depth, REFERENCE vs VALIDATED |
| `02_step_responses` | Surge / sway / heave against analytic terminals |
| `03_yaw_and_roll` | Yaw rate; roll free decay against the analytic period |
| `04_added_mass` | Early acceleration with and without added mass |
| `05_timestep_convergence` | Four quantities across four timesteps |
| `06_contact` | Five dock-contact approaches |
| `07_heading_repeatability` | Open-loop heading, three identical runs |
| `08_execution_mode_comparison` | Reduced vs physics, one mission |
| `09_T1_T6_responses` | Closed-loop control responses |
| `10_T5_wrench_allocation` | Demanded moment and allocated thrusts against saturation |
| `11_T7_repeatability` | Closed-loop cross-track, three runs |

## What is deliberately not here

The milestone reports (M0, M1, PC-2, M2, M2.5, M3, M4), console captures and
superseded intermediate runs are development history. They are retained by the
author but not published: this repository presents the released simulator, and
the numbers above are reproducible from the code without reading a diary.

Two artefacts are also local-only for a technical reason rather than an
editorial one: the integrity baseline for `src/uuv_adaptive_nav` covers a
*different* repository that a clean clone does not contain, and the PC-2
toolchain smoke test has been superseded by `uuv_sim_physics.toolchain`, which
ships as part of the package and runs the same check.
