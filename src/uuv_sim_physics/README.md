# uuv_sim_physics

The physics-capable execution path for the AUV Simulator. **Additive**: it sits
beside `uuv_mode_aware_navigation` and imports from it. Nothing in that package
was changed to make this one work, and nothing in it imports this one.

```
uuv_sim_physics  ──imports──▶  uuv_mode_aware_navigation
                                (never the reverse)
```

## Two backends, one contract

| Backend | Integrates motion via | Status |
|---|---|---|
| `ReducedBackend` | the existing `mission.Vehicle` | M1 — complete |
| `GazeboBackend` | Gazebo / DART rigid-body physics | M3 — not started |

Both answer to `DynamicsBackend` in [`backend.py`](uuv_sim_physics/backend.py):
`position`, `velocity`, `current`, `path_length_m`, `step()`, `reset()`.

`ReducedBackend` **delegates**; it does not reimplement. Every arithmetic
operation still happens inside the frozen `Vehicle.step`, which is why the
equivalence can be proved bit-for-bit rather than argued.

## Import discipline

Importing this package must stay free of ROS and Gazebo, because the reduced
path has to remain usable in the headless install:

```python
from uuv_sim_physics import DynamicsBackend, ReducedBackend   # always safe
from uuv_sim_physics.gazebo_backend import GazeboBackend      # M3; needs Gazebo
```

The Gazebo backend must **never** be re-exported from `__init__.py`. Doing so
would give the headless install a back-door dependency on Gazebo at exactly the
moment nobody is looking for one. `test_dependency_isolation.py` enforces this.

## Two subsystems, kept apart

| Subsystem | What it is | Where it runs |
|---|---|---|
| **A — vehicle & environment physics** | rigid body, buoyancy, hydrodynamics, thrusters, contact | Gazebo / DART |
| **B — underwater optical image formation** | attenuation, backscatter, veiling | analytic, applied after rendering |

B reuses `optics.py` and `imaging.py` unchanged. It is **not** part of the
physics solver and must not be described as such.

## Tests

```bash
cd projects/auv-simulator
export PYTHONPATH=src/uuv_mode_aware_navigation:src/uuv_sim_physics:src/uuv_sim_physics/test
python3 -m pytest src/uuv_sim_physics/test -q
```

The existing simulator's own suite is unaffected and still runs exactly as
documented — `pytest` from the repository root collects 437 tests, none of them
from here.

| File | Proves |
|---|---|
| `test_reduced_equivalence.py` | wrapper ≡ bare `Vehicle`, 20 seeds × 10⁴ steps, bitwise |
| `test_equivalence_coverage.py` | the comparison above can actually fail (1-ULP mutations) |
| `test_dependency_isolation.py` | no reverse coupling; no ROS/Gazebo in the import graph |

## Milestones

M0 baseline ✔ · **M1 backend abstraction ✔** · M2 world/vehicle assembly ·
M2.5 quantitative physics validation · M3 backend integration ·
M4 sensors + water column.
