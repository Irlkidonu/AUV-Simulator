# Frame conventions — `uuv_sim_physics`

Established at M2, before anything consumes sensor data, so that inconsistent
axes are a decision rather than a discovery.

## Convention

**REP-103 / SDF: right-handed, X forward, Y left, Z up.** Rotations are
right-handed about each axis; SDF poses are `x y z roll pitch yaw` with
roll/pitch/yaw applied **extrinsically in that order** (fixed-axis XYZ).

Depth is **negative z**. The free surface is not modelled; there is no z = 0
water plane. Seabed at z = −20 m, dock at z = −15 m. "Deeper" is more negative,
so a depth *reading* and a z *coordinate* have opposite signs — a sign error
here would be invisible until closed-loop control, which is why it is written
down now.

## Frame tree

```
world  (ENU-like, right-handed, Z up; origin at dock centreline, z = 0)
│
├── seabed                       (0, 0, −20)          static
├── rock_a … rock_e              various              static
│
├── docking_station              (0, 0, −15)          static
│   ├── dock_origin              ≡ model origin, the collar plane
│   ├── dock_throat              (0, 0, 0) rel. dock_origin      capture point
│   ├── dock_mouth               (+1.05, 0, 0)                   funnel outer rim
│   ├── led_constellation        (+0.05, 0, 0)                   optical target
│   └── approach_axis            direction (+1, 0, 0)
│
└── bluerov2_phys                spawn (4, 0, −15, 0, 0, π)
    └── base_link                vehicle body frame
        ├── centre of mass       (0, 0, −0.03)   rel. base_link
        ├── centre of buoyancy   (0, 0,  0.00)   collision-box centroid
        ├── camera               (0.23, 0, −0.01, 0, 0.05, 0)
        ├── prop_left            (−0.20, +0.16, 0)   axis +X
        ├── prop_right           (−0.20, −0.16, 0)   axis +X
        ├── prop_sway            (0, 0, −0.02)       axis +Y
        └── prop_vert            (0, 0, +0.10)       axis +Z
```

## Vehicle body frame

`base_link`, standard marine convention on a right-handed frame:

| Axis | Direction | Motion | Rotation |
|---|---|---|---|
| **+X** | forward (bow) | surge | roll (φ, p) |
| **+Y** | **left (port)** | sway | pitch (θ, q) |
| **+Z** | up | heave | yaw (ψ, r) |

Note **+Y is port**, not starboard. Classical Fossen texts use a NED body frame
with +Y to starboard and +Z down; SDF and REP-103 do not. The hydrodynamic
coefficients were transcribed from the reference unchanged, and every diagonal
term is sign-symmetric, so the transcription is unaffected — but any *new*
cross-coupling term added later must be derived in this frame, not copied from
a NED reference.

## Approach geometry

The vehicle spawns at (4, 0, −15) with **yaw = π**, so its body +X points along
world −X: the camera boresight faces the dock, and closing the range means
travelling in world −X while commanding **positive surge**.

Range to the dock at spawn is 4.0 m along the approach axis; the funnel mouth is
at world x = +1.05, so 2.95 m of free water precedes the capture cone.

## Sensor frames

| Sensor | Parent | Pose (x y z r p y) | Notes |
|---|---|---|---|
| camera | `base_link` | `0.23 0 −0.01 0 0.05 0` | +X boresight, pitched 0.05 rad (2.9°) **down** |
| headlight_l/r | `base_link` | `0.24 ±0.08 0` | spot, direction +X, 7 m range |
| FLS / gpu_lidar | — | — | **not on this vehicle** — see below |
| DVL | — | — | **not on this vehicle** — see below |
| IMU | — | — | **not on this vehicle** — see below |

A camera pitch of +0.05 rad in a right-handed frame with +Y to port is a
**downward** tilt. Worth stating explicitly, since the sign is easy to invert
and the error would surface as a constant bearing bias in perception.

### FLS, DVL and IMU are on the *other* reference vehicle

The reference physics world `underwater_docking_physics.sdf` gives
`bluerov2_phys` a camera and nothing else. The FLS (`gpu_lidar`), DVL
(`gz-sim-dvl-system`) and IMU live on `bluerov2` in the **kinematic** world
`underwater_docking.sdf`, which has no hydrodynamics, no thrusters and no
rigid-body actuation.

So the two capabilities the physics mode ultimately needs — real dynamics and
the full sensor suite — are split across two different vehicles in two
different worlds. M2 reproduces the physics vehicle faithfully, which means
reproducing its single camera. Merging the sensor suite onto the dynamic
vehicle is a **model change**, not a reproduction, and is therefore deferred to
M4, where each sensor is validated as it is added.

Recorded as `known_discrepancies.sensor_suite_split` in the M2 report.

## Docking reference frames

| Frame | Rel. `dock_origin` | Definition |
|---|---|---|
| `dock_throat` | `(0, 0, 0)` | capture point; the collar plane |
| `dock_mouth` | `(+1.05, 0, 0)` | funnel outer rim |
| `led_constellation` | `(+0.05, 0, 0)` | centroid of the four LEDs |
| approach axis | `(+1, 0, 0)` | vehicle approaches from +X, travelling −X |

The LED constellation is a 0.40 m square in the dock's YZ plane at x = +0.05,
symmetric about the axis. **That symmetry is exact**, so the four points alone
do not resolve roll — a property to keep in mind when the constellation is used
as an optical target.

## Handedness check

| Check | Expected | Verified at |
|---|---|---|
| Frame handedness | right-handed, X×Y = Z | M2 (declared) |
| Gravity | `(0, 0, −9.8)`, i.e. −Z | M2 (explicit in world) |
| Buoyancy | +Z, opposing gravity | M2.5 P1, P3 |
| CoB above CoM | +0.03 m ⇒ righting moment | M2.5 P9 |
| Positive surge ⇒ body +X | thrust along +X | M2.5 P5 |
| Positive sway ⇒ body +Y (port) | thrust along +Y | M2.5 P7 |
| Positive heave ⇒ body +Z (up) | thrust along +Z | M2.5 P6 |
| Positive yaw ⇒ bow to port | right-handed about +Z | M2.5 P8 |
| Camera pitch +0.05 rad ⇒ down | boresight below horizon | M4 |

The right-hand column is the point: M2 *declares* the convention; M2.5 and M4
*measure* whether the model obeys it.
