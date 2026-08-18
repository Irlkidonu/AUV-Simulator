"""P1-P16: the M2.5 quantitative physics validation protocol.

Each test states an analytic prediction derived from the configuration, measures
the simulator, and reports both. A test does not "pass because the number looked
reasonable" -- it passes because the measurement matches a figure computed
independently from the declared coefficients.

Where a prediction turned out to be wrong rather than the model, both the
original and the corrected prediction are recorded. Thresholds are never
loosened to accommodate a result.

Every test runs against whichever configuration is loaded, so the same protocol
produces the inherited baseline and the corrected result without modification.
"""

from __future__ import annotations

import math
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .. import world_builder
from . import harness

__all__ = ["Result", "ALL_TESTS", "run_test", "run_all"]

G = 9.8


@dataclass
class Result:
    name: str
    title: str
    status: str                       # pass | fail | not_executable
    predicted: float | None = None
    measured: float | None = None
    unit: str = ""
    tolerance: str = ""
    detail: str = ""
    data: dict = field(default_factory=dict)

    def line(self) -> str:
        mark = {"pass": "PASS", "fail": "FAIL",
                "not_executable": "N/E "}[self.status]
        if self.predicted is None:
            return f"  {self.name:4s} {mark}  {self.title}"
        return (f"  {self.name:4s} {mark}  {self.title}\n"
                f"        predicted {self.predicted:.5g} {self.unit}, "
                f"measured {self.measured:.5g} {self.unit}  [{self.tolerance}]")


# --- configuration helpers ---------------------------------------------------

def config() -> dict:
    return world_builder.load_config()


def _terminal_speed(thrust_N: float, linear: float, quadratic: float) -> float:
    """Solve |quad| v^2 + |lin| v = thrust."""
    a, b = abs(quadratic), abs(linear)
    if a == 0.0:
        return thrust_N / b
    return (-b + math.sqrt(b * b + 4 * a * thrust_N)) / (2 * a)


def total_mass(cfg: dict) -> float:
    """Hull plus every propeller link.

    The propeller links carry mass but no collision geometry, so they add weight
    without displacing water. Omitting them is what made the M2 buoyancy figure
    wrong by a factor of 39.
    """
    vehicle = cfg["vehicle_bluerov2_phys"]
    return (vehicle["base_link"]["mass_kg"]
            + sum(u["mass_kg"] for u in vehicle["thrusters"]["units"]))


def displaced_mass(cfg: dict) -> float:
    box = cfg["vehicle_bluerov2_phys"]["base_link"]["collision"]["size_m"]
    return box[0] * box[1] * box[2] * cfg["physics"]["fluid"]["density_kgm3"]


def net_weight_N(cfg: dict) -> float:
    """Positive = sinks."""
    return (total_mass(cfg) - displaced_mass(cfg)) * G


def variant_world(overrides: dict, directory: Path,
                  validated: bool = False) -> Path:
    """Write a world built from the configuration with a nested override applied.

    Used only by tests that must change a coefficient to isolate its effect
    (P4, P10, P15). The variant is written to a temporary directory and never
    replaces the package's world.
    """
    original = {name: (world_builder.CONFIG_DIR / f"{name}.yaml").read_bytes()
                for name in ("physics", "vehicle_bluerov2_phys", "dock_station")}
    try:
        for name, patch in overrides.items():
            path = world_builder.CONFIG_DIR / f"{name}.yaml"
            data = yaml.safe_load(path.read_text())
            _deep_update(data, patch)
            path.write_text(yaml.safe_dump(data, sort_keys=False))
        target = directory / "variant.sdf"
        target.write_text(world_builder.build_world_sdf(validated))
        return target
    finally:
        for name, blob in original.items():
            (world_builder.CONFIG_DIR / f"{name}.yaml").write_bytes(blob)


def _deep_update(base: dict, patch: dict) -> None:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = value


def thrust(**kwargs) -> dict[str, float]:
    """surge=, sway=, heave=, yaw= in Newtons -> per-joint thrust vector.

    The surge pair is driven in common mode for surge and differential mode for
    yaw, and the entire vector is returned at once so it can be applied
    atomically.

    Yaw sign: prop_left is at +Y (port). Driving it alone was measured to give
    r = -2.11 rad/s -- bow to starboard, i.e. NEGATIVE yaw about +Z. Positive
    yaw therefore requires the starboard thruster to push harder. Getting this
    backwards is what P13 caught on the first validated run.
    """
    surge = kwargs.get("surge", 0.0) / 2.0
    yaw = kwargs.get("yaw", 0.0) / 2.0
    return {"prop_left_joint": surge - yaw,
            "prop_right_joint": surge + yaw,
            "prop_sway_joint": kwargs.get("sway", 0.0),
            "prop_vert_joint": kwargs.get("heave", 0.0)}


# --- P1-P4 hydrostatics ------------------------------------------------------

def p1_zero_input_equilibrium(cfg: dict) -> Result:
    net = net_weight_N(cfg)
    zW = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]["linear_damping"]["zW"]
    zWabsW = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]["quadratic_damping"]["zWabsW"]
    predicted = -math.copysign(_terminal_speed(abs(net), zW, zWabsW), net)

    tel = harness.run([], duration_s=14.0, settle_s=4.0)
    measured = float(tel.steady_value(tel.body_velocity(), last_s=3.0)[2])
    ok = abs(measured - predicted) <= max(0.05 * abs(predicted), 5e-4)
    return Result(
        "P1", "zero-input vertical equilibrium", "pass" if ok else "fail",
        predicted, measured, "m/s", "5% or 0.5 mm/s",
        detail=(f"net weight {net:+.4f} N (total mass {total_mass(cfg):.4f} kg, "
                f"displaced {displaced_mass(cfg):.4f} kg); "
                f"depth drift {tel.position[-1][2] - tel.position[0][2]:+.4f} m "
                f"over {tel.t[-1]:.1f} s"),
        data={"net_weight_N": net, "total_mass_kg": total_mass(cfg),
              "displaced_mass_kg": displaced_mass(cfg),
              "depth_drift_m": float(tel.position[-1][2] - tel.position[0][2])})


def p2_mass_inertia_audit(cfg: dict) -> Result:
    """Configuration, generated SDF and the world Gazebo loads must agree."""
    import xml.etree.ElementTree as ET
    vehicle = cfg["vehicle_bluerov2_phys"]
    world = (world_builder.VALIDATED_WORLD_PATH if cfg.get("_validated")
             else world_builder.WORLD_PATH)
    root = ET.fromstring(world.read_text())
    model = next(m for m in root.findall("world/model")
                 if m.get("name") == "bluerov2_phys")
    inertial = model.find("link[@name='base_link']/inertial")

    problems = []
    if float(inertial.find("mass").text) != vehicle["base_link"]["mass_kg"]:
        problems.append("hull mass")
    for key, value in vehicle["base_link"]["inertia_kgm2"].items():
        if float(inertial.find(f"inertia/{key}").text) != value:
            problems.append(f"inertia {key}")
    for unit in vehicle["thrusters"]["units"]:
        link = model.find(f"link[@name='{unit['name']}']")
        if float(link.find("inertial/mass").text) != unit["mass_kg"]:
            problems.append(f"{unit['name']} mass")
    return Result(
        "P2", "mass and inertia consistency", "pass" if not problems else "fail",
        detail=(f"config == generated SDF for hull mass, 6 inertia terms and "
                f"4 propeller masses; total system mass {total_mass(cfg):.4f} kg"
                + (f"; MISMATCH: {problems}" if problems else "")),
        data={"total_mass_kg": total_mass(cfg), "mismatches": problems})


def p3_buoyancy_audit(cfg: dict) -> Result:
    ratio = displaced_mass(cfg) / total_mass(cfg)
    ok = abs(ratio - 1.0) <= 1e-3
    return Result(
        "P3", "displaced volume vs system mass", "pass" if ok else "fail",
        1.0, ratio, "ratio", "0.1%",
        detail=(f"displaced {displaced_mass(cfg):.4f} kg vs total "
                f"{total_mass(cfg):.4f} kg -> net {net_weight_N(cfg):+.4f} N. "
                "Only base_link carries collision geometry, so the four "
                "propeller links add mass without displacement."),
        data={"displaced_mass_kg": displaced_mass(cfg),
              "total_mass_kg": total_mass(cfg),
              "net_weight_N": net_weight_N(cfg)})


def p4_reduced_density_response(cfg: dict) -> Result:
    """Lower the fluid density and the vehicle must sink faster, as predicted."""
    density = 1000.0
    box = cfg["vehicle_bluerov2_phys"]["base_link"]["collision"]["size_m"]
    displaced = box[0] * box[1] * box[2] * density
    net = (total_mass(cfg) - displaced) * G
    hydro = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]
    predicted = -_terminal_speed(abs(net), hydro["linear_damping"]["zW"],
                                 hydro["quadratic_damping"]["zWabsW"])

    with tempfile.TemporaryDirectory() as tmp:
        # Only the fluid density changes. The buoyancy system carries its own
        # copy of that density in its plugin parameters, so it is patched in the
        # emitted SDF rather than by nulling the systems list.
        world = variant_world(
            {"physics": {"fluid": {"density_kgm3": density}}}, Path(tmp),
            validated=bool(cfg.get("_validated")))
        text = world.read_text().replace(
            "<uniform_fluid_density>1025</uniform_fluid_density>",
            f"<uniform_fluid_density>{density:g}</uniform_fluid_density>")
        world.write_text(text)
        tel = harness.run([], duration_s=14.0, settle_s=4.0, world=world)
    measured = float(tel.steady_value(tel.body_velocity(), last_s=3.0)[2])
    ok = abs(measured - predicted) <= max(0.05 * abs(predicted), 5e-4)
    return Result(
        "P4", "buoyancy responds to fluid density", "pass" if ok else "fail",
        predicted, measured, "m/s", "5%",
        detail=f"fluid density 1025 -> 1000 kg/m3, net weight {net:+.3f} N",
        data={"density_kgm3": density, "net_weight_N": net})


# --- P5-P11 hydrodynamics ----------------------------------------------------

def _terminal_axis(cfg: dict, axis: int, command: dict, thrust_N: float,
                   linear_key: str, quad_key: str, duration_s: float = 26.0):
    hydro = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]
    predicted = _terminal_speed(thrust_N, hydro["linear_damping"][linear_key],
                                hydro["quadratic_damping"][quad_key])
    tel = harness.run([harness.Command(0.0, command)],
                      duration_s=duration_s, settle_s=4.0)
    measured = float(tel.steady_value(tel.body_velocity(), last_s=3.0)[axis])
    return predicted, measured, tel


def p5_surge_terminal(cfg: dict) -> Result:
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    # Astern, away from the dock: with collision geometry present, a forward run
    # of this length ends against the structure and the steady window would
    # measure a stationary vehicle. Xu|u| is symmetric in u, so this is the same
    # terminal speed.
    predicted, measured, tel = _terminal_axis(
        cfg, 0, thrust(surge=-2 * limit), 2 * limit, "xU", "xUabsU")
    ok = abs(abs(measured) - predicted) <= 0.05 * predicted
    return Result(
        "P5", "surge terminal velocity", "pass" if ok else "fail",
        predicted, measured, "m/s", "5%",
        detail=(f"{2*limit:.0f} N total; sign {'+' if measured > 0 else '-'} "
                f"body-X (see P13 for the sign convention)"),
        data={"heading_drift_deg":
              float(np.degrees(tel.euler_rpy()[-1][2] - tel.euler_rpy()[0][2]))})


def p6_heave_terminal(cfg: dict) -> Result:
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    # The vehicle's own net weight opposes (or aids) the thruster.
    net = net_weight_N(cfg)
    hydro = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]
    predicted = _terminal_speed(limit - net, hydro["linear_damping"]["zW"],
                                hydro["quadratic_damping"]["zWabsW"])
    tel = harness.run([harness.Command(0.0, thrust(heave=limit))],
                      duration_s=26.0, settle_s=4.0)
    measured = float(tel.steady_value(tel.body_velocity(), last_s=3.0)[2])
    ok = abs(measured - predicted) <= 0.05 * predicted
    return Result(
        "P6", "heave terminal velocity", "pass" if ok else "fail",
        predicted, measured, "m/s", "5%",
        detail=f"{limit:.0f} N up, less net weight {net:+.4f} N",
        data={})


def p7_sway_terminal(cfg: dict) -> Result:
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    predicted, measured, tel = _terminal_axis(
        cfg, 1, thrust(sway=limit), limit, "yV", "yVabsV")
    ok = abs(abs(measured) - predicted) <= 0.05 * predicted
    return Result(
        "P7", "sway terminal velocity", "pass" if ok else "fail",
        predicted, measured, "m/s", "5%",
        detail="linear sway damping is zero in the inherited model, so this is "
               "purely quadratic",
        data={})


def p7b_low_speed_sway(cfg: dict) -> Result:
    """The docking-relevant regime, where quadratic damping nearly vanishes."""
    hydro = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]
    yV = hydro["linear_damping"]["yV"]
    yVabsV = hydro["quadratic_damping"]["yVabsV"]
    small = 2.0                       # N, a gentle lateral correction
    predicted = _terminal_speed(small, yV, yVabsV)

    tel = harness.run([harness.Command(0.0, thrust(sway=small)),
                       harness.Command(14.0, thrust())],
                      duration_s=26.0, settle_s=4.0)
    body = tel.body_velocity()
    driven = tel.window(11.0, 14.0)
    measured = float(np.abs(driven.body_velocity()[:, 1]).mean())

    # Decay after the command is removed: time to fall to 1/e of peak.
    after = tel.window(14.0, 26.0)
    speeds = np.abs(after.body_velocity()[:, 1])
    peak = speeds[0]
    below = np.where(speeds <= peak / math.e)[0]
    decay_s = float(after.t[below[0]] - after.t[0]) if len(below) else float("inf")

    ok = abs(measured - predicted) <= max(0.10 * predicted, 5e-3)
    return Result(
        "P7b", "low-speed sway (docking regime)", "pass" if ok else "fail",
        predicted, measured, "m/s", "10%",
        detail=(f"{small:.0f} N lateral; 1/e decay after thrust removal "
                f"{decay_s:.2f} s. With yV = {yV:g} the only resistance at low "
                f"speed is quadratic, which vanishes as v -> 0."),
        data={"decay_1_over_e_s": decay_s, "thrust_N": small})


def p8_yaw_step(cfg: dict) -> Result:
    vehicle = cfg["vehicle_bluerov2_phys"]
    limit = vehicle["thrusters"]["common"]["max_thrust_cmd_N"]
    arm = abs(vehicle["thrusters"]["units"][0]["position_m"][1])
    moment = 2 * limit * arm
    hydro = vehicle["hydrodynamics"]
    predicted = _terminal_speed(moment, hydro["linear_damping"]["nR"],
                                hydro["quadratic_damping"]["nRabsR"])
    tel = harness.run([harness.Command(0.0, thrust(yaw=2 * limit))],
                      duration_s=22.0, settle_s=4.0)
    measured = float(tel.steady_value(tel.body_rates(), last_s=3.0)[2])
    ok = abs(abs(measured) - predicted) <= 0.10 * predicted
    return Result(
        "P8", "yaw rate under full differential thrust", "pass" if ok else "fail",
        predicted, measured, "rad/s", "10%",
        detail=f"moment {moment:.2f} N.m at a {arm:.2f} m lever",
        data={"moment_Nm": moment, "lever_m": arm})


def p9_roll_free_decay(cfg: dict) -> Result:
    """Righting moment from the CoB-CoM separation sets the natural period."""
    vehicle = cfg["vehicle_bluerov2_phys"]
    mass = total_mass(cfg)
    bg = vehicle["derived_buoyancy"]["bg_separation_m"]
    ixx = vehicle["base_link"]["inertia_kgm2"]["ixx"]
    added = abs(vehicle["hydrodynamics"]["added_mass"]["kDotP"])
    predicted = 2 * math.pi / math.sqrt(mass * G * bg / (ixx + added))

    # Excite roll with an impulsive asymmetric heave, then release.
    tel = harness.run([harness.Command(0.0, {"prop_vert_joint": 30.0}),
                       harness.Command(0.4, thrust())],
                      duration_s=24.0, settle_s=4.0)
    roll = tel.euler_rpy()[:, 0]
    roll = roll - roll[-1]
    # Period from zero crossings after the excitation.
    mask = tel.t > 1.0
    t, r = tel.t[mask], roll[mask]
    crossings = np.where(np.diff(np.sign(r)))[0]
    if len(crossings) >= 3:
        measured = float(2 * np.mean(np.diff(t[crossings])))
        ok = abs(measured - predicted) <= 0.10 * predicted
        detail = f"{len(crossings)} zero crossings; amplitude {np.abs(r).max():.4f} rad"
    else:
        measured, ok = float("nan"), False
        detail = (f"only {len(crossings)} zero crossings -- roll was not excited "
                  f"enough to measure a period (amplitude "
                  f"{np.abs(r).max():.2e} rad)")
    return Result(
        "P9", "roll free-decay period", "pass" if ok else "fail",
        predicted, measured, "s", "10%", detail=detail,
        data={"amplitude_rad": float(np.abs(r).max())})


def p10_added_mass_influence(cfg: dict) -> Result:
    """Zeroing added mass must raise the initial surge acceleration."""
    vehicle = cfg["vehicle_bluerov2_phys"]
    mass = total_mass(cfg)
    xDotU = abs(vehicle["hydrodynamics"]["added_mass"]["xDotU"])
    predicted = mass / (mass + xDotU)          # accel ratio, with : without

    limit = vehicle["thrusters"]["common"]["max_thrust_cmd_N"]
    command = thrust(surge=2 * limit)

    def initial_accel(world=None) -> float:
        tel = harness.run([harness.Command(0.0, command)],
                          duration_s=3.0, settle_s=4.0, world=world)
        early = tel.window(0.05, 0.55)
        speed = np.abs(early.body_velocity()[:, 0])
        return float(np.polyfit(early.t, speed, 1)[0])

    with_am = initial_accel()
    with tempfile.TemporaryDirectory() as tmp:
        world = variant_world(
            {"vehicle_bluerov2_phys": {"hydrodynamics": {"added_mass": {
                "xDotU": -0.0, "yDotV": -0.0, "zDotW": -0.0}}}}, Path(tmp),
            validated=bool(cfg.get("_validated")))
        without_am = initial_accel(world)

    measured = with_am / without_am if without_am else float("nan")
    ok = abs(measured - predicted) <= 0.05
    return Result(
        "P10", "added-mass influence on surge", "pass" if ok else "fail",
        predicted, measured, "ratio", "0.05 absolute",
        detail=(f"initial acceleration {with_am:.4f} m/s2 with added mass, "
                f"{without_am:.4f} m/s2 without"),
        data={"accel_with": with_am, "accel_without": without_am})


def p11_surge_step_shape(cfg: dict) -> Result:
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    tel = harness.run([harness.Command(0.0, thrust(surge=-2 * limit))],
                      duration_s=26.0, settle_s=4.0)
    speed = np.abs(tel.body_velocity()[:, 0])
    steady = float(speed[tel.t >= tel.t[-1] - 3.0].mean())
    overshoot = float((speed.max() - steady) / steady * 100.0)
    rise = tel.t[np.argmax(speed >= 0.9 * steady)] if (speed >= 0.9 * steady).any() else float("nan")
    ok = overshoot < 5.0
    return Result(
        "P11", "surge step response shape", "pass" if ok else "fail",
        0.0, overshoot, "% overshoot", "< 5%",
        detail=f"steady {steady:.4f} m/s, 90% rise at {rise:.2f} s",
        data={"steady_mps": steady, "rise_90_s": float(rise)})


def p12_current_disturbance(cfg: dict) -> Result:
    return Result(
        "P12", "current disturbance response", "not_executable",
        detail=("The model has no ambient current. gz-sim-hydrodynamics-system "
                "accepts no flow input and the reference world never defined "
                "one, so there is nothing to measure. Adding a current is a "
                "model extension, deferred beyond M2.5."))


# --- P13-P16 actuation, contact, numerics, frames ----------------------------

def p13_sign_map(cfg: dict) -> Result:
    """Each single-axis command must accelerate along its declared body axis."""
    limit = 40.0
    cases = {
        "+surge": (thrust(surge=2 * limit), 0, +1),
        "-surge": (thrust(surge=-2 * limit), 0, -1),
        "+sway": (thrust(sway=limit), 1, +1),
        "-sway": (thrust(sway=-limit), 1, -1),
        "+heave": (thrust(heave=limit), 2, +1),
        "-heave": (thrust(heave=-limit), 2, -1),
    }
    observed, wrong = {}, []
    for label, (command, axis, expected) in cases.items():
        tel = harness.run([harness.Command(0.0, command)],
                          duration_s=6.0, settle_s=4.0)
        value = float(tel.window(2.0, 6.0).body_velocity()[:, axis].mean())
        observed[label] = value
        if math.copysign(1.0, value) != expected:
            wrong.append(label)

    # Yaw, measured as a body rate rather than a velocity.
    for label, sign in (("+yaw", +1), ("-yaw", -1)):
        tel = harness.run([harness.Command(0.0, thrust(yaw=sign * 2 * limit))],
                          duration_s=6.0, settle_s=4.0)
        rate = float(tel.window(2.0, 6.0).body_rates()[:, 2].mean())
        observed[label] = rate
        if math.copysign(1.0, rate) != sign:
            wrong.append(label)

    return Result(
        "P13", "command sign map (6 DOF)", "pass" if not wrong else "fail",
        detail=("all axes follow their declared sign" if not wrong
                else f"inverted: {', '.join(wrong)}"),
        data={"body_response": observed, "inverted": wrong})


def p14_dock_contact(cfg: dict) -> Result:
    if not cfg["dock_station"]["collision"]["present"]:
        return Result(
            "P14", "dock contact and funnel guidance", "not_executable",
            detail=("The docking station carries no collision geometry, so "
                    "contact cannot occur and the vehicle passes through the "
                    "structure. Blocked until collision primitives are added."))
    return _p14_measure(cfg)


#: A centred low-speed approach must be able to put the vehicle's centre past
#: the collar plane. v2.0.0 emitted a 0.1587 m throat against a 0.1749 m hull
#: half-diagonal, so entry was impossible -- and P14 still passed, because every
#: criterion it checked (no tunnelling, bounded speed, finite state) remains true
#: when the vehicle simply stops against a wall. Contact being well behaved is
#: not the same as the dock being usable, and only the second property is what
#: correction C2 exists to provide.
THROAT_ENTRY_SCENARIO = "low-speed seating"
THROAT_ENTRY_MAX_X_M = 0.0


def _p14_measure(cfg: dict) -> Result:
    """Five contact scenarios at docking-representative speed."""
    limit = 18.0                       # gentle approach, not a crash test
    scenarios = {
        "frontal": thrust(surge=limit),
        "lateral offset": thrust(surge=limit, sway=4.0),
        "angular misalignment": thrust(surge=limit, yaw=3.0),
        "funnel edge": thrust(surge=limit, heave=4.0),
        "low-speed seating": thrust(surge=limit / 2),
    }
    outcomes, failures = {}, []
    dock_x = cfg["dock_station"]["pose"][0]
    for label, command in scenarios.items():
        tel = harness.run([harness.Command(0.0, command)],
                          duration_s=30.0, settle_s=4.0)
        x = tel.position[:, 0]
        speed = np.linalg.norm(tel.world_velocity(), axis=1)
        finite = bool(np.isfinite(tel.position).all() and np.isfinite(speed).all())
        # The vehicle starts at x = +4 and drives toward the dock at x = 0.
        # Passing to x < dock_x - 0.3 means it went through the structure.
        tunnelled = bool(x.min() < dock_x - 0.30)
        outcomes[label] = {"closest_x_m": float(x.min()),
                           "max_speed_mps": float(speed.max()),
                           "finite": finite, "tunnelled": tunnelled}
        if tunnelled or not finite or speed.max() > 10.0:
            failures.append(label)

    # Functional property: the dock must be enterable, not merely solid.
    entry = outcomes.get(THROAT_ENTRY_SCENARIO, {})
    entered = bool(entry.get("closest_x_m", 1e9) < THROAT_ENTRY_MAX_X_M)
    if not entered:
        failures.append(f"{THROAT_ENTRY_SCENARIO}: no throat entry")

    detail = ("5/5 scenarios: contact held, no tunnelling, bounded speeds; "
              f"throat entry reached x = {entry.get('closest_x_m', float('nan')):.4f} m"
              if not failures else f"failed: {failures}")
    return Result(
        "P14", "dock contact, funnel guidance and throat entry",
        "pass" if not failures else "fail",
        detail=detail,
        data={"scenarios": outcomes, "throat_entry": {
            "scenario": THROAT_ENTRY_SCENARIO,
            "closest_x_m": entry.get("closest_x_m"),
            "threshold_m": THROAT_ENTRY_MAX_X_M,
            "entered": entered}})


def p15_timestep_sensitivity(cfg: dict) -> Result:
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    command = thrust(surge=-2 * limit)          # astern; see P5

    def terminal(world=None) -> float:
        tel = harness.run([harness.Command(0.0, command)],
                          duration_s=24.0, settle_s=4.0, world=world)
        return float(np.abs(tel.steady_value(tel.body_velocity(), last_s=3.0)[0]))

    at_1ms = terminal()
    with tempfile.TemporaryDirectory() as tmp:
        world = variant_world(
            {"physics": {"engine": {"max_step_size_s": 0.0005}}}, Path(tmp),
            validated=bool(cfg.get("_validated")))
        at_half = terminal(world)
    measured = abs(at_half - at_1ms) / at_1ms * 100.0
    ok = measured < 1.0
    return Result(
        "P15", "timestep sensitivity (1 ms vs 0.5 ms)", "pass" if ok else "fail",
        0.0, measured, "% difference", "< 1%",
        detail=f"terminal surge {at_1ms:.4f} m/s at 1 ms, {at_half:.4f} m/s at 0.5 ms",
        data={"terminal_1ms": at_1ms, "terminal_0p5ms": at_half})


def p16_frame_audit(cfg: dict) -> Result:
    """Declared conventions must match the generated world."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(world_builder.WORLD_PATH.read_text())
    problems = []

    gravity = [float(v) for v in root.find("world/gravity").text.split()]
    if gravity != [0.0, 0.0, -9.8]:
        problems.append(f"gravity {gravity}")

    vehicle = cfg["vehicle_bluerov2_phys"]
    com_z = vehicle["base_link"]["center_of_mass_m"][2]
    cob_z = vehicle["derived_buoyancy"]["center_of_buoyancy_m"][2]
    if not cob_z > com_z:
        problems.append("centre of buoyancy is not above centre of mass")

    model = next(m for m in root.findall("world/model")
                 if m.get("name") == "bluerov2_phys")
    expected_axes = {u["joint"]: [float(a) for a in u["axis"]]
                     for u in vehicle["thrusters"]["units"]}
    for joint, axis in expected_axes.items():
        actual = [float(v) for v in
                  model.find(f"joint[@name='{joint}']/axis/xyz").text.split()]
        if actual != axis:
            problems.append(f"{joint} axis {actual} != {axis}")

    yaw = vehicle["spawn_pose"][5]
    if abs(yaw - math.pi) > 1e-3:
        problems.append(f"spawn yaw {yaw} is not pi")

    return Result(
        "P16", "frame conventions", "pass" if not problems else "fail",
        detail=("gravity -Z, CoB above CoM, 4 thruster axes and spawn yaw all "
                "match the declared convention" if not problems
                else "; ".join(problems)),
        data={"problems": problems})


ALL_TESTS = {
    "P1": p1_zero_input_equilibrium, "P2": p2_mass_inertia_audit,
    "P3": p3_buoyancy_audit, "P4": p4_reduced_density_response,
    "P5": p5_surge_terminal, "P6": p6_heave_terminal, "P7": p7_sway_terminal,
    "P7b": p7b_low_speed_sway, "P8": p8_yaw_step, "P9": p9_roll_free_decay,
    "P10": p10_added_mass_influence, "P11": p11_surge_step_shape,
    "P12": p12_current_disturbance, "P13": p13_sign_map,
    "P14": p14_dock_contact, "P15": p15_timestep_sensitivity,
    "P16": p16_frame_audit,
}


def run_test(name: str, cfg: dict | None = None) -> Result:
    return ALL_TESTS[name](cfg or config())


def run_all(names=None, cfg: dict | None = None) -> list[Result]:
    cfg = cfg or config()
    return [ALL_TESTS[name](cfg) for name in (names or ALL_TESTS)]
