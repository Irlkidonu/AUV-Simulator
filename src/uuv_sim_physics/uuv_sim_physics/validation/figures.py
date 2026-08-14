"""Render the M2.5 validation figure set, and re-measure P10 cleanly.

Every panel is evidence for a specific P-test. Nothing decorative is drawn: a
figure that does not settle a question is not produced.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

from .. import world_builder             # noqa: E402
from . import harness, protocol          # noqa: E402

OUT = Path("baselines/M2.5/figures")
REF = world_builder.WORLD_PATH
VAL = world_builder.VALIDATED_WORLD_PATH

plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.autolayout": True})
C_REF, C_VAL = "#B4656F", "#3A6EA5"


def _save(fig, name: str) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {name}", flush=True)


def fig_equilibrium(cfg_ref, cfg_val):
    """P1/P3: the inherited vehicle sinks; the corrected one holds depth."""
    fig, ax = plt.subplots(figsize=(5.2, 3.0))
    for world, cfg, colour, label in ((REF, cfg_ref, C_REF, "REFERENCE"),
                                      (VAL, cfg_val, C_VAL, "VALIDATED")):
        tel = harness.run([], duration_s=14.0, settle_s=4.0, world=world)
        depth = tel.position[:, 2] - tel.position[0, 2]
        net = protocol.net_weight_N(cfg)
        ax.plot(tel.t, depth, color=colour, lw=2,
                label=f"{label}  (net {net:+.4f} N)")
    ax.set_xlabel("time (s)"); ax.set_ylabel("depth change (m)")
    ax.set_title("P1/P3  zero-input vertical equilibrium")
    ax.legend(frameon=False)
    _save(fig, "01_equilibrium.png")


def fig_steps(cfg):
    """P5/P6/P7/P11: step responses, body frame, against analytic terminals."""
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    hydro = cfg["vehicle_bluerov2_phys"]["hydrodynamics"]
    panels = [
        ("surge", 0, protocol.thrust(surge=-2 * limit),
         protocol._terminal_speed(2 * limit, hydro["linear_damping"]["xU"],
                                  hydro["quadratic_damping"]["xUabsU"])),
        ("sway", 1, protocol.thrust(sway=limit),
         protocol._terminal_speed(limit, hydro["linear_damping"]["yV"],
                                  hydro["quadratic_damping"]["yVabsV"])),
        ("heave", 2, protocol.thrust(heave=limit),
         protocol._terminal_speed(limit, hydro["linear_damping"]["zW"],
                                  hydro["quadratic_damping"]["zWabsW"])),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.0))
    for ax, (name, axis, command, predicted) in zip(axes, panels):
        tel = harness.run([harness.Command(0.0, command)],
                          duration_s=20.0, settle_s=3.0, world=VAL)
        speed = np.abs(tel.body_velocity()[:, axis])
        ax.plot(tel.t, speed, color=C_VAL, lw=1.8, label="measured")
        ax.axhline(predicted, color="0.35", ls="--", lw=1.2,
                   label=f"analytic {predicted:.3f}")
        ax.set_title(f"{name} step  ({'2x' if name=='surge' else ''}{limit:.0f} N)")
        ax.set_xlabel("time (s)"); ax.set_ylabel("body speed (m/s)")
        ax.legend(frameon=False, fontsize=8)
    fig.suptitle("P5 / P7 / P6  translational step responses (VALIDATED)", y=1.04)
    _save(fig, "02_step_responses.png")


def fig_yaw_and_roll(cfg):
    """P8 yaw rate and P9 roll free decay."""
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    vehicle = cfg["vehicle_bluerov2_phys"]
    arm = abs(vehicle["thrusters"]["units"][0]["position_m"][1])
    hydro = vehicle["hydrodynamics"]
    predicted_r = protocol._terminal_speed(
        2 * limit * arm, hydro["linear_damping"]["nR"],
        hydro["quadratic_damping"]["nRabsR"])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.4, 3.0))
    tel = harness.run([harness.Command(0.0, protocol.thrust(yaw=2 * limit))],
                      duration_s=16.0, settle_s=3.0, world=VAL)
    ax1.plot(tel.t, tel.body_rates()[:, 2], color=C_VAL, lw=1.8, label="measured")
    ax1.axhline(predicted_r, color="0.35", ls="--", lw=1.2,
                label=f"analytic {predicted_r:.3f}")
    ax1.set_title(f"P8  yaw rate ({2*limit*arm:.1f} N.m)")
    ax1.set_xlabel("time (s)"); ax1.set_ylabel("r (rad/s)")
    ax1.legend(frameon=False, fontsize=8)

    tel = harness.run([harness.Command(0.0, {"prop_vert_joint": 30.0}),
                       harness.Command(0.4, protocol.thrust())],
                      duration_s=20.0, settle_s=3.0, world=VAL)
    roll = tel.euler_rpy()[:, 0]
    ax2.plot(tel.t, np.degrees(roll - roll[-1]), color=C_VAL, lw=1.5)
    mass = protocol.total_mass(cfg)
    bg = vehicle["derived_buoyancy"]["bg_separation_m"]
    period = 2 * np.pi / np.sqrt(mass * 9.8 * bg /
                                 (vehicle["base_link"]["inertia_kgm2"]["ixx"]
                                  + abs(hydro["added_mass"]["kDotP"])))
    for k in range(1, 8):
        ax2.axvline(1.0 + k * period, color="0.75", lw=0.8, ls=":")
    ax2.set_title(f"P9  roll free decay (analytic T = {period:.3f} s)")
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("roll (deg)")
    _save(fig, "03_yaw_and_roll.png")


def fig_added_mass(cfg):
    """P10 re-measured near t=0, before drag develops."""
    mass = protocol.total_mass(cfg)
    xd = abs(cfg["vehicle_bluerov2_phys"]["hydrodynamics"]["added_mass"]["xDotU"])
    F = 100.0
    command = protocol.thrust(surge=-F)

    def series(world):
        tel = harness.run([harness.Command(0.0, command)], duration_s=1.5,
                          settle_s=3.0, world=world)
        return tel.t, np.abs(tel.body_velocity()[:, 0])

    t_with, v_with = series(VAL)
    with tempfile.TemporaryDirectory() as tmp:
        variant = protocol.variant_world(
            {"vehicle_bluerov2_phys": {"hydrodynamics": {"added_mass": {
                "xDotU": -0.0, "yDotV": -0.0, "zDotW": -0.0}}}},
            Path(tmp), validated=True)
        t_without, v_without = series(variant)

    results = {}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.6, 3.0))
    for t, v, colour, label in ((t_with, v_with, C_VAL, "with added mass"),
                                (t_without, v_without, C_REF, "added mass = 0")):
        ax1.plot(t, v, color=colour, lw=1.8, label=label)
    ax1.set_xlim(0, 0.6); ax1.set_xlabel("time (s)")
    ax1.set_ylabel("body surge speed (m/s)")
    ax1.set_title("P10  early acceleration, 100 N")
    ax1.legend(frameon=False, fontsize=8)

    # Acceleration estimated over shrinking windows: the ideal ratio is only
    # approached as the window -> 0, because drag grows with the speed reached
    # inside it, and the zero-added-mass case reaches more speed.
    windows = [0.05, 0.075, 0.10, 0.15, 0.20, 0.30]
    ratios = []
    for w in windows:
        a1 = np.polyfit(t_with[t_with <= w], v_with[t_with <= w], 1)[0]
        a0 = np.polyfit(t_without[t_without <= w], v_without[t_without <= w], 1)[0]
        ratios.append(a1 / a0)
        results[f"window_{w}s"] = {"a_with": float(a1), "a_without": float(a0),
                                   "ratio": float(a1 / a0)}
    ideal = mass / (mass + xd)
    ax2.plot(windows, ratios, "o-", color=C_VAL, lw=1.8, label="measured ratio")
    ax2.axhline(ideal, color="0.35", ls="--", lw=1.2,
                label=f"ideal m/(m+Xu̇) = {ideal:.3f}")
    ax2.set_xlabel("fit window (s)"); ax2.set_ylabel("a(with) / a(without)")
    ax2.set_title("P10  ratio vs fit window")
    ax2.legend(frameon=False, fontsize=8)
    _save(fig, "04_added_mass.png")

    results["ideal_ratio"] = float(ideal)
    Path("baselines/M2.5/p10_clean.json").write_text(
        json.dumps(results, indent=2) + "\n")
    return results


def fig_timestep():
    """P15 convergence across four timesteps."""
    merged = {}
    for name in ("timestep_2p0_1p0.json", "timestep_0p5_0p25.json"):
        path = Path("baselines/M2.5") / name
        if path.exists():
            merged.update(json.loads(path.read_text())["results"])
    if not merged:
        return None
    steps = sorted(float(k[:-2]) for k in merged)
    keys = [("surge_terminal_mps", "surge terminal (m/s)"),
            ("heave_terminal_mps", "heave terminal (m/s)"),
            ("yaw_rate_radps", "yaw rate (rad/s)"),
            ("contact_closest_x_m", "contact closest x (m)")]
    fig, axes = plt.subplots(1, 4, figsize=(12.5, 2.9))
    for ax, (key, label) in zip(axes, keys):
        values = [merged[f"{s}ms"][key] for s in steps]
        ax.plot(steps, values, "o-", color=C_VAL, lw=1.8)
        ax.set_xscale("log"); ax.set_xticks(steps)
        ax.set_xticklabels([str(s) for s in steps])
        ax.invert_xaxis()
        ax.set_xlabel("timestep (ms)"); ax.set_title(label, fontsize=9)
    fig.suptitle("P15  timestep convergence (VALIDATED)", y=1.05)
    _save(fig, "05_timestep_convergence.png")
    return merged


def fig_contact(cfg):
    """P14: five contact scenarios against the corrected dock."""
    limit = 18.0
    scenarios = {
        "frontal": protocol.thrust(surge=limit),
        "lateral offset": protocol.thrust(surge=limit, sway=4.0),
        "angular misalign": protocol.thrust(surge=limit, yaw=3.0),
        "funnel edge": protocol.thrust(surge=limit, heave=4.0),
        "low-speed seating": protocol.thrust(surge=limit / 2),
    }
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.2, 3.2))
    summary = {}
    for (label, command), colour in zip(scenarios.items(), plt.cm.viridis(
            np.linspace(0.1, 0.85, len(scenarios)))):
        tel = harness.run([harness.Command(0.0, command)],
                          duration_s=26.0, settle_s=3.0, world=VAL)
        speed = np.linalg.norm(tel.world_velocity(), axis=1)
        ax1.plot(tel.t, tel.position[:, 0], color=colour, lw=1.6, label=label)
        ax2.plot(tel.t, speed, color=colour, lw=1.4)
        summary[label] = {"closest_x_m": float(tel.position[:, 0].min()),
                          "max_speed_mps": float(speed.max())}
    for ax, y, text in ((ax1, 1.05, "funnel mouth"), (ax1, 0.0, "dock throat")):
        ax.axhline(y, color="0.4", ls="--", lw=1.0)
        ax.text(0.4, y + 0.05, text, fontsize=7, color="0.35")
    ax1.set_xlabel("time (s)"); ax1.set_ylabel("world x (m)")
    ax1.set_title("P14  approach and contact")
    ax1.legend(frameon=False, fontsize=7)
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("speed (m/s)")
    ax2.set_title("contact speeds remain bounded")
    _save(fig, "06_contact.png")
    Path("baselines/M2.5/p14_contact.json").write_text(
        json.dumps(summary, indent=2) + "\n")
    return summary


def fig_heading_repeatability(cfg):
    """C3: heading behaviour under repeated identical symmetric surge."""
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]
    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    drifts = []
    for trial in range(3):
        tel = harness.run([harness.Command(0.0, protocol.thrust(surge=-2 * limit))],
                          duration_s=14.0, settle_s=3.0, world=VAL)
        yaw = np.degrees(tel.euler_rpy()[:, 2])
        ax.plot(tel.t, yaw - yaw[0], lw=1.6, label=f"trial {trial + 1}")
        drifts.append(float(yaw[-1] - yaw[0]))
    ax.set_xlabel("time (s)"); ax.set_ylabel("heading change (deg)")
    ax.set_title("C3  repeated identical surge, atomic commands")
    ax.legend(frameon=False, fontsize=8)
    _save(fig, "07_heading_repeatability.png")
    Path("baselines/M2.5/heading_repeatability.json").write_text(
        json.dumps({"yaw_drift_deg": drifts,
                    "spread_deg": max(drifts) - min(drifts)}, indent=2) + "\n")
    return drifts


def main() -> None:
    cfg_ref = world_builder.load_config()
    cfg_val = world_builder.load_config(validated=True)
    print("rendering M2.5 figure set")
    fig_equilibrium(cfg_ref, cfg_val)
    fig_steps(cfg_val)
    fig_yaw_and_roll(cfg_val)
    p10 = fig_added_mass(cfg_val)
    fig_timestep()
    fig_contact(cfg_val)
    drifts = fig_heading_repeatability(cfg_val)
    print("\nP10 ratio by window:")
    for key, value in p10.items():
        if key.startswith("window"):
            print(f"  {key:16s} a_with={value['a_with']:.4f} "
                  f"a_without={value['a_without']:.4f} ratio={value['ratio']:.4f}")
    print(f"  ideal {p10['ideal_ratio']:.4f}")
    print(f"\nheading drift over 3 identical runs: "
          f"{['%.2f' % d for d in drifts]} deg")


if __name__ == "__main__":
    main()
