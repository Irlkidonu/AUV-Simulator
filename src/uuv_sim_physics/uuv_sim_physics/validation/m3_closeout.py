"""M3 closeout: L8 comparison, L9 figures, L7 (P4 on VALIDATED).

Re-runs the seven manoeuvres once to capture traces for plotting, since M3
recorded metrics but did not archive telemetry.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

from .. import world_builder             # noqa: E402
from ..control import maneuvers          # noqa: E402
from ..execution import PhysicsExecution, ReducedExecution   # noqa: E402
from ..gazebo_backend import GazeboBackend                   # noqa: E402
from . import protocol                   # noqa: E402

OUT = Path("baselines/M3/figures")
C_RED, C_PHY = "#B4656F", "#3A6EA5"
plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.autolayout": True})

SPAWN = (4.0, 0.0, -15.0)
TARGET = (1.6, 0.0)
DEPTH, HEADING = -15.0, math.pi


def _save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, bbox_inches="tight"); plt.close(fig)
    print(f"  wrote {name}", flush=True)


# --- L8 -----------------------------------------------------------------

def comparison() -> dict:
    """Same mission-level intent, two execution modes."""
    reduced = ReducedExecution(SPAWN).goto(TARGET, depth_m=DEPTH,
                                           heading_rad=HEADING, timeout_s=60.0)
    with GazeboBackend() as backend:
        physics = PhysicsExecution(backend).goto(TARGET, depth_m=DEPTH,
                                                 heading_rad=HEADING,
                                                 timeout_s=60.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.4, 3.4))
    for outcome, colour in ((reduced, C_RED), (physics, C_PHY)):
        ax1.plot(outcome.trajectory[:, 0], outcome.trajectory[:, 1],
                 color=colour, lw=2, label=f"{outcome.mode}")
        distance = np.linalg.norm(outcome.trajectory[:, :2]
                                  - np.array(TARGET), axis=1)
        ax2.plot(outcome.times, distance, color=colour, lw=2,
                 label=f"{outcome.mode}  ({outcome.completion_time_s:.1f} s)")
    ax1.plot(*SPAWN[:2], "ko", ms=5); ax1.plot(*TARGET, "k*", ms=11)
    ax1.set_xlabel("world x (m)"); ax1.set_ylabel("world y (m)")
    ax1.set_title("path"); ax1.legend(frameon=False); ax1.invert_xaxis()
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("distance to waypoint (m)")
    ax2.set_title("approach"); ax2.legend(frameon=False)
    fig.suptitle("L8  same mission intent, two execution modes", y=1.04)
    _save(fig, "08_execution_mode_comparison.png")

    result = {"reduced": reduced.summary(), "physics": physics.summary()}
    Path("baselines/M3/execution_comparison.json").write_text(
        json.dumps(result, indent=2) + "\n")
    return result


# --- L9 -----------------------------------------------------------------

def figures() -> dict:
    outcomes = {}
    for name in ("T1", "T2", "T3", "T4", "T5", "T6"):
        outcomes[name] = maneuvers.TESTS[name]()
        print(f"  captured {name}", flush=True)

    fig, axes = plt.subplots(2, 3, figsize=(11.5, 6.0))

    tr = outcomes["T1"].trace
    axes[0, 0].plot(tr.t, tr.position()[:, 2], color=C_PHY, lw=1.8)
    axes[0, 0].axhline(-13.5, color="0.4", ls="--", lw=1.1)
    axes[0, 0].set_title("T1  depth hold (+1.5 m)"); axes[0, 0].set_ylabel("z (m)")

    tr = outcomes["T2"].trace
    axes[0, 1].plot(tr.t, np.degrees(np.unwrap(tr.array("yaw")) - math.pi),
                    color=C_PHY, lw=1.8)
    axes[0, 1].axhline(0, color="0.4", ls="--", lw=1.1)
    axes[0, 1].set_title("T2  heading under 0.4 m/s surge\n(open loop: ~291 deg)")
    axes[0, 1].set_ylabel("heading error (deg)")

    tr = outcomes["T3"].trace
    p = tr.position()
    axes[0, 2].plot(p[:, 0], (p[:, 1] - p[0, 1]) * 1000.0, color=C_PHY, lw=1.8)
    axes[0, 2].set_title("T3  straight-line track")
    axes[0, 2].set_xlabel("world x (m)"); axes[0, 2].set_ylabel("cross-track (mm)")
    axes[0, 2].invert_xaxis()

    tr = outcomes["T4"].trace
    axes[1, 0].plot(tr.t, tr.position()[:, 1], color=C_PHY, lw=1.8)
    axes[1, 0].axhline(1.0, color="0.4", ls="--", lw=1.1)
    axes[1, 0].set_title("T4  lateral offset (1 m)"); axes[1, 0].set_ylabel("y (m)")

    tr = outcomes["T5"].trace
    axes[1, 1].plot(tr.t, np.degrees(np.unwrap(tr.array("yaw"))), color=C_PHY, lw=1.8)
    axes[1, 1].axhline(math.degrees(math.pi / 2), color="0.4", ls="--", lw=1.1)
    axes[1, 1].set_title("T5  yaw slew (-90 deg)"); axes[1, 1].set_ylabel("yaw (deg)")

    tr = outcomes["T6"].trace
    p = tr.position()
    axes[1, 2].plot(p[:, 0], p[:, 1], color=C_PHY, lw=1.8)
    axes[1, 2].plot(3.0, 0.0, "k*", ms=11)
    axes[1, 2].set_title("T6  waypoint approach")
    axes[1, 2].set_xlabel("world x (m)"); axes[1, 2].set_ylabel("world y (m)")
    axes[1, 2].invert_xaxis()

    for ax in axes.flat:
        if not ax.get_xlabel():
            ax.set_xlabel("time (s)")
    fig.suptitle("M3  T1-T6 closed-loop responses "
                 "(GROUND_TRUTH_CONTROL_VALIDATION)", y=1.02)
    _save(fig, "09_T1_T6_responses.png")

    # Actuation detail for one representative manoeuvre.
    tr = outcomes["T5"].trace
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True)
    ax1.plot(tr.t, [r["wrench"]["mz"] for r in tr.rows], color=C_PHY, lw=1.6)
    ax1.set_ylabel("demanded Mz (N.m)"); ax1.set_title("T5  wrench and allocation")
    for joint, colour in zip(("prop_left_joint", "prop_right_joint"),
                             (C_RED, C_PHY)):
        ax2.plot(tr.t, [r["thrust"][joint] for r in tr.rows], color=colour,
                 lw=1.5, label=joint.replace("_joint", ""))
    ax2.axhline(50, color="0.5", ls=":", lw=1); ax2.axhline(-50, color="0.5", ls=":", lw=1)
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("thrust (N)")
    ax2.legend(frameon=False, fontsize=8)
    _save(fig, "10_T5_wrench_allocation.png")

    # T7 repeatability from three fresh T3 runs.
    fig, ax = plt.subplots(figsize=(5.4, 3.2))
    distances = []
    for trial in range(3):
        outcome = maneuvers.TESTS["T3"]()
        p = outcome.trace.position()
        ax.plot(outcome.trace.t, (p[:, 1] - p[0, 1]) * 1000.0, lw=1.5,
                label=f"trial {trial + 1}")
        distances.append(outcome.metrics["distance_m"])
    ax.set_xlabel("time (s)"); ax.set_ylabel("cross-track (mm)")
    ax.set_title(f"T7  repeatability, spread "
                 f"{max(distances) - min(distances) * 1:.2e} m")
    ax.legend(frameon=False, fontsize=8)
    _save(fig, "11_T7_repeatability.png")

    return {name: o.metrics for name, o in outcomes.items()}


# --- L7 -----------------------------------------------------------------

def p4_validated() -> dict:
    cfg = world_builder.load_config(validated=True)
    from . import harness
    original = harness.run
    validated_world = world_builder.VALIDATED_WORLD_PATH

    def run(cmds=None, **kw):
        kw.setdefault("world", validated_world)
        return original(cmds, **kw)

    harness.run = run
    protocol.harness.run = run
    try:
        result = protocol.run_test("P4", cfg)
    finally:
        harness.run = original
        protocol.harness.run = original
    print("  " + result.line().strip(), flush=True)
    if result.detail:
        print(f"        {result.detail}", flush=True)
    return {"status": result.status, "predicted": result.predicted,
            "measured": result.measured, "detail": result.detail}


def main() -> None:
    print("L7  P4 on VALIDATED")
    p4 = p4_validated()
    print("\nL8  execution-mode comparison")
    compared = comparison()
    for mode, values in compared.items():
        print(f"  {mode:8s} {values}", flush=True)
    print("\nL9  figures")
    figures()
    Path("baselines/M3/closeout.json").write_text(
        json.dumps({"p4_validated": p4, "execution_comparison": compared},
                   indent=2, default=str) + "\n")


if __name__ == "__main__":
    main()
