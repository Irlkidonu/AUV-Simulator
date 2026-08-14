"""P15 convergence study: is 0.5 ms inside the converged regime, or merely closer?

A single 1 ms vs 0.5 ms comparison cannot answer that -- two points always
differ by *something*. Four timesteps across four dynamic quantities can, by
showing whether successive refinements stop changing the answer.

Quantities: surge terminal, heave terminal, steady yaw rate, and one contact
case. Contact is included because it is the stiffest thing DART has to resolve
here and is the most likely to need a finer step than free-water motion does.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import numpy as np

from .. import world_builder
from . import harness, protocol

STEPS_MS = (2.0, 1.0, 0.5, 0.25)


def _world_at(step_ms: float, directory: Path) -> Path:
    return protocol.variant_world(
        {"physics": {"engine": {"max_step_size_s": step_ms / 1000.0}}},
        directory, validated=True)


def measure(world: Path, cfg: dict) -> dict:
    limit = cfg["vehicle_bluerov2_phys"]["thrusters"]["common"]["max_thrust_cmd_N"]

    surge = harness.run([harness.Command(0.0, protocol.thrust(surge=-2 * limit))],
                        duration_s=22.0, settle_s=3.0, world=world)
    heave = harness.run([harness.Command(0.0, protocol.thrust(heave=limit))],
                        duration_s=22.0, settle_s=3.0, world=world)
    yaw = harness.run([harness.Command(0.0, protocol.thrust(yaw=2 * limit))],
                      duration_s=18.0, settle_s=3.0, world=world)
    # Contact: drive gently at the dock and record where it is stopped.
    contact = harness.run([harness.Command(0.0, protocol.thrust(surge=18.0))],
                          duration_s=26.0, settle_s=3.0, world=world)

    speeds = np.linalg.norm(contact.world_velocity(), axis=1)
    return {
        "surge_terminal_mps": abs(float(
            surge.steady_value(surge.body_velocity(), last_s=3.0)[0])),
        "heave_terminal_mps": abs(float(
            heave.steady_value(heave.body_velocity(), last_s=3.0)[2])),
        "yaw_rate_radps": abs(float(
            yaw.steady_value(yaw.body_rates(), last_s=3.0)[2])),
        "contact_closest_x_m": float(contact.position[:, 0].min()),
        "contact_max_speed_mps": float(speeds.max()),
        "contact_finite": bool(np.isfinite(contact.position).all()),
        "sample_rate_hz": float(surge.meta["mean_rate_hz"]),
    }


def run(output: Path | None = None, steps=None) -> dict:
    cfg = world_builder.load_config(validated=True)
    results: dict[str, dict] = {}
    for step in (steps or STEPS_MS):
        with tempfile.TemporaryDirectory() as tmp:
            world = _world_at(step, Path(tmp))
            results[f"{step}ms"] = measure(world, cfg)
            print(f"  {step:>5} ms  " + "  ".join(
                f"{k.split('_')[0]}={v:.5f}" if isinstance(v, float) else f"{k}={v}"
                for k, v in results[f"{step}ms"].items()
                if k.endswith(("mps", "radps", "x_m"))), flush=True)

    # Successive relative change, which is what "converged" actually means.
    keys = ("surge_terminal_mps", "heave_terminal_mps", "yaw_rate_radps",
            "contact_closest_x_m")
    deltas = {}
    labels = list(results)
    for coarse, fine in zip(labels, labels[1:]):
        deltas[f"{coarse} -> {fine}"] = {
            key: abs(results[fine][key] - results[coarse][key])
                 / max(abs(results[coarse][key]), 1e-12) * 100.0
            for key in keys}

    report = {"steps_ms": [float(l[:-2]) for l in results], "results": results,
              "successive_change_percent": deltas}
    if output:
        Path(output).write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    import sys
    chosen = [float(a) for a in sys.argv[1:]] or None
    tag = "_".join(str(s) for s in (chosen or STEPS_MS)).replace(".", "p")
    report = run(Path(f"baselines/M2.5/timestep_{tag}.json"), chosen)
    print("\nsuccessive change (%):")
    for pair, values in report["successive_change_percent"].items():
        print(f"  {pair:18s} " + "  ".join(f"{k.split('_')[0]}={v:6.3f}"
                                           for k, v in values.items()))
