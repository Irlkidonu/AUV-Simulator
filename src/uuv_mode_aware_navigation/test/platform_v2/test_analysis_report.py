"""The standard report preserves pairing and exposes the Pareto view."""

from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[4] / "tools" / "analyse.py"
SPEC = importlib.util.spec_from_file_location("v2_analyse", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
ANALYSE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSE)


def _row(scenario, seed, policy, error, elapsed, altitude, path, completed="True"):
    return {
        "scenario": scenario,
        "seed": str(seed),
        "policy": policy,
        "completed": completed,
        "rms_cross_track_m": str(error),
        "safety_violations": "0",
        "elapsed_s": str(elapsed),
        "mean_altitude_m": str(altitude),
        "path_length_m": str(path),
    }


def test_report_uses_paired_differences() -> None:
    rows = [
        _row("A", 1, "fixed", 2.0, 10, 2, 5),
        _row("A", 1, "adaptive", 1.0, 8, 3, 6),
        _row("B", 2, "fixed", 4.0, 12, 2, 6),
        _row("B", 2, "adaptive", 2.0, 9, 3, 7),
    ]
    report = ANALYSE.build_report(
        rows, reference="fixed", methods=["adaptive"],
        metrics=["rms_cross_track_m"],
    )
    paired = report["paired"][0]
    assert paired["n"] == 2
    assert paired["mean_difference"] == -1.5
    assert paired["confidence_95"] == [-2.0, -1.0]
    assert report["survey_productivity_m2ps"]["adaptive"] > 2.0


def test_report_surfaces_static_pareto_front() -> None:
    policies = [
        _row("A", 1, "adaptive", 1.5, 10, 2, 10),
        _row("A", 1, "fixed", 2.0, 10, 2, 8),
    ]
    sweep = [
        _row("A", 1, "static_fast", 2.0, 10, 2, 10),
        _row("A", 1, "static_accurate", 1.0, 20, 2, 10),
    ]
    report = ANALYSE.build_report(
        policies, reference="fixed", methods=["adaptive"],
        metrics=["rms_cross_track_m"], sweep_rows=sweep,
    )
    assert set(report["pareto"]["frontier"]) == {"static_fast", "static_accurate"}
