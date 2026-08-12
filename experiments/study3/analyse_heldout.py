#!/usr/bin/env python3
"""Analysis of the completed one-shot Study 3 held-out evaluation.

Reports the frozen comparisons declared in ``STUDY3_HELDOUT_DESIGN_V1.json``:
REACTIVE minus universal ``fixed_155`` as primary, and REACTIVE minus
deployment-informed FIXED as supporting. Read-only over immutable packets.

No threshold is applied and no verdict is assigned. The held-out result is
reported regardless of direction, per STUDY3_PROTOCOL.md section 10.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import PRIMARY  # noqa: E402

PACKETS = HERE / "redesign_results/heldout"
HELD_OUT_ROOT = 32_000_000
BOOTSTRAP = 10_000
RNG_SEED = 32_000_999          # analysis-only; affects no simulation
POLICIES = ("fixed", "deployment_fixed", "reactive")
METRICS = ("completed", "safety_violation", "overall_rmse_m", "rmse_transition_m",
           "peak_error_m", "unaided_time_s", "longest_unaided_gap_s",
           "survey_coverage_fraction", "optical_fixes", "acoustic_fixes",
           "physical_interventions", "mode_switches")


def load():
    rows = []
    for path in sorted(PACKETS.glob("*.json")):
        packet = json.loads(path.read_text())
        stored = packet.pop("packet_sha256", None)
        canonical = hashlib.sha256(json.dumps(
            packet, sort_keys=True, separators=(",", ":"),
            allow_nan=True).encode()).hexdigest()
        if stored != canonical:
            raise RuntimeError(f"packet checksum failed: {path}")
        if packet["identity"]["root"] != HELD_OUT_ROOT:
            raise RuntimeError(f"non-held-out packet in the held-out directory: {path}")
        rows.append(packet["result"])
    return rows


def bootstrap_ci(differences, strata, rng):
    differences = np.asarray(differences, dtype=float)
    strata = np.asarray(strata)
    groups = [np.flatnonzero(strata == s) for s in np.unique(strata)]
    means = np.empty(BOOTSTRAP)
    for draw in range(BOOTSTRAP):
        picked = np.concatenate([g[rng.integers(0, g.size, g.size)] for g in groups])
        means[draw] = differences[picked].mean()
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def contrast(table, keys, treatment, comparator, field, rng):
    differences = [table[k][treatment][field] - table[k][comparator][field] for k in keys]
    low, high = bootstrap_ci(differences, [k[0] for k in keys], rng)
    array = np.asarray(differences, dtype=float)
    return {"mean": float(array.mean()), "ci_low": low, "ci_high": high,
            "pairs": len(keys),
            "wins": int(np.sum(array < -1e-12)), "ties": int(np.sum(np.abs(array) <= 1e-12)),
            "losses": int(np.sum(array > 1e-12))}


def main():
    rng = np.random.default_rng(RNG_SEED)
    rows = load()
    table = {}
    for row in rows:
        table.setdefault((row["family"], row["index"]), {})[row["policy"]] = row
    table = {k: v for k, v in table.items() if len(v) == len(POLICIES)}
    primary = sorted(k for k in table if k[0] in PRIMARY)
    controls = sorted(k for k in table if k[0] not in PRIMARY)

    mean = lambda keys, policy, field: float(np.mean([table[k][policy][field] for k in keys]))
    report = {"schema": "study3_heldout_analysis_v1", "root": HELD_OUT_ROOT,
              "packets": len(rows), "primary_pairs": len(primary),
              "control_pairs": len(controls),
              "note": "One-shot held-out result, reported regardless of direction."}

    for scope, keys in (("primary", primary), ("controls", controls)):
        report[f"{scope}_means"] = {
            field: {policy: mean(keys, policy, field) for policy in POLICIES}
            for field in METRICS}

    contrasts = {}
    for field in METRICS:
        contrasts[f"reactive_minus_fixed.{field}"] = contrast(
            table, primary, "reactive", "fixed", field, rng)
        contrasts[f"reactive_minus_deployment.{field}"] = contrast(
            table, primary, "reactive", "deployment_fixed", field, rng)
    report["primary_contrasts"] = contrasts

    families = {}
    for family in sorted({k[0] for k in table}):
        keys = sorted(k for k in table if k[0] == family)
        families[family] = {
            "pairs": len(keys),
            "means": {policy: {f: mean(keys, policy, f)
                               for f in ("overall_rmse_m", "rmse_transition_m",
                                         "longest_unaided_gap_s", "completed",
                                         "safety_violation")}
                      for policy in POLICIES},
            "reactive_minus_fixed_rmse": contrast(table, keys, "reactive", "fixed",
                                                  "overall_rmse_m", rng),
            "reactive_minus_deployment_rmse": contrast(table, keys, "reactive",
                                                       "deployment_fixed",
                                                       "overall_rmse_m", rng)}
    report["families"] = families

    path = HERE / "redesign_results/heldout_analysis.json"
    path.write_text(json.dumps(report, sort_keys=True, indent=2, allow_nan=True) + "\n")
    print(json.dumps({k: report[k] for k in
                      ("schema", "root", "packets", "primary_pairs", "control_pairs")},
                     indent=2, sort_keys=True))
    print("\nprimary contrasts (95% family-stratified paired bootstrap)")
    for name in ("overall_rmse_m", "rmse_transition_m", "longest_unaided_gap_s",
                 "completed", "safety_violation"):
        for comparator, label in (("fixed", "vs universal fixed_155"),
                                  ("deployment", "vs deployment-informed")):
            c = contrasts[f"reactive_minus_{comparator}.{name}"]
            print(f"  {name:24s} {label:26s} {c['mean']:+9.4f} "
                  f"[{c['ci_low']:+.4f}, {c['ci_high']:+.4f}]  "
                  f"w/t/l {c['wins']}/{c['ties']}/{c['losses']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
