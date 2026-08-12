#!/usr/bin/env python3
"""Single analysis of the Study 3 final DEVELOPMENT validation V5.

Evaluates exactly the eight gating criteria predeclared in
``STUDY3_FINAL_VALIDATION_V5_PROTOCOL.md``. Thresholds are taken from that
protocol and are not arguments: this script cannot be re-run with adjusted
definitions to obtain a different verdict.
"""
from __future__ import annotations

import hashlib
import json
import statistics as st
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import PRIMARY  # noqa: E402
from uuv_mode_aware_navigation.study3.transition_driver import (  # noqa: E402
    truth_side_best_viable_mode)
from uuv_mode_aware_navigation.study3.environment_generator import (  # noqa: E402
    generate_environment, load_environment_config)

PACKETS = HERE / "redesign_results/final_validation_v5"
#: The smoke run shares the packet directory; the campaign is root-scoped.
VALIDATION_ROOT = 31_900_000
ENVIRONMENT_CONFIG = HERE / "examples/moderate_severe_variable_environment.json"

# --- predeclared thresholds (protocol section "Predeclared gating criteria")
SAFETY_MARGIN = 0.02
COMPLETION_MARGIN = 0.05
NON_INFERIORITY_RMSE_M = 0.10
ADAPTATION_RATE = 0.70
ADAPTATION_LATENCY_S = 12.0
BOOTSTRAP = 10_000
RNG_SEED = 31_900_999          # analysis-only; affects no simulation


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
        if packet["identity"]["root"] != VALIDATION_ROOT:
            continue                      # smoke packets share this directory
        result = packet["result"]
        result["environment_seed"] = packet["identity"]["environment_seed"]
        rows.append(result)
    return rows


def paired(rows, part, key):
    """Index runs by pairing key -> policy -> result."""
    table = {}
    for row in rows:
        if row["part"] != part:
            continue
        table.setdefault(key(row), {})[row["policy"]] = row
    return {k: v for k, v in table.items() if len(v) == 3}


def bootstrap_ci(differences, strata, rng):
    """95% stratified paired bootstrap over the difference vector."""
    differences = np.asarray(differences, dtype=float)
    strata = np.asarray(strata)
    if differences.size == 0:
        return (float("nan"), float("nan"))
    means = np.empty(BOOTSTRAP)
    groups = [np.flatnonzero(strata == s) for s in np.unique(strata)]
    for draw in range(BOOTSTRAP):
        picked = np.concatenate([
            g[rng.integers(0, g.size, g.size)] for g in groups])
        means[draw] = differences[picked].mean()
    return (float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5)))


def contrast(table, treatment, comparator, field, rng):
    keys = sorted(table)
    differences = [table[k][treatment][field] - table[k][comparator][field]
                   for k in keys]
    strata = [k[0] for k in keys]
    low, high = bootstrap_ci(differences, strata, rng)
    return {"mean": float(np.mean(differences)) if differences else float("nan"),
            "ci_low": low, "ci_high": high, "pairs": len(keys)}


def adaptation(rows):
    """C7/C8: adequate contemporaneously-supported match after a viability change.

    Uses the corrected pilot definitions. Truth-side viability establishes the
    acceptable set; it never enters a policy.
    """
    config = load_environment_config(ENVIRONMENT_CONFIG)
    episodes, matched, latencies = 0, 0, []
    for row in rows:
        if row["part"] != "generated" or row["policy"] != "reactive":
            continue
        realization = generate_environment(config, row["environment_seed"], 180.0, 2.0)
        trace = row.get("causal_trace") or []
        if not trace:
            continue
        # Truth-side viability at the altitude the vehicle physically flew.
        # `trace` entries are (time_s, ..., action_dict, true_z, error) sampled
        # every dt, so the frame index is time / dt.
        best = [truth_side_best_viable_mode(
                    realization.physical_state(int(round(entry[0] / 2.0)),
                                               altitude_m=max(0.0, -float(entry[7]))))
                for entry in trace]
        selected = [entry[6]["navigation_mode"] for entry in trace]
        times = [entry[0] for entry in trace]
        # An episode is a maximal run of constant truth-side best viable mode
        # that begins after launch, i.e. after the first sample.
        start = 1
        while start < len(best):
            end = start
            while end + 1 < len(best) and best[end + 1] == best[start]:
                end += 1
            if best[start] != best[start - 1] and (end - start + 1) * 2.0 >= 6.0:
                episodes += 1
                hit = None
                for position in range(start, end + 1):
                    supported = selected[position] == best[start]
                    if supported:
                        hit = times[position] - times[start]
                        break
                if hit is not None:
                    matched += 1
                    latencies.append(hit)
            start = end + 1
    return {"episodes": episodes, "matched": matched,
            "rate": (matched / episodes) if episodes else float("nan"),
            "median_latency_s": st.median(latencies) if latencies else float("nan"),
            "mean_latency_s": st.mean(latencies) if latencies else float("nan")}


def main():
    rng = np.random.default_rng(RNG_SEED)
    rows = load()
    scripted = paired(rows, "scripted", lambda r: (r["family"], r["index"]))
    primary = {k: v for k, v in scripted.items() if k[0] in PRIMARY}
    controls = {k: v for k, v in scripted.items() if k[0] not in PRIMARY}
    generated = paired(rows, "generated", lambda r: (r["family"], r["environment_seed"]))

    mean = lambda table, policy, field: float(np.mean(
        [table[k][policy][field] for k in sorted(table)]))

    report = {"schema": "study3_final_validation_v5_analysis_v1",
              "packets": len(rows), "primary_pairs": len(primary),
              "control_pairs": len(controls), "generated_pairs": len(generated)}

    for field in ("completed", "safety_violation", "overall_rmse_m",
                  "rmse_transition_m", "peak_error_m", "unaided_time_s",
                  "longest_unaided_gap_s", "survey_coverage_fraction",
                  "physical_interventions", "mode_switches"):
        report.setdefault("primary_means", {})[field] = {
            policy: mean(primary, policy, field)
            for policy in ("fixed", "deployment_fixed", "reactive")}

    contrasts = {}
    for field in ("overall_rmse_m", "rmse_transition_m", "longest_unaided_gap_s",
                  "completed", "safety_violation", "unaided_time_s"):
        contrasts[f"reactive_minus_fixed.{field}"] = contrast(
            primary, "reactive", "fixed", field, rng)
        contrasts[f"reactive_minus_deployment.{field}"] = contrast(
            primary, "reactive", "deployment_fixed", field, rng)
    report["contrasts"] = contrasts
    report["adaptation"] = adaptation(rows)

    pm = report["primary_means"]
    ad = report["adaptation"]
    criteria = {
        "C1_safety": (pm["safety_violation"]["reactive"]
                      <= pm["safety_violation"]["deployment_fixed"] + SAFETY_MARGIN
                      and pm["safety_violation"]["reactive"]
                      <= pm["safety_violation"]["fixed"] + SAFETY_MARGIN),
        "C2_completion": (pm["completed"]["reactive"]
                          >= pm["completed"]["deployment_fixed"] - COMPLETION_MARGIN),
        "C3_overall_rmse_superiority":
            contrasts["reactive_minus_fixed.overall_rmse_m"]["mean"] < 0
            and contrasts["reactive_minus_fixed.overall_rmse_m"]["ci_high"] < 0,
        "C4_transition_rmse_superiority":
            contrasts["reactive_minus_fixed.rmse_transition_m"]["mean"] < 0
            and contrasts["reactive_minus_fixed.rmse_transition_m"]["ci_high"] < 0,
        "C5_aiding_gap_superiority":
            contrasts["reactive_minus_fixed.longest_unaided_gap_s"]["mean"] < 0
            and contrasts["reactive_minus_fixed.longest_unaided_gap_s"]["ci_high"] < 0,
        "C6_non_inferiority":
            contrasts["reactive_minus_deployment.overall_rmse_m"]["ci_high"]
            <= NON_INFERIORITY_RMSE_M
            and contrasts["reactive_minus_deployment.completed"]["ci_low"]
            >= -COMPLETION_MARGIN,
        "C7_adaptation_rate": bool(ad["episodes"] > 0
                                   and ad["rate"] >= ADAPTATION_RATE),
        "C8_adaptation_latency": bool(ad["episodes"] > 0
                                      and ad["median_latency_s"] <= ADAPTATION_LATENCY_S),
    }
    report["criteria"] = {k: bool(v) for k, v in criteria.items()}
    report["verdict"] = ("RECOMMEND FREEZE" if all(criteria.values())
                         else "NO-FREEZE")
    report["failed_criteria"] = sorted(k for k, v in criteria.items() if not v)

    path = HERE / "redesign_results/final_validation_v5_analysis.json"
    path.write_text(json.dumps(report, sort_keys=True, indent=2,
                               allow_nan=True) + "\n")
    print(json.dumps(report, sort_keys=True, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
