#!/usr/bin/env python3
"""Final result tables for Study 3. Derivation only.

Reads frozen, checksum-verified packets and the preserved analysis records and
emits tables. It computes no new science: every number here is either copied
from a preserved analysis record or aggregated directly from immutable packets.
No threshold is applied and no interpretation is added.
"""
from __future__ import annotations

import csv
import hashlib
import json
import statistics as st
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "redesign_results"
OUT = HERE / "final_tables"
POLICIES = ("deployment_fixed", "reactive", "predictive")


def load(path):
    return json.loads(Path(path).read_text())


def packets(directory, root):
    rows = []
    for path in sorted((RESULTS / directory).glob("*.json")):
        packet = json.loads(path.read_text())
        stored = packet.pop("packet_sha256", None)
        canonical = hashlib.sha256(json.dumps(
            packet, sort_keys=True, separators=(",", ":"),
            allow_nan=True).encode()).hexdigest()
        if stored != canonical:
            raise RuntimeError(f"checksum failed: {path}")
        if packet["identity"]["root"] != root:
            raise RuntimeError(f"unexpected root in {path}")
        row = dict(packet["result"])
        row["_identity"] = packet["identity"]
        rows.append(row)
    return rows


def write_csv(name, header, records):
    path = OUT / name
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(records)
    return path


def contrasts_table(analysis, part, label):
    records = []
    for key, value in sorted(analysis["contrasts"].items()):
        comparison, metric = key.rsplit(".", 1)
        records.append([label, part, comparison, metric,
                        value["mean"], value["ci_low"], value["ci_high"],
                        value["pairs"], value["wins"], value["ties"], value["losses"]])
    return records


def means_table(analysis, part, label):
    records = []
    for metric, by_policy in sorted(analysis["means"].items()):
        for policy in POLICIES:
            if policy in by_policy:
                records.append([label, part, policy, metric, by_policy[policy]])
    return records


def family_summary(rows):
    """Per-family Part A summary, aggregated from packets."""
    grouped = {}
    for row in rows:
        identity = row["_identity"]
        if identity["part"] != "scripted":
            continue
        grouped.setdefault((identity["family"], identity["group"],
                            identity["policy"]), []).append(row)
    records = []
    for (family, group, policy), members in sorted(grouped.items()):
        mean = lambda field: st.mean(m[field] for m in members)
        records.append([family, group, policy, len(members),
                        mean("completed"), mean("safety_violation"),
                        mean("overall_rmse_m"), mean("rmse_transition_m"),
                        mean("peak_error_m"), mean("longest_unaided_gap_s"),
                        mean("unaided_time_s"), mean("survey_coverage_fraction"),
                        mean("mode_switches"), mean("physical_interventions"),
                        mean("preemptive_actions")])
    return records


def main():
    OUT.mkdir(exist_ok=True)

    generated = load(RESULTS / "heldout_v2_analysis_generated.json")
    scripted = load(RESULTS / "heldout_v2_analysis_scripted.json")
    original = load(RESULTS / "heldout_analysis.json")

    # --- contrasts, both parts, plus the original 32M block -----------------
    records = (contrasts_table(generated, "B_generated", "heldout_36M_corrected")
               + contrasts_table(scripted, "A_scripted", "heldout_36M_corrected"))
    for key, value in sorted(original["primary_contrasts"].items()):
        comparison, metric = key.rsplit(".", 1)
        records.append(["heldout_32M_precorrection", "A_scripted_primary",
                        comparison, metric, value["mean"], value["ci_low"],
                        value["ci_high"], value["pairs"], value["wins"],
                        value["ties"], value["losses"]])
    write_csv("contrasts.csv",
              ["block", "part", "comparison", "metric", "mean", "ci_low",
               "ci_high", "pairs", "wins", "ties", "losses"], records)

    # --- means ---------------------------------------------------------------
    records = (means_table(generated, "B_generated", "heldout_36M_corrected")
               + means_table(scripted, "A_scripted", "heldout_36M_corrected"))
    for scope, key in (("A_scripted_primary", "primary_means"),
                       ("A_scripted_controls", "controls_means")):
        for metric, by_policy in sorted(original[key].items()):
            for policy, value in sorted(by_policy.items()):
                records.append(["heldout_32M_precorrection", scope, policy,
                                metric, value])
    write_csv("means.csv", ["block", "part", "policy", "metric", "mean"], records)

    # --- per-family Part A ---------------------------------------------------
    rows = packets("heldout_v2", 36_000_000)
    write_csv("part_a_family_summary.csv",
              ["family", "group", "policy", "n", "completed", "safety_violation",
               "overall_rmse_m", "rmse_transition_m", "peak_error_m",
               "longest_unaided_gap_s", "unaided_time_s",
               "survey_coverage_fraction", "mode_switches",
               "physical_interventions", "preemptive_actions"],
              family_summary(rows))

    # --- adaptation ----------------------------------------------------------
    records = []
    for policy, value in sorted(generated["adaptation_v5_definition"].items()):
        records.append(["B_generated", "v5_c7_c8", policy, value["episodes"],
                        value["matched"], value["coverage"],
                        value["median_latency_s"], value["mean_latency_s"]])
    for policy, value in sorted(generated["adaptation_pilot_definition"].items()):
        records.append(["B_generated", "pilot_adequate", policy, value["episodes"],
                        value["adequate_matches"], value["adequate_rate"],
                        value["adequate_delay_median_s"], value["adequate_delay_mean_s"]])
        records.append(["B_generated", "pilot_exact", policy,
                        value["exact_evaluable_episodes"], value["exact_matches"],
                        value["exact_rate"], value["exact_delay_median_s"],
                        value["exact_delay_mean_s"]])
    write_csv("adaptation.csv",
              ["part", "definition", "policy", "episodes", "matched", "rate",
               "median_delay_s", "mean_delay_s"], records)

    # --- switching and intervention cost ------------------------------------
    records = []
    for part, analysis in (("A_scripted", scripted), ("B_generated", generated)):
        for metric in ("mode_switches", "physical_interventions",
                       "preemptive_actions", "survey_coverage_fraction"):
            for policy in POLICIES:
                records.append([part, policy, metric,
                                analysis["means"][metric][policy]])
    write_csv("switching_and_intervention_cost.csv",
              ["part", "policy", "metric", "mean"], records)

    # --- safety and completion ----------------------------------------------
    records = []
    for part, analysis in (("A_scripted", scripted), ("B_generated", generated)):
        for policy in POLICIES:
            records.append([part, policy,
                            analysis["means"]["completed"][policy],
                            analysis["means"]["safety_violation"][policy]])
    write_csv("safety_and_completion.csv",
              ["part", "policy", "completed", "safety_violation"], records)

    # --- the decision record -------------------------------------------------
    design = load(HERE / "STUDY3_HELDOUT_V2_DESIGN.json")
    primary = generated["contrasts"]["reactive_minus_deployment.overall_rmse_m"]
    adequate = generated["adaptation_pilot_definition"]["reactive"]["adequate_rate"]
    decision = {
        "schema": "study3_final_decision_record_v1",
        "block": "held-out root 36,000,000, corrected controller, executed once",
        "decision_rules": design["decision_rules"],
        "condition_1_rmse": {
            "metric": "reactive_minus_deployment.overall_rmse_m (Part B only)",
            "mean": primary["mean"], "ci_low": primary["ci_low"],
            "ci_high": primary["ci_high"], "pairs": primary["pairs"],
            "met": bool(primary["mean"] < 0 and primary["ci_high"] < 0)},
        "condition_2_adaptation": {
            "metric": "replay-verified adequate viable-mode adaptation, REACTIVE, Part B",
            "value": adequate, "threshold": 0.50, "met": bool(adequate > 0.50)},
        "primary_claim_supported": bool(primary["mean"] < 0 and primary["ci_high"] < 0
                                        and adequate > 0.50),
        "part_a_role": "robustness and reproducibility; superiority not required",
        "pooling": "prohibited; Parts A and B never combined",
        "predictive": "secondary, no threshold, reported regardless of direction"}
    (OUT / "final_decision_record.json").write_text(
        json.dumps(decision, indent=1, sort_keys=True) + "\n")

    # --- combined bundle -----------------------------------------------------
    bundle = {
        "schema": "study3_final_tables_v1",
        "heldout_36M_corrected": {"part_a_scripted": scripted,
                                  "part_b_generated": generated},
        "heldout_32M_precorrection": original,
        "decision": decision}
    (OUT / "study3_final_tables.json").write_text(
        json.dumps(bundle, indent=1, sort_keys=True, allow_nan=True) + "\n")

    print("final tables written to", OUT.name)
    for path in sorted(OUT.iterdir()):
        print(f"  {path.name:40s} {path.stat().st_size:>8d} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
