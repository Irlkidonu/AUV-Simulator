#!/usr/bin/env python3
"""Analysis of the post-freeze exploratory Part E1 block.

Read-only over the immutable Part E1 packets at exploratory root 33,000,000.
Reports outcome means, paired contrasts with intervals, and adaptation under
both predeclared definitions.

**No threshold is applied and no verdict is assigned by this code.** It reports
the frozen comparisons with intervals; the decision rules are stated in
STUDY3_HELDOUT_V2_DESIGN.json and are the researcher's. The result is reported
regardless of direction. Set HELDOUT_V2_PART to "scripted" or "generated".

This block concerns the CORRECTED controller. It does not revise the original
held-out block at root 32,000,000, which stands as evidence for the
pre-correction controller.

The adequate/exact definition needs per-step observability, which a packet does
not store, so REACTIVE runs are deterministically replayed in-process. Every
replay must reproduce its packet's ``trace_digest``; a mismatch aborts. The
replay is the same run, so it adds no evidence and spends no seed.
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

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FixedConfiguration, PolicyKind, Study3Policy, generate_environment,
    load_environment_config, run_one, truth_side_best_viable_mode)

PACKETS = HERE / "redesign_results/heldout_v2"
CONFIG_PATH = HERE / "examples/moderate_severe_variable_environment.json"
HELD_OUT_ROOT = 36_000_000
BOOTSTRAP = 10_000
RNG_SEED = 36_000_999          # analysis-only; affects no simulation
POLICIES = ("deployment_fixed", "reactive", "predictive")
HORIZON_S, DT_S = 180.0, 2.0
BOUNDARY = .35
PERSISTENCE_SAMPLES = 3
MINIMUM_EPISODE_S = 6.0
METRICS = ("completed", "safety_violation", "overall_rmse_m", "rmse_transition_m",
           "peak_error_m", "unaided_time_s", "longest_unaided_gap_s",
           "survey_coverage_fraction", "optical_fixes", "acoustic_fixes",
           "physical_interventions", "mode_switches", "preemptive_actions")


def load():
    packets = []
    for path in sorted(PACKETS.glob("*.json")):
        packet = json.loads(path.read_text())
        stored = packet.pop("packet_sha256", None)
        canonical = hashlib.sha256(json.dumps(
            packet, sort_keys=True, separators=(",", ":"),
            allow_nan=True).encode()).hexdigest()
        if stored != canonical:
            raise RuntimeError(f"packet checksum failed: {path}")
        if packet["identity"]["root"] != HELD_OUT_ROOT:
            raise RuntimeError(f"foreign packet in the held-out v2 directory: {path}")
        packets.append(packet)
    return packets


def bootstrap_ci(differences, rng):
    """95% paired bootstrap. One generated family, so a single stratum."""
    differences = np.asarray(differences, dtype=float)
    if differences.size == 0:
        return float("nan"), float("nan")
    means = np.empty(BOOTSTRAP)
    for draw in range(BOOTSTRAP):
        means[draw] = differences[rng.integers(0, differences.size,
                                               differences.size)].mean()
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def contrast(table, keys, treatment, comparator, field, rng):
    array = np.asarray([table[k][treatment][field] - table[k][comparator][field]
                        for k in keys], dtype=float)
    low, high = bootstrap_ci(array, rng)
    return {"mean": float(array.mean()), "ci_low": low, "ci_high": high,
            "pairs": len(keys),
            "wins": int(np.sum(array < -1e-12)), "ties": int(np.sum(np.abs(array) <= 1e-12)),
            "losses": int(np.sum(array > 1e-12))}


# --- adaptation, definition 1: the V5 C7/C8 definition -----------------------
# Transcribed from analyse_final_validation_v5.py::adaptation without change,
# so the coverage number here is directly comparable to the 59.8% figure.

def adaptation_v5(rows, policy):
    config = load_environment_config(CONFIG_PATH)
    episodes, matched, latencies = 0, 0, []
    for row in rows:
        if row["policy"] != policy:
            continue
        realization = generate_environment(config, row["environment_seed"],
                                           HORIZON_S, DT_S)
        trace = row.get("causal_trace") or []
        if not trace:
            continue
        best = [truth_side_best_viable_mode(
                    realization.physical_state(int(round(entry[0] / DT_S)),
                                               altitude_m=max(0.0, -float(entry[7]))))
                for entry in trace]
        selected = [entry[6]["navigation_mode"] for entry in trace]
        times = [entry[0] for entry in trace]
        start = 1
        while start < len(best):
            end = start
            while end + 1 < len(best) and best[end + 1] == best[start]:
                end += 1
            if best[start] != best[start - 1] and (end - start + 1) * DT_S >= MINIMUM_EPISODE_S:
                episodes += 1
                hit = next((times[p] - times[start] for p in range(start, end + 1)
                            if selected[p] == best[start]), None)
                if hit is not None:
                    matched += 1
                    latencies.append(hit)
            start = end + 1
    return {"definition": "V5 C7/C8", "policy": policy, "episodes": episodes,
            "matched": matched,
            "coverage": (matched / episodes) if episodes else float("nan"),
            "median_latency_s": st.median(latencies) if latencies else float("nan"),
            "mean_latency_s": st.mean(latencies) if latencies else float("nan"),
            "max_latency_s": max(latencies) if latencies else float("nan")}


# --- adaptation, definition 2: the corrected pilot adequate/exact ------------
# physical_acceptable_modes and observable_support are transcribed from
# analyse_generated_environment_pilot_corrected.py without change.

def physical_acceptable_modes(state):
    response = dict(state.service_response_probability)
    absolute = set()
    acoustic_ok = state.acoustic_noise_db <= 65.
    if ("lbl" in state.deployed_acoustic_services and acoustic_ok and
            response.get("lbl", state.acoustic_response_probability) >= .5 and
            state.lbl_geometry_scale >= .35):
        absolute.add("lbl_aided")
    if ("usbl" in state.deployed_acoustic_services and acoustic_ok and
            response.get("usbl", state.acoustic_response_probability) >= .5):
        absolute.add("usbl_aided")
    optical = state.turbidity <= .35
    bottom = state.dvl_lock_probability >= .5
    water = state.dvl_water_track_probability >= .5
    if optical:
        absolute.add("optical_dvl" if bottom else "optical_no_bottom_lock")
    if absolute:
        return frozenset(absolute)
    if bottom or water:
        return frozenset({"relative_dead_reckoning"})
    return frozenset({"terminal_degraded"})


def observable_support(record, mode):
    observation = record["observation"]
    belief = record["belief"]
    services = {x.name for x in observation.acoustic.service_evidence
                if x.responding and x.gives_position}
    if mode == "lbl_aided":
        return "lbl" in services
    if mode == "usbl_aided":
        return "usbl" in services
    if mode == "optical_dvl":
        return (observation.optical.available and belief["optical"] >= BOUNDARY
                and observation.dvl.bottom_lock)
    if mode == "optical_no_bottom_lock":
        return (observation.optical.available and belief["optical"] >= BOUNDARY
                and not observation.dvl.bottom_lock)
    if mode == "relative_dead_reckoning":
        absolute_observed = bool(services or (observation.optical.available
                                              and belief["optical"] >= BOUNDARY))
        return (not absolute_observed and belief["velocity"] >= BOUNDARY and
                (observation.dvl.bottom_lock or observation.dvl.water_track))
    if mode == "terminal_degraded":
        return record["action"].mission_action == "surface_for_gps"
    return False


def episode_spans(values, limit):
    result, start = [], 0
    for end in range(1, limit + 1):
        if end == limit or values[end] != values[start]:
            if start > 0 and end - start >= PERSISTENCE_SAMPLES:
                result.append((start, end, values[start]))
            start = end
    return result


def replay(packet):
    """Re-execute one packet in-process and verify it is the same run."""
    identity = packet["identity"]
    realization = generate_environment(load_environment_config(CONFIG_PATH),
                                       identity["environment_seed"], HORIZON_S, DT_S)

    class Recorder(Study3Policy):
        records = []

        def step(self, observation):
            action, output = super().step(observation)
            self.__class__.records.append({
                "observation": observation, "action": action,
                "belief": dict(output.belief.usable_probability)})
            return action, output

    Recorder.records = []
    result, trace = run_one(
        identity["root"], identity["family"], identity["index"],
        PolicyKind(identity["policy"]),
        FixedConfiguration(**identity["configuration"]),
        horizon_s=HORIZON_S, dt_s=DT_S, image_period_s=4.0, keep_trace=True,
        redesign_version=3, environment_realization=realization,
        policy_factory=Recorder)
    if result.trace_digest != packet["result"]["trace_digest"]:
        raise RuntimeError(
            f"replay diverged for seed {identity['environment_seed']}: the "
            "analysis is not observing the executed run")
    return Recorder.records, trace, realization


def adaptation_pilot(packets, policy):
    episodes = []
    for packet in packets:
        if packet["identity"]["policy"] != policy:
            continue
        records, trace, realization = replay(packet)
        acceptable = [physical_acceptable_modes(
                          realization.physical_state(step, altitude_m=-row[7],
                                                     position_xy=(0., 0.)))
                      for step, row in enumerate(trace)]
        terminal = next((i for i, r in enumerate(records)
                         if r["action"].mission_action == "surface_for_gps"), None)
        limit = len(records) if terminal is None else terminal + 1
        limit = min(limit, len(acceptable))
        for start, end, modes in episode_spans(acceptable, limit):
            ambiguous = len(modes) > 1
            adequate = next((j for j in range(start, end)
                             if records[j]["action"].navigation_mode in modes and
                             observable_support(records[j],
                                                records[j]["action"].navigation_mode)), None)
            preferred = next(iter(modes)) if not ambiguous else None
            exact = (next((j for j in range(start, end)
                           if records[j]["action"].navigation_mode == preferred and
                           observable_support(records[j], preferred)), None)
                     if preferred is not None else None)
            episodes.append({
                "duration_s": (end - start) * DT_S, "ambiguous": ambiguous,
                "adequate_match": adequate is not None,
                "adequate_delay_s": None if adequate is None else (adequate - start) * DT_S,
                "exact_evaluable": preferred is not None,
                "exact_match": None if preferred is None else exact is not None,
                "exact_delay_s": None if exact is None else (exact - start) * DT_S})
    evaluable = [e for e in episodes if e["exact_evaluable"]]
    adequate_delays = [e["adequate_delay_s"] for e in episodes if e["adequate_match"]]
    exact_delays = [e["exact_delay_s"] for e in evaluable if e["exact_match"]]
    number = lambda values, function: function(values) if values else float("nan")
    return {"definition": "corrected pilot adequate/exact", "policy": policy,
            "episodes": len(episodes),
            "ambiguous_episodes": sum(e["ambiguous"] for e in episodes),
            "adequate_matches": sum(e["adequate_match"] for e in episodes),
            "adequate_rate": (sum(e["adequate_match"] for e in episodes) / len(episodes))
                             if episodes else float("nan"),
            "adequate_delay_median_s": number(adequate_delays, st.median),
            "adequate_delay_mean_s": number(adequate_delays, st.mean),
            "adequate_delay_max_s": number(adequate_delays, max),
            "exact_evaluable_episodes": len(evaluable),
            "exact_matches": sum(bool(e["exact_match"]) for e in evaluable),
            "exact_rate": (sum(bool(e["exact_match"]) for e in evaluable) / len(evaluable))
                          if evaluable else float("nan"),
            "exact_delay_median_s": number(exact_delays, st.median),
            "exact_delay_mean_s": number(exact_delays, st.mean),
            "replays_verified": sum(1 for p in packets
                                    if p["identity"]["policy"] == policy)}


def main():
    rng = np.random.default_rng(RNG_SEED)
    packets = load()
    rows = []
    for packet in packets:
        row = dict(packet["result"])
        row["environment_seed"] = packet["identity"]["environment_seed"]
        rows.append(row)

    part = __import__("os").environ.get("HELDOUT_V2_PART", "generated")
    # Parts A and B are analysed separately and are never pooled. Restrict every
    # downstream computation, not only the pairing table, to the selected part.
    packets = [p for p in packets if p["identity"]["part"] == part]
    rows = [r for r in rows if r.get("part") == part]
    table = {}
    for row in rows:
        key = ((row["family"], row["index"]) if part == "scripted"
               else row["environment_seed"])
        table.setdefault(key, {})[row["policy"]] = row
    keys = sorted((k for k, v in table.items() if len(v) == len(POLICIES)),
                  key=lambda k: (str(k),))

    mean = lambda policy, field: float(np.mean([table[k][policy][field] for k in keys]))
    report = {"schema": "study3_heldout_v2_analysis_v1", "part": part,
              "classification": "HELD_OUT",
              "not_held_out": True, "not_confirmatory": True,
              "note": "Exploratory. No threshold applied, no verdict assigned. "
                      "Cannot revise the held-out result or the freeze decision.",
              "root": HELD_OUT_ROOT, "packets": len(packets),
              "paired_environments": len(keys),
              "means": {field: {policy: mean(policy, field) for policy in POLICIES}
                        for field in METRICS}}

    contrasts = {}
    for treatment, comparator, label in (("reactive", "deployment_fixed",
                                          "reactive_minus_deployment"),
                                         ("predictive", "reactive",
                                          "predictive_minus_reactive"),
                                         ("predictive", "deployment_fixed",
                                          "predictive_minus_deployment")):
        for field in METRICS:
            contrasts[f"{label}.{field}"] = contrast(table, keys, treatment,
                                                     comparator, field, rng)
    report["contrasts"] = contrasts
    # Both adaptation definitions are expressed over generated environment
    # realizations, so they are computed for Part B only. The frozen decision
    # rules bind adaptation to Part B alone, so this restricts nothing they use.
    if part == "generated":
        report["adaptation_v5_definition"] = {p: adaptation_v5(rows, p) for p in POLICIES}
        report["adaptation_pilot_definition"] = {p: adaptation_pilot(packets, p)
                                                 for p in ("reactive", "predictive")}
    else:
        report["adaptation_v5_definition"] = {}
        report["adaptation_pilot_definition"] = {}

    path = HERE / "redesign_results/heldout_v2_analysis.json"
    path.write_text(json.dumps(report, sort_keys=True, indent=2, allow_nan=True) + "\n")

    print(f"packets {len(packets)}  paired environments {len(keys)}")
    print("\nmeans by policy")
    print(f"  {'metric':26s} {'DEPLOY_FIXED':>14s} {'REACTIVE':>12s} {'PREDICTIVE':>12s}")
    for field in METRICS:
        m = report["means"][field]
        print(f"  {field:26s} {m['deployment_fixed']:14.4f} {m['reactive']:12.4f} "
              f"{m['predictive']:12.4f}")
    print("\npaired contrasts (95% paired bootstrap, 60 environments)")
    for label in ("reactive_minus_deployment", "predictive_minus_reactive",
                  "predictive_minus_deployment"):
        print(f"  [{label}]")
        for field in METRICS:
            c = contrasts[f"{label}.{field}"]
            print(f"    {field:26s} {c['mean']:+10.4f} "
                  f"[{c['ci_low']:+.4f}, {c['ci_high']:+.4f}]  "
                  f"w/t/l {c['wins']}/{c['ties']}/{c['losses']}")
    if not report["adaptation_v5_definition"]:
        print("\n(adaptation metrics are defined over generated environments; "
              "not computed for Part A)")
        return 0
    print("\nadaptation, V5 C7/C8 definition")
    for policy in POLICIES:
        a = report["adaptation_v5_definition"][policy]
        print(f"  {policy:16s} episodes {a['episodes']:4d}  matched {a['matched']:4d}  "
              f"coverage {a['coverage']:.4f}  median latency {a['median_latency_s']:.1f} s")
    a = report["adaptation_pilot_definition"]["reactive"]
    print("\nadaptation, corrected pilot definition (REACTIVE, replay-verified)")
    print(f"  episodes {a['episodes']}  ambiguous {a['ambiguous_episodes']}")
    print(f"  adequate {a['adequate_matches']}/{a['episodes']} = {a['adequate_rate']:.4f}  "
          f"median delay {a['adequate_delay_median_s']:.1f} s")
    print(f"  exact    {a['exact_matches']}/{a['exact_evaluable_episodes']} = "
          f"{a['exact_rate']:.4f}  median delay {a['exact_delay_median_s']:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
