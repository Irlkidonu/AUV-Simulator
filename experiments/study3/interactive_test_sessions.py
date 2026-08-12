#!/usr/bin/env python3
"""Interactive Study 3 control-window testing harness.

Acts as a **tester**, not a developer. Nothing in the controller, thresholds,
scenarios or existing evidence is modified. This script only drives the
interactive environment and records what happens.

Disturbances are injected through ``InteractiveEnvironment.set_control`` -- the
same call ``control_window.py`` makes when a slider moves or a toggle flips. The
GUI is not scripted with synthetic clicks: driving the API directly is what makes
an event sequence reproducible, which is the whole point of a recording.

The eight schedules below were authored in full before any session was run, and
no schedule was revised after seeing an outcome.

Each session is recorded once under REACTIVE, saved as a checksummed recording,
then replayed unchanged against DEPLOYMENT_FIXED, REACTIVE and PREDICTIVE. The
REACTIVE replay must reproduce the live run's ``trace_digest``; if it does not,
the harness is not observing what it recorded and the run is reported as such.
"""
from __future__ import annotations

import argparse
import json
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import PolicyKind  # noqa: E402
from uuv_mode_aware_navigation.study3.interactive import (  # noqa: E402
    load_recording, run_interactive_session, save_recording)
from uuv_mode_aware_navigation.study3.scenarios import PhysicalState  # noqa: E402
from uuv_mode_aware_navigation.study3.transition_driver import (  # noqa: E402
    truth_side_best_viable_mode)

OUT = HERE / "interactive_sessions"
SEED_BASE = 34_000_000
ROOT_BASE = 34_100_000
HORIZON_S = 600.0
DT_S = 1.0
REPLAY_POLICIES = ("deployment_fixed", "reactive", "predictive")

#: Window used only to *describe* a mode change that reverts. Reported as a
#: descriptive statistic alongside raw counts; it is not an acceptance
#: threshold and nothing is judged against it.
REVERSION_WINDOW_S = 15.0

#: (time_s, control, value). Authored blind; never revised after a result.
SCHEDULES = {
 "S1_optical_degrade_recover": [
    (60, "turbidity", .28), (95, "turbidity", .44), (130, "turbidity", .62),
    (175, "turbidity", .81), (240, "optical_failure", True),
    (300, "optical_failure", False), (300, "turbidity", .55),
    (355, "turbidity", .30), (410, "turbidity", .12), (470, "turbidity", .08)],
 "S2_dvl_bottom_then_water": [
    (70, "dvl_bottom_probability", .55), (110, "dvl_bottom_probability", .22),
    (150, "dvl_bottom_probability", .04), (235, "dvl_water_probability", .40),
    (290, "dvl_water_probability", .06), (380, "dvl_bottom_probability", .85),
    (430, "dvl_water_probability", .88), (500, "dvl_noise_scale", 3.5)],
 "S3_currents_building_and_veering": [
    (55, "current_east_mps", .06), (100, "current_east_mps", .14),
    (145, "current_north_mps", -.11), (200, "current_east_mps", .24),
    (260, "current_north_mps", -.22), (330, "current_east_mps", -.18),
    (400, "current_north_mps", .15), (480, "current_east_mps", .02),
    (480, "current_north_mps", 0.)],
 "S4_acoustic_noise_and_lbl_geometry": [
    (65, "acoustic_noise_db", 56.), (120, "lbl_geometry_scale", .62),
    (165, "acoustic_noise_db", 66.), (220, "lbl_geometry_scale", .28),
    (275, "acoustic_noise_db", 74.), (340, "acoustic_noise_db", 61.),
    (395, "lbl_geometry_scale", .85), (455, "acoustic_noise_db", 49.),
    (520, "lbl_geometry_scale", 1.)],
 "S5_lbl_loss_then_usbl_departure": [
    (80, "lbl_available", False), (140, "turbidity", .20),
    (190, "usbl_available", False), (300, "usbl_available", True),
    (420, "lbl_available", True)],
 "S6_usbl_departure_under_turbidity": [
    (50, "turbidity", .30), (90, "usbl_available", False),
    (140, "turbidity", .58), (210, "lbl_geometry_scale", .35),
    (270, "turbidity", .86), (340, "dvl_bottom_probability", .30),
    (430, "turbidity", .25), (500, "usbl_available", True)],
 "S7_compound_optical_dvl_then_acoustic": [
    (75, "turbidity", .70), (120, "dvl_crashout", True),
    (190, "acoustic_noise_db", 70.), (250, "lbl_available", False),
    (330, "acoustic_failure", True), (420, "acoustic_failure", False),
    (420, "lbl_available", True), (480, "dvl_crashout", False),
    (480, "turbidity", .15)],
 # S3b exists because S3 aborts on the world-texture limit (see the results
 # record). Its currents reverse so net drift stays inside the map while the
 # magnitudes stay high. Chosen for the map bound, not for any policy outcome,
 # and authored before it was run.
 "S3b_currents_reversing_in_map": [
    (40, "current_east_mps", .18), (85, "current_east_mps", -.20),
    (130, "current_north_mps", .16), (175, "current_north_mps", -.18),
    (220, "current_east_mps", .22), (240, "dvl_water_probability", .35),
    (265, "current_east_mps", -.22), (310, "current_north_mps", .20),
    (355, "current_north_mps", -.20), (400, "current_east_mps", .12),
    (400, "current_north_mps", .12), (450, "current_east_mps", -.14),
    (450, "current_north_mps", -.14), (510, "current_east_mps", 0.),
    (510, "current_north_mps", 0.)],
 # S9 covers recovery under compound failure. S7 and S8 commit to surfacing,
 # which ends the mission, so their later recovery events never fire; this one
 # recovers before any terminal commitment. Also authored before it was run.
 "S9_compound_recover_before_terminal": [
    (70, "turbidity", .62), (110, "dvl_bottom_probability", .20),
    (150, "lbl_geometry_scale", .30), (190, "acoustic_noise_db", 68.),
    (230, "turbidity", .28), (270, "dvl_bottom_probability", .90),
    (320, "acoustic_noise_db", 52.), (360, "lbl_geometry_scale", .95),
    (420, "turbidity", .75), (470, "usbl_available", False),
    (520, "turbidity", .15), (520, "usbl_available", True)],
 "S8_total_loss_to_surfacing": [
    (90, "turbidity", .55), (140, "dvl_bottom_probability", .05),
    (185, "lbl_available", False), (230, "usbl_available", False),
    (275, "optical_failure", True), (310, "dvl_water_probability", .03),
    (460, "optical_failure", False), (460, "turbidity", .10),
    (460, "dvl_bottom_probability", .95), (460, "dvl_water_probability", .90),
    (460, "lbl_available", True), (460, "usbl_available", True)],
}


#: Fixed seed offset per session, so adding a schedule never renumbers an
#: existing recording. Original eight keep the offsets they were recorded with.
SESSION_INDEX = {
    "S1_optical_degrade_recover": 0,
    "S2_dvl_bottom_then_water": 1,
    "S3_currents_building_and_veering": 2,
    "S4_acoustic_noise_and_lbl_geometry": 3,
    "S5_lbl_loss_then_usbl_departure": 4,
    "S6_usbl_departure_under_turbidity": 5,
    "S7_compound_optical_dvl_then_acoustic": 6,
    "S8_total_loss_to_surfacing": 7,
    "S3b_currents_reversing_in_map": 8,
    "S9_compound_recover_before_terminal": 9,
}


def _collect(telemetry, truth, completion):
    """Derive comparison quantities. Uses frozen result metrics where they exist."""
    result = completion.get("result") or {}
    modes = [(p["time_s"], p["navigation_mode"]) for p in telemetry]
    changes = [(t, m) for i, (t, m) in enumerate(modes)
               if i and m != modes[i - 1][1]]
    reversions = sum(1 for i in range(1, len(changes))
                     if i + 1 < len(changes)
                     and changes[i + 1][1] == changes[i - 1][1]
                     and changes[i + 1][0] - changes[i][0] <= REVERSION_WINDOW_S)
    errors = [p["horizontal_error_m"] for p in telemetry]
    surfacing = next((p["time_s"] for p in telemetry if p["terminal_or_surfacing"]), None)

    # Adaptation episodes, V5 C7/C8 definition, over the truth-side sequence.
    best = [truth_side_best_viable_mode(PhysicalState(**s["physical"])) for s in truth]
    selected = {p["time_s"]: p["navigation_mode"] for p in telemetry}
    times = [s["time_s"] for s in truth]
    episodes, matched, latencies = 0, 0, []
    start = 1
    while start < len(best):
        end = start
        while end + 1 < len(best) and best[end + 1] == best[start]:
            end += 1
        if best[start] != best[start - 1] and (end - start + 1) * DT_S >= 6.0:
            episodes += 1
            hit = next((times[p] - times[start] for p in range(start, end + 1)
                        if selected.get(times[p]) == best[start]), None)
            if hit is not None:
                matched += 1
                latencies.append(hit)
        start = end + 1

    return {
        "status": completion.get("status"),
        "trace_digest": result.get("trace_digest"),
        "completed": result.get("completed"),
        "safety_violation": result.get("safety_violation"),
        "overall_rmse_m": result.get("overall_rmse_m"),
        "rmse_transition_m": result.get("rmse_transition_m"),
        "peak_error_m": result.get("peak_error_m"),
        "unaided_time_s": result.get("unaided_time_s"),
        "longest_unaided_gap_s": result.get("longest_unaided_gap_s"),
        "survey_coverage_fraction": result.get("survey_coverage_fraction"),
        "physical_interventions": result.get("physical_interventions"),
        "mode_switches": result.get("mode_switches"),
        "telemetry_samples": len(telemetry),
        "telemetry_rmse_m": (st.mean(e ** 2 for e in errors) ** .5) if errors else None,
        "telemetry_peak_error_m": max(errors) if errors else None,
        "mode_changes": len(changes),
        "distinct_modes": sorted({m for _, m in modes}),
        "mode_sequence": [{"time_s": t, "mode": m} for t, m in changes],
        "reversions_within_window": reversions,
        "surfacing_time_s": surfacing,
        "adaptation_episodes": episodes,
        "adaptation_matched": matched,
        "adaptation_coverage": (matched / episodes) if episodes else None,
        "adaptation_median_latency_s": st.median(latencies) if latencies else None,
        "adaptation_max_latency_s": max(latencies) if latencies else None,
    }


def _session(policy, seed, root, *, schedule=None, replay=None):
    telemetry, truth, environment_box = [], [], {}

    def on_environment(environment):
        environment_box["environment"] = environment
        environment.on_physical_state = truth.append

    def on_telemetry(packet):
        telemetry.append(packet)
        if schedule is None:
            return
        environment = environment_box["environment"]
        # Inject exactly as an operator moving a control would, in band with the
        # running simulation. effective_step is current_step + 1 inside set_control.
        while schedule and packet["time_s"] >= schedule[0][0]:
            _, control, value = schedule.pop(0)
            environment.set_control(control, value)

    environment, completion = run_interactive_session(
        policy_kind=PolicyKind(policy), seed=seed, root=root, index=0,
        horizon_s=HORIZON_S, dt_s=DT_S, pace=False, replay_record=replay,
        on_environment=on_environment, on_telemetry=on_telemetry)
    return environment, completion, telemetry, truth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", default=None, help="run a single named session")
    arguments = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    names = [arguments.only] if arguments.only else list(SCHEDULES)
    report = {"schema": "study3_interactive_test_v1",
              "classification": "INTERACTIVE_TESTING_NOT_CAMPAIGN_EVIDENCE",
              "horizon_s": HORIZON_S, "dt_s": DT_S,
              "reversion_window_s": REVERSION_WINDOW_S,
              "reversion_window_note": "descriptive only; not an acceptance threshold",
              "replay_policies": list(REPLAY_POLICIES), "sessions": {}}

    for name in names:
        seed, root = SEED_BASE + SESSION_INDEX[name], ROOT_BASE + SESSION_INDEX[name]
        schedule = sorted(SCHEDULES[name])
        print(f"\n=== {name}  seed {seed} root {root} ===")
        environment, completion, telemetry, truth = _session(
            "reactive", seed, root, schedule=list(schedule))
        recording_path = OUT / f"{name}.json"
        record = save_recording(recording_path, environment, "reactive",
                                root=root, index=0)
        live = _collect(telemetry, truth, completion)
        print(f"  recorded {len(record['events'])} events, "
              f"status {live['status']}, modes {live['mode_changes']}")

        entry = {"seed": seed, "root": root,
                 "recording": recording_path.name,
                 "recording_sha256": record["sha256"],
                 "events": record["events"],
                 "declared_schedule": [{"time_s": t, "control": c, "value": v}
                                       for t, c, v in schedule],
                 "live_reactive": live, "replays": {}}

        for policy in REPLAY_POLICIES:
            _, replay_completion, replay_telemetry, replay_truth = _session(
                policy, seed, root, replay=str(recording_path))
            entry["replays"][policy] = _collect(replay_telemetry, replay_truth,
                                                replay_completion)
            print(f"  replay {policy:17s} status {entry['replays'][policy]['status']:9s} "
                  f"rmse {entry['replays'][policy]['overall_rmse_m']}")

        reactive_replay = entry["replays"]["reactive"]
        entry["replay_reproduces_live_reactive"] = bool(
            reactive_replay["trace_digest"] and
            reactive_replay["trace_digest"] == live["trace_digest"])
        print(f"  REACTIVE replay reproduces live run: "
              f"{entry['replay_reproduces_live_reactive']}")
        report["sessions"][name] = entry

    path = OUT / "interactive_test_results.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=True) + "\n")
    print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
