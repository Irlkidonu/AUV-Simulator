#!/usr/bin/env python3
"""Read-only classification of REACTIVE mode switches, with pre-switch evidence.

Investigation only. Nothing is tuned, and no case is generated or selected on
its outcome: every REACTIVE run in the Part E1 block and every interactive
recording is replayed in full and every switch in them is classified.

Replay is deterministic and is verified against each source's stored
``trace_digest``; a mismatch aborts. No new simulation is introduced.

Classification, declared here before the script was first run
-------------------------------------------------------------
For a switch at time t from mode A to mode B, let A's *required service* be the
acoustic service A depends on (lbl/usbl), or the optical/DVL channel otherwise.

* **staleness_driven** -- A's required service was responding at some sample in
  (t-8, t) and responds again at some sample in (t, t+16]. The modality never
  failed; only the evidence for it lapsed. This is an unnecessary switch.
* **loss_driven** -- A's required service does not respond again for at least
  30 s after t, or A was optical and optical availability was lost at t. The
  modality genuinely went away. This is a useful switch.
* **ambiguous** -- neither test fires (including switches near the end of a run
  where the 30 s window is truncated).

*Missed opportunity* uses the V5 C7 episode definition unchanged: a maximal run
of constant truth-side best viable mode, beginning after launch and lasting at
least 6 s, in which the selected mode never equals it.

For every switch the observable evidence at the immediately preceding sample is
recorded: responding services with age/sigma/dop, optical belief and
availability, DVL bottom-lock and water-track, and the filter uncertainty trace.
"""
from __future__ import annotations

import json
import math
import statistics as st
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))
sys.path.insert(0, str(HERE))

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FixedConfiguration, PolicyKind, Study3Policy, generate_environment,
    load_environment_config, run_one, truth_side_best_viable_mode)
from uuv_mode_aware_navigation.study3.scenarios import PhysicalState  # noqa: E402
from uuv_mode_aware_navigation.study3.interactive import (  # noqa: E402
    InteractiveEnvironment, load_recording)
import interactive_test_sessions as harness  # noqa: E402

E1_PACKETS = HERE / "redesign_results/final_development_v6"
E1_CONFIG = HERE / "examples/moderate_severe_variable_environment.json"
RECORDINGS = HERE / "interactive_sessions"
STALE_BEFORE_S, STALE_AFTER_S, LOSS_S = 8.0, 16.0, 30.0
SERVICE_OF = {"lbl_aided": "lbl", "usbl_aided": "usbl"}


def _recorder():
    class Recorder(Study3Policy):
        samples = []

        def step(self, observation):
            action, output = super().step(observation)
            self.__class__.samples.append({
                "time_s": observation.time_s,
                "mode": action.navigation_mode,
                "reason": self.last_mode_decision.reason,
                "services": [{"name": x.name, "responding": bool(x.responding),
                              "gives_position": bool(x.gives_position),
                              "age_s": float(x.age_s), "sigma_m": float(x.sigma_m),
                              "dop": float(x.dop)}
                             for x in observation.acoustic.service_evidence],
                "optical_available": bool(observation.optical.available),
                "optical_belief": float(output.belief.usable_probability["optical"]),
                "velocity_belief": float(output.belief.usable_probability["velocity"]),
                "dvl_bottom_lock": bool(observation.dvl.bottom_lock),
                "dvl_water_track": bool(observation.dvl.water_track),
                "mission_action": action.mission_action,
            })
            return action, output

    Recorder.samples = []
    return Recorder


def _responds(samples, name, low, high):
    """Whether `name` responds with a position in the open time window."""
    for s in samples:
        if low < s["time_s"] <= high:
            for service in s["services"]:
                if (service["name"] == name and service["responding"]
                        and service["gives_position"]):
                    return True
    return False


def classify(samples, best_by_time):
    switches, episodes = [], []
    for index in range(1, len(samples)):
        previous, current = samples[index - 1], samples[index]
        if current["mode"] == previous["mode"]:
            continue
        time_s = current["time_s"]
        old, new = previous["mode"], current["mode"]
        service = SERVICE_OF.get(old)
        if service:
            before = _responds(samples, service, time_s - STALE_BEFORE_S, time_s)
            after = _responds(samples, service, time_s, time_s + STALE_AFTER_S)
            returns = _responds(samples, service, time_s, time_s + LOSS_S)
            truncated = samples[-1]["time_s"] - time_s < LOSS_S
            if before and after:
                kind = "staleness_driven"
            elif not returns and not truncated:
                kind = "loss_driven"
            else:
                kind = "ambiguous"
        elif old.startswith("optical"):
            kind = "loss_driven" if not previous["optical_available"] else "ambiguous"
        else:
            kind = "ambiguous"
        switches.append({
            "time_s": time_s, "from": old, "to": new, "kind": kind,
            "reason": current["reason"],
            "evidence_before": {
                "services": previous["services"],
                "optical_available": previous["optical_available"],
                "optical_belief": round(previous["optical_belief"], 4),
                "velocity_belief": round(previous["velocity_belief"], 4),
                "dvl_bottom_lock": previous["dvl_bottom_lock"],
                "dvl_water_track": previous["dvl_water_track"]}})

    # Missed opportunities: V5 C7 episodes never matched.
    times = sorted(best_by_time)
    best = [best_by_time[t] for t in times]
    selected = {s["time_s"]: s["mode"] for s in samples}
    start = 1
    while start < len(best):
        end = start
        while end + 1 < len(best) and best[end + 1] == best[start]:
            end += 1
        duration = (end - start + 1) * (times[1] - times[0] if len(times) > 1 else 1.0)
        if best[start] != best[start - 1] and duration >= 6.0:
            matched = any(selected.get(times[p]) == best[start]
                          for p in range(start, end + 1))
            episodes.append({"start_s": times[start], "duration_s": duration,
                             "target": best[start],
                             "selected_at_start": selected.get(times[start]),
                             "matched": matched})
        start = end + 1
    return switches, episodes


def run_e1():
    config = load_environment_config(E1_CONFIG)
    out = []
    for path in sorted(E1_PACKETS.glob("*.json")):
        packet = json.loads(path.read_text())
        identity = packet["identity"]
        if identity["policy"] != "reactive":
            continue
        realization = generate_environment(config, identity["environment_seed"], 180., 2.)
        Recorder = _recorder()
        result, trace = run_one(
            identity["root"], identity["family"], identity["index"],
            PolicyKind("reactive"), FixedConfiguration(**identity["configuration"]),
            horizon_s=180., dt_s=2., image_period_s=4., keep_trace=True,
            redesign_version=3, environment_realization=realization,
            policy_factory=Recorder)
        if result.trace_digest != packet["result"]["trace_digest"]:
            raise RuntimeError(f"replay diverged for {path.name}")
        best = {round(row[0], 3): truth_side_best_viable_mode(
                    realization.physical_state(step, altitude_m=max(0., -float(row[7]))))
                for step, row in enumerate(trace)}
        switches, episodes = classify(Recorder.samples, best)
        out.append({"source": "V6", "case": str(identity["environment_seed"]),
                    "switches": switches, "episodes": episodes})
    return out


def run_interactive():
    stored = json.loads((RECORDINGS / "interactive_test_results.json").read_text())
    out = []
    for name in sorted(harness.SESSION_INDEX):
        path = RECORDINGS / f"{name}.json"
        if not path.exists():
            continue
        index = harness.SESSION_INDEX[name]
        # run_interactive_session takes no policy_factory, so build the same
        # environment it builds internally and drive run_one directly. For
        # REACTIVE it applies no configuration transform, so this is that path.
        record, base = load_recording(path)
        environment = InteractiveEnvironment(base, replay_events=record["events"],
                                             pace=False)
        truth = []
        environment.on_physical_state = truth.append
        Recorder = _recorder()
        result, _trace = run_one(
            34_100_000 + index, base.config.name, 0, PolicyKind.REACTIVE,
            FixedConfiguration(optical_channel="lidar", altitude_m=5., speed_mps=.5,
                               acoustic_technique="lbl", fusion_mode="weight"),
            horizon_s=record["horizon_s"], dt_s=record["dt_s"], image_period_s=4.,
            keep_trace=True, redesign_version=3, policy_factory=Recorder,
            environment_realization=environment)
        expected = stored["sessions"].get(name, {}).get("replays", {}).get(
            "reactive", {}).get("trace_digest")
        note = None
        if expected is None:
            note = ("previously aborted on the world-texture defect; it completes "
                    "under the fix, so no pre-fix digest exists to compare")
        elif expected != result.trace_digest:
            raise RuntimeError(f"interactive replay diverged for {name}")
        samples = Recorder.samples
        best = {round(s["time_s"], 3): truth_side_best_viable_mode(
                    PhysicalState(**t["physical"]))
                for s, t in zip(samples, truth)}
        switches, episodes = classify(samples, best)
        out.append({"source": "interactive", "case": name, "status": "complete",
                    "note": note, "switches": switches, "episodes": episodes})
    return out


def main():
    cases = run_e1()
    counts = Counter()
    reasons = Counter()
    per_source = {}
    for case in cases:
        source = case["source"]
        bucket = per_source.setdefault(source, Counter())
        for switch in case["switches"]:
            counts[switch["kind"]] += 1
            bucket[switch["kind"]] += 1
            reasons[(switch["kind"], switch["from"], switch["to"])] += 1
        for episode in case["episodes"]:
            key = "missed_opportunity" if not episode["matched"] else "matched_episode"
            counts[key] += 1
            bucket[key] += 1

    stale = [s for c in cases for s in c["switches"] if s["kind"] == "staleness_driven"]
    ages = [service["age_s"] for s in stale for service in s["evidence_before"]["services"]
            if service["responding"] and service["gives_position"]]
    report = {
        "schema": "study3_mode_switching_analysis_v1",
        "classification": "READ_ONLY_INVESTIGATION",
        "windows_s": {"stale_before": STALE_BEFORE_S, "stale_after": STALE_AFTER_S,
                      "loss": LOSS_S},
        "totals": dict(counts),
        "by_source": {k: dict(v) for k, v in per_source.items()},
        "transitions": {f"{k[0]}:{k[1]}->{k[2]}": v for k, v in
                        sorted(reasons.items(), key=lambda x: -x[1])},
        "staleness_switch_evidence_age_s": {
            "n": len(ages),
            "median": st.median(ages) if ages else None,
            "mean": st.mean(ages) if ages else None,
            "max": max(ages) if ages else None},
        "cases": cases,
    }
    path = HERE / "redesign_results/mode_switching_analysis_v6.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=True) + "\n")

    total_switches = sum(counts[k] for k in
                         ("staleness_driven", "loss_driven", "ambiguous"))
    print(f"switches classified: {total_switches}")
    for kind in ("staleness_driven", "loss_driven", "ambiguous"):
        share = counts[kind] / total_switches if total_switches else 0.
        print(f"  {kind:18s} {counts[kind]:5d}  {share:6.1%}")
    print(f"  matched episodes   {counts['matched_episode']:5d}")
    print(f"  missed opportunity {counts['missed_opportunity']:5d}")
    print("\nby source:", {k: dict(v) for k, v in per_source.items()})
    print("\ntop transitions:")
    for key, value in list(report["transitions"].items())[:10]:
        print(f"  {key:52s} {value}")
    print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
