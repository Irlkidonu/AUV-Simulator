#!/usr/bin/env python3
"""Why PREDICTIVE never emits a pre-emptive action. Diagnostic only.

Read-only. PREDICTIVE is not modified, and no new comparison is run: this
replays the interactive recordings that already carry PREDICTIVE replays and
recomputes, per step, the exact predicate chain in ``policies.py`` that gates
``preemptive``. It counts how often each link holds.

The chain, transcribed from Study3Policy.step:

    optical_now   = usable["optical"]  < boundary
    acoustic_now  = usable["acoustic"] < boundary
    velocity_now  = usable["velocity"] < boundary
    optical_bad   = optical_now  or optical_evidence.warning
    acoustic_bad  = acoustic_now or "acoustic" in impending
    velocity_bad  = velocity_now or "velocity" in impending
    raw_trigger   = velocity_bad or (optical_bad and acoustic_bad)
    trigger       = raw_trigger and mode.fallback_required
                                and mode.mode is RELATIVE_DEAD_RECKONING
    current_loss  = velocity_now or (optical_now and acoustic_now)
    predicted_trigger = trigger and not current_loss      # <- pre-emption
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))
sys.path.insert(0, str(HERE))

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FixedConfiguration, PolicyKind, Study3Policy, run_one)
from uuv_mode_aware_navigation.study3.modes import NavigationMode  # noqa: E402
from uuv_mode_aware_navigation.study3.interactive import (  # noqa: E402
    InteractiveEnvironment, load_recording)
import interactive_test_sessions as harness  # noqa: E402

RECORDINGS = HERE / "interactive_sessions"


def _probe():
    class Probe(Study3Policy):
        rows = []

        def step(self, observation):
            action, output = super().step(observation)
            usable = output.belief.usable_probability
            boundary = self.fixed.usable_probability_boundary
            mode = self.last_mode_decision
            warning = bool(getattr(self.last_optical_evidence_forecast, "warning", False))
            impending = set(output.forecast.impending)
            optical_now = usable["optical"] < boundary
            acoustic_now = usable["acoustic"] < boundary
            velocity_now = usable["velocity"] < boundary
            optical_bad = optical_now or warning
            acoustic_bad = acoustic_now or "acoustic" in impending
            velocity_bad = velocity_now or "velocity" in impending
            raw = velocity_bad or (optical_bad and acoustic_bad)
            in_dr = mode.mode is NavigationMode.RELATIVE_DEAD_RECKONING
            trigger = bool(raw and mode.fallback_required and in_dr)
            current_loss = velocity_now or (optical_now and acoustic_now)
            self.__class__.rows.append({
                "time_s": observation.time_s,
                "forecast_nonempty": bool(impending),
                "optical_warning": warning,
                "raw_trigger": bool(raw),
                "in_dead_reckoning": in_dr,
                "fallback_required": bool(mode.fallback_required),
                "trigger": trigger,
                "current_loss": bool(current_loss),
                "predicted_trigger": bool(trigger and not current_loss),
                "action_preemptive": bool(action.preemptive),
            })
            return action, output

    Probe.rows = []
    return Probe


def main():
    counts = Counter()
    total = 0
    per_case = {}
    for name in sorted(harness.SESSION_INDEX):
        path = RECORDINGS / f"{name}.json"
        if not path.exists():
            continue
        index = harness.SESSION_INDEX[name]
        record, base = load_recording(path)
        environment = InteractiveEnvironment(base, replay_events=record["events"],
                                             pace=False)
        Probe = _probe()
        run_one(34_100_000 + index, base.config.name, 0, PolicyKind.PREDICTIVE,
                FixedConfiguration(optical_channel="lidar", altitude_m=5.,
                                   speed_mps=.5, acoustic_technique="lbl",
                                   fusion_mode="weight"),
                horizon_s=record["horizon_s"], dt_s=record["dt_s"],
                image_period_s=4., keep_trace=True, redesign_version=3,
                policy_factory=Probe, environment_realization=environment)
        rows = Probe.rows
        total += len(rows)
        case = Counter()
        for row in rows:
            for key in ("forecast_nonempty", "optical_warning", "raw_trigger",
                        "in_dead_reckoning", "fallback_required", "trigger",
                        "current_loss", "predicted_trigger", "action_preemptive"):
                if row[key]:
                    counts[key] += 1
                    case[key] += 1
        per_case[name] = dict(case)

    # The decisive question: among steps where the chain reaches `trigger`,
    # how many are already a current loss (so pre-emption is excluded)?
    report = {"schema": "study3_predictive_diagnostic_v1",
              "classification": "READ_ONLY_DIAGNOSTIC",
              "policy_modified": False,
              "steps": total, "counts": dict(counts), "by_case": per_case}
    path = HERE / "redesign_results/predictive_diagnostic.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    print(f"PREDICTIVE steps examined: {total}\n")
    order = ("forecast_nonempty", "optical_warning", "raw_trigger",
             "in_dead_reckoning", "fallback_required", "trigger",
             "current_loss", "predicted_trigger", "action_preemptive")
    for key in order:
        print(f"  {key:20s} {counts[key]:6d}  {counts[key]/total:6.2%}")
    print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
