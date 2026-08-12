#!/usr/bin/env python3
"""Read-only consistency check for Study 3 development and held-out separation."""

from __future__ import annotations

import json
from pathlib import Path


HERE=Path(__file__).resolve().parent


def main():
    seeds=json.loads((HERE/"STUDY3_SEED_REGISTRY.json").read_text())
    design=json.loads((HERE/"STUDY3_DESIGN.json").read_text())
    development_roots={root for values in seeds["development_attempt_roots"].values() for root in values}
    heldout=seeds["held_out"]["final_reserved_unexecuted"]
    assert heldout not in development_roots
    assert not development_roots&set(seeds["forbidden_roots"])
    assert all(31_000_000<=root<32_000_000 for root in development_roots)
    primary=len(design["primary_families"])*30*len(design["policies"])
    controls=len(design["control_families"])*20*len(design["policies"])
    assert primary==design["heldout_runs"]["primary"]==840
    assert controls==design["heldout_runs"]["controls"]==240
    assert primary+controls==design["heldout_runs"]["total"]==1080
    selected=sum(design["development_runs"][k] for k in
                 ("scenario_calibration","fixed_successive_halving","adaptive_tuning","development_confirmation"))
    assert selected==design["development_runs"]["selected_path_total"]==4160
    assert not design["heldout_runs"]["executed"]
    forbidden_outputs=("heldout_result.json","campaign_complete.json","heldout_attempt.json")
    assert not any((HERE/name).exists() for name in forbidden_outputs)
    print("Study 3 design PASS: redesign roots registered; selected development path 4160 / proposed 1080 held-out; held-out unexecuted and unauthorized")
    return 0


if __name__=="__main__":raise SystemExit(main())
