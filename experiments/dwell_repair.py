#!/usr/bin/env python3
"""What would the aggregate be if the terminal dwell were derived correctly?

DEVELOPMENT SEEDS ONLY. The held-out block is spent and is not touched here.

Section 5.4 of the manuscript already reports that the declared dwell of twelve
seconds sits on the unfavourable side of a cliff the data locate between eight
and ten seconds, and that reporting the pre-registered value costs roughly a
factor of seven in the compound family. That was reported and not acted on.

This script measures what acting on it would do to the aggregate. It runs only
the `proposed` policy: the dwell is a tier-3 parameter, and every other
comparator either has tier 3 disabled (`ablation_a1`, `ablation_a2`) or has no
manager at all, so their rows are unchanged and are read from the existing
development campaign.

Nothing in the frozen tree is edited. The dwell is set per instance, after
construction, so `freeze.py --verify` continues to pass and the artefact that
produced the first study's results stays byte-identical.

    python3 experiments/dwell_repair.py --dwell 6 8 12 --jobs 7
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "uuv_mode_aware_navigation"
sys.path.insert(0, str(PACKAGE))

sys.path.insert(0, str(PACKAGE / "scripts"))

from uuv_mode_aware_navigation.analysis import aggregate_outcome  # noqa: E402
from uuv_mode_aware_navigation.campaign import (  # noqa: E402
    DEVELOPMENT_SEED_ROOT,
    NoiseProfile,
    Scenario,
    run_scenario,
)
from uuv_mode_aware_navigation.comparators import ProposedPolicy  # noqa: E402

# The scenario family, the availability calibration and the optical-feedback
# calibration are defined in the campaign script rather than in the package, so
# they are imported from there. Importing it is side-effect free: everything it
# does at module scope is definitions, and main() is guarded.
from run_campaign import (  # noqa: E402
    BASELINE_CURRENT,
    calibrate,
    calibrate_optical_feedback,
    scenario_family,
)

RESULTS = PACKAGE / "results"
SEEDS = 10


def _scenarios(root: int) -> list[Scenario]:
    return [
        Scenario(
            name=entry[0],
            seed=root + 1000 + k,
            water=entry[1],
            schedule=entry[2],
            current=entry[3] if len(entry) > 3 else BASELINE_CURRENT,
            noise=entry[4] if len(entry) > 4 else NoiseProfile.constant(40.0),
        )
        for entry in scenario_family()
        for k in range(SEEDS)
    ]


_CTX: dict = {}


def _init(root: int, dwell: float) -> None:
    _CTX["model"] = calibrate(root + 1)
    _CTX["feedback"] = calibrate_optical_feedback(root + 2)
    _CTX["dwell"] = dwell


def _one(scenario: Scenario) -> dict:
    policy = ProposedPolicy(_CTX["model"])
    # The dwell lives on the manager, not on the policy wrapper. Setting it on
    # the wrapper -- which is what the first version of this script did --
    # silently creates an unused attribute and every dwell value returns
    # identical results. Asserted rather than assumed, because that failure is
    # invisible in the output unless you happen to notice the numbers repeat.
    assert hasattr(policy.manager, "blackout_timeout_s"), \
        "manager has no blackout_timeout_s; the override target moved"
    policy.manager.blackout_timeout_s = float(_CTX["dwell"])
    assert policy.manager.blackout_timeout_s == float(_CTX["dwell"])
    result = run_scenario(scenario, policy, optical_feedback=_CTX["feedback"])
    result.policy = "proposed"
    return result.to_row()


def _load(path: Path) -> list[dict]:
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r["completed"] = r["completed"].strip().lower() in ("true", "1", "yes")
        for k in ("rms_cross_track_m", "elapsed_s", "safety_violations",
                  "coverage_fraction", "max_cross_track_m"):
            r[k] = float(r[k])
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dwell", type=float, nargs="+", default=[6.0, 8.0, 12.0])
    ap.add_argument("--jobs", type=int, default=7)
    ap.add_argument("--root", type=int, default=DEVELOPMENT_SEED_ROOT)
    ap.add_argument("--out", type=Path, default=ROOT / "experiments" / "dwell_repair.csv")
    args = ap.parse_args()

    baseline = _load(RESULTS / "campaign_v5.csv")
    norms = json.load((RESULTS / "DEVELOPMENT_NORMALISERS.json").open())["normalisation"]
    others = [r for r in baseline if r["policy"] != "proposed"]
    scenarios = _scenarios(args.root)

    print(f"development seeds only, root {args.root}, {len(scenarios)} scenarios")
    print(f"comparators read from campaign_v5.csv; only `proposed` is re-run\n")

    j_ref = aggregate_outcome(baseline, normalisation=norms)
    print(f"reference (declared dwell 12 s): proposed {j_ref['proposed']:.3f}  "
          f"fixed {j_ref['fixed']:.3f}  -> F1 "
          f"{'TRIGGERED' if j_ref['proposed'] > j_ref['fixed'] else 'not triggered'}\n")

    all_rows: list[dict] = []
    for dwell in args.dwell:
        t0 = time.time()
        with Pool(args.jobs, initializer=_init, initargs=(args.root, dwell)) as pool:
            rows = pool.map(_one, scenarios, chunksize=1)
        for r in rows:
            r["dwell_s"] = dwell
        all_rows.extend(rows)

        combined = others + rows
        J = aggregate_outcome(combined, normalisation=norms)
        e7 = [r for r in rows if r["scenario"] == "E7_compound"]
        e8 = [r for r in rows if r["scenario"] == "E8_turbid_dvl_loss"]
        fired = sum(1 for r in rows if str(r.get("surfaced")).strip().lower()
                    in ("true", "1", "yes"))
        fired_e8 = sum(1 for r in e8 if str(r.get("surfaced")).strip().lower()
                       in ("true", "1", "yes"))
        wins = sum(
            1 for fam in sorted({r["scenario"] for r in rows})
            if np.mean([r["rms_cross_track_m"] for r in rows if r["scenario"] == fam])
            < np.mean([r["rms_cross_track_m"] for r in others
                       if r["scenario"] == fam and r["policy"] == "fixed"])
        )
        print(f"dwell {dwell:5.1f} s  ({time.time()-t0:5.1f} s)")
        print(f"   J proposed {J['proposed']:6.3f}   fixed {J['fixed']:6.3f}   "
              f"-> F1 {'TRIGGERED' if J['proposed'] > J['fixed'] else '*** NOT TRIGGERED ***'}")
        print(f"   E7 cross-track {np.mean([r['rms_cross_track_m'] for r in e7]):7.3f} m"
              f"   E8 {np.mean([r['rms_cross_track_m'] for r in e8]):6.3f} m")
        print(f"   terminal action fired in {fired}/{len(rows)} runs; "
              f"in E8 (must be 0): {fired_e8}")
        print(f"   families beaten on cross-track: {wins}/15\n")

    if all_rows:
        fields = sorted({k for r in all_rows for k in r})
        with args.out.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(all_rows)
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
