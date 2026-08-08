#!/usr/bin/env python3
"""Part B: the configuration sweep over the held-out scenarios.

PROTOCOL S2.5, declared before either part of the held-out campaign was run,
specifies a two-part execution. Part A flew the comparators against the
development-selected baseline and produced every quantity F1--F4 are judged on.
This is Part B: all 144 static configurations over the same held-out scenarios,
which yields two things Part A could not --

  * the configuration a hindsight sweep of the held-out block would have
    selected, reported as a SUPPLEMENTARY comparison, and
  * the per-seed hindsight ceiling used for the bracket figure.

Why this is a separate tool rather than a flag on the campaign runner
--------------------------------------------------------------------

The runner's held-out gate refuses a second execution once the block is spent,
and that refusal is correct: it is what stops a method being re-evaluated after
its author has seen how it did. Part A spent the block. Adding an exemption to
that gate would have meant weakening the one mechanism protecting the result, so
it was not added.

Instead this script does the sweep outside the gate and says so plainly. What
makes that legitimate is not a code path but a property of the work: **a static
sweep flies no adaptive policy.** It cannot tune, retune or reselect anything
about the method, because the method is not in it. Every configuration here is a
fixed point in the action space, flown open-loop.

What this cannot do, and does not
---------------------------------

F1's verdict is bound to Part A. If the sweep reveals a baseline stronger than
the development-selected one, that comparison is reported *in addition* and
labelled supplementary. The comparator campaign is never re-run against it.
Choosing which baseline to be judged by, after seeing the result, is the failure
the whole protocol exists to prevent, and no amount of convenience justifies it.

    python3 experiments/heldout_sweep.py --jobs 6
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "uuv_mode_aware_navigation"
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "scripts"))

from uuv_mode_aware_navigation.campaign import (  # noqa: E402
    HELDOUT_SEED_ROOT,
    NoiseProfile,
    Scenario,
    TerrainProfile,
    run_scenario,
)
from uuv_mode_aware_navigation.comparators import FixedPolicy  # noqa: E402
from uuv_mode_aware_navigation.manager import DEFAULT_CANDIDATES  # noqa: E402

from run_campaign import (  # noqa: E402
    BASELINE_CURRENT,
    calibrate_optical_feedback,
    scenario_family,
)

SEEDS = 20
CTX: dict = {}


def _scenarios() -> list[Scenario]:
    out = []
    for entry in scenario_family():
        for k in range(SEEDS):
            out.append(Scenario(
                name=entry[0],
                seed=HELDOUT_SEED_ROOT + 1000 + k,
                water=entry[1],
                schedule=entry[2],
                current=entry[3] if len(entry) > 3 else BASELINE_CURRENT,
                noise=entry[4] if len(entry) > 4 else NoiseProfile.constant(40.0),
                terrain=entry[5] if len(entry) > 5 else TerrainProfile.constant(0.12),
                prior_map=entry[6] if len(entry) > 6 else True,
            ))
    return out


def _init() -> None:
    # Same calibration seed the held-out comparator run used, so the two parts
    # describe one campaign rather than two.
    CTX["feedback"] = calibrate_optical_feedback(HELDOUT_SEED_ROOT + 2)


def _one(task):
    scenario, config = task
    result = run_scenario(
        scenario, FixedPolicy(config), optical_feedback=CTX["feedback"]
    )
    result.policy = config.name
    return result.to_row()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--out", type=Path,
                    default=PACKAGE / "results" / "static_sweep_held_out_2.csv")
    args = ap.parse_args()

    scenarios = _scenarios()
    tasks = [(s, c) for s in scenarios for c in DEFAULT_CANDIDATES]
    print(f"Part B: {len(DEFAULT_CANDIDATES)} configurations x "
          f"{len(scenarios)} held-out scenarios = {len(tasks)} runs")
    print("static configurations only; no adaptive policy is flown")
    print(f"seed root {HELDOUT_SEED_ROOT}, {SEEDS} seeds per family\n")

    t0 = time.time()
    done = 0
    rows = []
    with Pool(args.jobs, initializer=_init) as pool:
        for row in pool.imap_unordered(_one, tasks, chunksize=4):
            rows.append(row)
            done += 1
            if done % 2000 == 0:
                el = (time.time() - t0) / 60.0
                rate = done / max(el, 1e-9)
                print(f"  {done}/{len(tasks)} ({done/len(tasks):.1%}), "
                      f"{el:.0f} min elapsed, "
                      f"~{(len(tasks)-done)/max(rate,1e-9):.0f} min remaining",
                      flush=True)

    rows.sort(key=lambda r: (r["scenario"], r["seed"], r["policy"]))
    fields = sorted({k for r in rows for k in r})
    with args.out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {len(rows)} rows to {args.out}")
    print(f"elapsed {(time.time()-t0)/3600:.1f} h")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
