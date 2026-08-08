#!/usr/bin/env python3
"""Turn the held-out CSV into the numbers Section 5.10 needs. Read-only.

This lives outside the package on purpose. `scripts/*.py` is covered by the
freeze record, so adding an analysis script there after the freeze would make
`freeze.py --verify` report the tree as modified -- which is exactly the signal
that must stay meaningful. Analysis of a result is not part of the source that
produced it, so it belongs here.

    python3 experiments/analyse_held_out.py \
        --held-out src/uuv_mode_aware_navigation/results/held_out.csv \
        --development src/uuv_mode_aware_navigation/results/campaign_v5.csv

Nothing here writes to the campaign outputs, and nothing here re-runs anything.

## The one subtlety, stated up front

`aggregate_outcome` normalises its three components by their pooled mean across
all policies. Its docstring says the constants come from development data and
are never re-derived on the held-out set; the campaign script calls it without
passing them, so on the held-out set they *are* re-derived. That is not a thumb
on the scale -- the normaliser is symmetric across policies, so it cannot favour
one -- but it does mean a held-out `J` and a development `J` are not on the same
ruler, and the paper puts them side by side.

The tree was frozen before this was noticed and the campaign was already
running, so the code was not changed. Instead both are reported:

  * `J_self`  -- normalised on held-out data, which is what the campaign log
                 prints, and what internal comparisons between policies on the
                 held-out set should use;
  * `J_dev`   -- the same runs scored with the development normalisers, which is
                 the only version comparable to the numbers already in the paper.

Any claim of the form "held-out J moved relative to development J" must use
`J_dev`. Section 5.10 says which it is using.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "uuv_mode_aware_navigation"
sys.path.insert(0, str(PACKAGE))

from uuv_mode_aware_navigation.analysis import (  # noqa: E402
    DEFAULT_WEIGHTS,
    aggregate_outcome,
    paired_difference,
)

# Reported in the order the manuscript discusses them, not alphabetically.
POLICY_ORDER = [
    "oracle", "fixed", "proposed", "ablation_a2", "covariance_only",
    "ablation_a1", "residual_only", "dead_reckoning",
]


def load(path: Path) -> list[dict]:
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"{path} is empty")
    for row in rows:
        row["completed"] = row["completed"].strip().lower() in ("true", "1", "yes")
        for key in ("rms_cross_track_m", "max_cross_track_m", "elapsed_s",
                    "safety_violations", "coverage_fraction", "seed",
                    "rms_position_error_m", "mean_altitude_m",
                    "surfacing_commanded", "surfaced", "mode_transitions"):
            if key in row and row[key] not in ("", None):
                try:
                    row[key] = float(row[key])
                except ValueError:
                    row[key] = float(row[key].strip().lower() in ("true", "yes"))
    return rows


def development_normalisers(rows: list[dict]) -> dict[str, float]:
    """Recover the constants `aggregate_outcome` would derive from these rows.

    Reimplemented rather than imported because the function computes them
    internally and does not return them. The arithmetic is copied from
    `analysis.aggregate_outcome` and is checked against it below.
    """
    pooled: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        pooled["failed_mission_rate"].append(0.0 if row["completed"] else 1.0)
        pooled["rms_cross_track_m"].append(row["rms_cross_track_m"])
        pooled["safety_violation_rate"].append(
            row["safety_violations"] / max(row["elapsed_s"], 1e-9)
        )
    out = {}
    for key in DEFAULT_WEIGHTS:
        scale = float(np.mean(pooled[key])) if pooled[key] else 0.0
        out[key] = scale if abs(scale) > 1e-12 else 1.0
    return out


def _check_normalisers(rows: list[dict], norms: dict[str, float]) -> None:
    """Passing the derived constants must reproduce the self-normalised result.

    If this fails, the reimplementation above has drifted from the function it
    mirrors, and every `J_dev` below would be quietly wrong.
    """
    a = aggregate_outcome(rows)
    b = aggregate_outcome(rows, normalisation=norms)
    for policy in a:
        assert abs(a[policy] - b[policy]) < 1e-9, (
            f"normaliser reimplementation disagrees for {policy}: "
            f"{a[policy]} vs {b[policy]}"
        )


def _rate(rows: list[dict], predicate) -> float:
    return float(np.mean([1.0 if predicate(r) else 0.0 for r in rows])) if rows else float("nan")


def section(title: str) -> None:
    print(f"\n{'=' * 78}\n{title}\n{'=' * 78}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--held-out", type=Path, required=True)
    ap.add_argument("--development", type=Path, required=True)
    ap.add_argument("--sweep", type=Path, default=None,
                    help="held-out static sweep, for the PROTOCOL 6.1b "
                         "both-C1-candidates report")
    args = ap.parse_args()

    held = load(args.held_out)
    dev = load(args.development)

    families = sorted({r["scenario"] for r in held})
    seeds = sorted({int(r["seed"]) for r in held})
    print(f"held-out : {len(held)} runs, {len(families)} families, "
          f"{len(seeds)} seeds, seed range {min(seeds)}--{max(seeds)}")
    print(f"developed: {len(dev)} runs, "
          f"{len({r['scenario'] for r in dev})} families, "
          f"{len({int(r['seed']) for r in dev})} seeds")

    dev_seeds = {int(r["seed"]) for r in dev}
    overlap = dev_seeds & set(seeds)
    print(f"seed overlap between the two sets: {len(overlap)}"
          f"{' -- DISJOINT' if not overlap else '  *** NOT DISJOINT ***'}")

    # --- the aggregate, both ways ---------------------------------------
    norms = development_normalisers(dev)
    _check_normalisers(dev, norms)

    j_self = aggregate_outcome(held)
    j_dev = aggregate_outcome(held, normalisation=norms)
    j_development = aggregate_outcome(dev)

    section("Aggregate primary outcome J (lower is better)")
    print("  J_self : held-out runs, normalised on held-out data (campaign log)")
    print("  J_dev  : held-out runs, development normalisers -- comparable to the paper")
    print(f"\n{'policy':<18}{'J_self':>10}{'J_dev':>10}"
          f"{'J (dev campaign)':>20}{'change':>10}")
    order = [p for p in POLICY_ORDER if p in j_self] + \
            [p for p in sorted(j_self) if p not in POLICY_ORDER]
    for policy in order:
        was = j_development.get(policy, float("nan"))
        now = j_dev[policy]
        print(f"{policy:<18}{j_self[policy]:10.3f}{now:10.3f}{was:20.3f}"
              f"{now - was:+10.3f}")

    # --- Both C1 candidates, as PROTOCOL 6.1b requires ------------------
    # The rule fixed on 31 July, before the campaign that would settle it:
    # J keeps the mean, both candidates are reported, and the proposed method is
    # evaluated against each. Discharged here on held-out data as well as on
    # development, because the rule does not restrict itself to development.
    if args.sweep is not None and args.sweep.exists():
        section("Both C1 candidates on the held-out sweep (PROTOCOL 6.1b)")
        sweep = load(args.sweep)

        def family_scores(rows: list[dict], policy: str) -> list[float]:
            out = []
            for fam in sorted({r["scenario"] for r in rows}):
                s = [r for r in rows if r["policy"] == policy and r["scenario"] == fam]
                if not s:
                    continue
                out.append(
                    np.mean([0.0 if r["completed"] else 1.0 for r in s])
                    / norms["failed_mission_rate"]
                    + np.mean([r["rms_cross_track_m"] for r in s])
                    / norms["rms_cross_track_m"]
                    + np.mean([r["safety_violations"] / max(r["elapsed_s"], 1e-9)
                               for r in s]) / norms["safety_violation_rate"]
                )
            return out

        configs = sorted({r["policy"] for r in sweep})
        scored = {c: family_scores(sweep, c) for c in configs}
        by_mean = {c: float(np.mean(v)) for c, v in scored.items() if v}
        by_median = {c: float(np.median(v)) for c, v in scored.items() if v}
        c1_mean = min(by_mean, key=by_mean.get)
        c1_median = min(by_median, key=by_median.get)
        proposed_J = float(np.mean(family_scores(held, "proposed")))

        print(f"  configurations swept: {len(configs)}")
        print(f"  C1 by MEAN   (pre-registered): {c1_mean}")
        print(f"      J = {by_mean[c1_mean]:.3f}")
        print(f"  C1 by MEDIAN                 : {c1_median}")
        print(f"      J = {by_mean[c1_median]:.3f} (scored on the mean aggregate)")
        print(f"  proposed                     : J = {proposed_J:.3f}")
        for label, cfg in (("mean-selected", c1_mean), ("median-selected", c1_median)):
            gap = proposed_J - by_mean[cfg]
            print(f"    proposed vs {label:<16} {gap:+.3f} -> proposed "
                  f"{'WINS' if gap < 0 else 'loses'}")
        rank_mean_under_median = sorted(by_median, key=by_median.get).index(c1_mean) + 1
        rank_median_under_mean = sorted(by_mean, key=by_mean.get).index(c1_median) + 1
        print(f"  mean-selected ranks {rank_mean_under_median}/{len(by_mean)} under the median")
        print(f"  median-selected ranks {rank_median_under_mean}/{len(by_mean)} under the mean")
        print("  NOTE: J is defined with the mean, so the mean-selected "
              "configuration IS C1 and F1 is evaluated against it.")
    else:
        section("Both C1 candidates -- SKIPPED (no --sweep given)")
        print("  PROTOCOL 6.1b requires both candidates to be reported. Pass "
              "--sweep to discharge it on held-out data.")

    # --- F1: the predeclared primary claim ------------------------------
    section("F1 -- does the manager beat the tuned fixed policy on J?")
    for label, table in (("J_self", j_self), ("J_dev", j_dev)):
        if "proposed" in table and "fixed" in table:
            gap = table["proposed"] - table["fixed"]
            verdict = "NOT triggered (proposed wins)" if gap < 0 else "TRIGGERED"
            print(f"  {label}: proposed {table['proposed']:.3f} vs fixed "
                  f"{table['fixed']:.3f}  -> {gap:+.3f}  -> F1 {verdict}")

    # --- F4: is the contribution a navigation one? ----------------------
    section("F4 -- is tier 1 alone indistinguishable from dead reckoning?")
    print("  (the decisive test: if A1 matches the full manager, the result is "
          "a measurement-weighting result)")
    for family in families:
        subset = [r for r in held if r["scenario"] == family]
        vals = {}
        for policy in ("proposed", "ablation_a1", "dead_reckoning", "fixed"):
            sel = [r["rms_cross_track_m"] for r in subset if r["policy"] == policy]
            vals[policy] = float(np.mean(sel)) if sel else float("nan")
        flags = ""
        if vals["dead_reckoning"] > 0:
            if abs(vals["ablation_a1"] - vals["dead_reckoning"]) < 0.05 * vals["dead_reckoning"]:
                flags = "  A1 ~= dead reckoning"
        print(f"  {family:<26} proposed {vals['proposed']:8.2f}  "
              f"A1 {vals['ablation_a1']:8.2f}  DR {vals['dead_reckoning']:8.2f}"
              f"  fixed {vals['fixed']:8.2f}{flags}")

    print()
    for metric in ("rms_cross_track_m", "coverage_fraction"):
        for other in ("ablation_a1", "fixed", "dead_reckoning"):
            # Argument order is (rows, metric, method, reference): the sign is
            # method minus reference, so negative favours `proposed` on
            # cross-track and disfavours it on coverage.
            d = paired_difference(held, metric, "proposed", other)
            print(f"  paired proposed - {other:<15} on {metric:<20}"
                  f" {d.mean_difference:+8.3f}  [{d.lower:+.3f}, {d.upper:+.3f}]"
                  f"{'  (interval spans zero)' if d.lower <= 0 <= d.upper else ''}")

    # --- F3: was any gain bought with unsafety? -------------------------
    section("F3 -- safety violations and tail cross-track")
    print(f"{'policy':<18}{'viol/s':>12}{'tail xtrack':>14}{'coverage':>11}"
          f"{'elapsed s':>12}")
    for policy in order:
        sel = [r for r in held if r["policy"] == policy]
        if not sel:
            continue
        viol = float(np.mean([r["safety_violations"] / max(r["elapsed_s"], 1e-9)
                              for r in sel]))
        tail = float(np.percentile([r["max_cross_track_m"] for r in sel], 95))
        cov = float(np.mean([r["coverage_fraction"] for r in sel]))
        el = float(np.mean([r["elapsed_s"] for r in sel]))
        print(f"{policy:<18}{viol:12.5f}{tail:14.2f}{cov:11.3f}{el:12.1f}")

    # --- discrimination: which families can separate anything? ----------
    section("Which families discriminate?")
    discriminating = []
    for family in families:
        subset = [r for r in held if r["scenario"] == family]
        f_dr = _rate([r for r in subset if r["policy"] == "dead_reckoning"],
                     lambda r: not r["completed"])
        f_fx = _rate([r for r in subset if r["policy"] == "fixed"],
                     lambda r: not r["completed"])
        flat = f_dr == 0.0
        if not flat:
            discriminating.append(family)
        print(f"  {family:<26} DR fails {f_dr:5.0%}  fixed fails {f_fx:5.0%}"
              f"   {'FLAT' if flat else 'discriminates'}")
    print(f"\n  {len(discriminating)} of {len(families)} families discriminate: "
          f"{discriminating}")

    # --- per-family cross-track: the '14 of 15' style count -------------
    section("Per-family mean RMS cross-track, proposed vs fixed")
    wins = 0
    for family in families:
        subset = [r for r in held if r["scenario"] == family]
        p = float(np.mean([r["rms_cross_track_m"] for r in subset
                           if r["policy"] == "proposed"]))
        f = float(np.mean([r["rms_cross_track_m"] for r in subset
                           if r["policy"] == "fixed"]))
        better = p < f
        wins += better
        print(f"  {family:<26} proposed {p:8.3f}  fixed {f:8.3f}  "
              f"{'proposed' if better else 'fixed'}")
    print(f"\n  proposed wins cross-track in {wins} of {len(families)} families")
    print(f"  of the {len(discriminating)} that discriminate, proposed wins "
          f"{sum(1 for fam in discriminating if np.mean([r['rms_cross_track_m'] for r in held if r['scenario'] == fam and r['policy'] == 'proposed']) < np.mean([r['rms_cross_track_m'] for r in held if r['scenario'] == fam and r['policy'] == 'fixed']))}")

    # --- terminal action ------------------------------------------------
    section("Terminal action (surface for GPS)")
    for policy in ("proposed", "ablation_a2", "fixed"):
        sel = [r for r in held if r["policy"] == policy]
        if not sel:
            continue
        cmd = sum(1 for r in sel if r.get("surfacing_commanded"))
        did = sum(1 for r in sel if r.get("surfaced"))
        print(f"  {policy:<18} commanded {cmd:4d} / {len(sel)} runs, "
              f"surfaced {did:4d}")
    for family in families:
        sel = [r for r in held if r["scenario"] == family and r["policy"] == "proposed"]
        cmd = sum(1 for r in sel if r.get("surfacing_commanded"))
        if cmd:
            errs = [r["rms_position_error_m"] for r in sel]
            print(f"    {family:<24} fired in {cmd}/{len(sel)}  "
                  f"mean position error {float(np.mean(errs)):.3f} m")

    # --- bracket recovery ------------------------------------------------
    section("Oracle recovery per family (pooling this ratio is invalid)")
    ratios = []
    for family in families:
        subset = [r for r in held if r["scenario"] == family]
        def mean_of(policy: str) -> float:
            sel = [r["rms_cross_track_m"] for r in subset if r["policy"] == policy]
            return float(np.mean(sel)) if sel else float("nan")
        fx, pr, orc = mean_of("fixed"), mean_of("proposed"), mean_of("oracle")
        span = fx - orc
        if abs(span) < 1e-9:
            print(f"  {family:<26} degenerate bracket (fixed == oracle)")
            continue
        recovery = (fx - pr) / span
        ratios.append(recovery)
        print(f"  {family:<26} fixed {fx:8.3f}  proposed {pr:8.3f}  "
              f"oracle {orc:8.3f}   recovery {recovery:+.2f}")
    if ratios:
        print(f"\n  mean over non-degenerate families: {float(np.mean(ratios)):+.2f}")

    print("\nDone. Nothing above was written to disk.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
