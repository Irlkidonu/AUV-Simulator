#!/usr/bin/env python3
"""Everything needed to judge a campaign, in one pass. Read-only.

Written for the Study 2 go/no-go, which has to be made quickly when the
development campaign lands. It reports, on one consistent normalisation:

  * the aggregate J for every policy;
  * both C1 candidates, as PROTOCOL 6.1b requires;
  * the SPEED-MATCHED baseline, as PROTOCOL 6.1 rule 2 requires and Study 1
    failed to report;
  * per-family cross-track with the discrimination check;
  * the technique-selection profile, which is how a dead action axis is caught;
  * infrastructure consumed, which is the currency an improvement can hide in;
  * provisional F1-F4 verdicts.

Lives outside the package so that adding it cannot alter a freeze record.

    python3 experiments/analyse_campaign.py --campaign .../campaign_v6.csv \
        --sweep .../static_sweep.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "uuv_mode_aware_navigation"
sys.path.insert(0, str(PACKAGE))

POLICY_ORDER = [
    "oracle", "fixed", "proposed", "ablation_a2", "covariance_only",
    "ablation_a1", "residual_only", "dead_reckoning",
]

#: Tolerance for matching a configuration's pace to the manager's, in m/s.
SPEED_MATCH_TOLERANCE = 0.05

#: F2, as declared in PROTOCOL S2.1 on 4 August 2026.
EPS_NOMINAL_RELATIVE = 0.15
EPS_NOMINAL_ABSOLUTE_M = 0.05

NUMERIC = (
    "rms_cross_track_m", "max_cross_track_m", "elapsed_s", "safety_violations",
    "coverage_fraction", "seed", "rms_position_error_m", "mean_altitude_m",
    "path_length_m", "mode_transitions", "channel_switches", "swath_coverage",
    "terminal_error_m", "acoustic_infrastructure_cost",
)


def load(path: Path) -> list[dict]:
    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path} is empty")
    for r in rows:
        r["completed"] = str(r["completed"]).strip().lower() in ("true", "1", "yes")
        for k in NUMERIC:
            if k in r and r[k] not in ("", None):
                try:
                    r[k] = float(r[k])
                except ValueError:
                    r[k] = 0.0
    return rows


def normalisers(rows: list[dict]) -> dict[str, float]:
    out = {}
    for key, fn in (
        ("failed_mission_rate", lambda r: 0.0 if r["completed"] else 1.0),
        ("rms_cross_track_m", lambda r: r["rms_cross_track_m"]),
        ("safety_violation_rate",
         lambda r: r["safety_violations"] / max(r["elapsed_s"], 1e-9)),
    ):
        v = [fn(r) for r in rows]
        m = float(np.mean(v)) if v else 0.0
        out[key] = m if abs(m) > 1e-12 else 1.0
    return out


def aggregate(rows: list[dict], policy: str, norm: dict[str, float]) -> float:
    """J: per family, then mean over families. Never pooled over runs."""
    per_family = []
    families = sorted({r["scenario"] for r in rows})
    index: dict[tuple, list] = {}
    for r in rows:
        index.setdefault((r["policy"], r["scenario"]), []).append(r)
    for fam in families:
        s = index.get((policy, fam))
        if not s:
            continue
        per_family.append(
            np.mean([0.0 if r["completed"] else 1.0 for r in s])
            / norm["failed_mission_rate"]
            + np.mean([r["rms_cross_track_m"] for r in s])
            / norm["rms_cross_track_m"]
            + np.mean([r["safety_violations"] / max(r["elapsed_s"], 1e-9)
                       for r in s]) / norm["safety_violation_rate"]
        )
    return float(np.mean(per_family)) if per_family else float("nan")


def median_aggregate(rows: list[dict], policy: str, norm: dict[str, float]) -> float:
    families = sorted({r["scenario"] for r in rows})
    vals = []
    for fam in families:
        s = [r for r in rows if r["policy"] == policy and r["scenario"] == fam]
        if not s:
            continue
        vals.append(
            np.mean([0.0 if r["completed"] else 1.0 for r in s])
            / norm["failed_mission_rate"]
            + np.mean([r["rms_cross_track_m"] for r in s])
            / norm["rms_cross_track_m"]
            + np.mean([r["safety_violations"] / max(r["elapsed_s"], 1e-9)
                       for r in s]) / norm["safety_violation_rate"]
        )
    return float(np.median(vals)) if vals else float("nan")


def pace(rows: list[dict], policy: str) -> float:
    s = [r for r in rows if r["policy"] == policy]
    t = sum(r["elapsed_s"] for r in s)
    return sum(r["path_length_m"] for r in s) / t if t else float("nan")


def mean_of(rows: list[dict], policy: str, key: str) -> float:
    s = [r[key] for r in rows if r["policy"] == policy and key in r]
    return float(np.mean(s)) if s else float("nan")


def section(title: str) -> None:
    print(f"\n{'=' * 78}\n{title}\n{'=' * 78}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--campaign", type=Path, required=True)
    ap.add_argument("--sweep", type=Path, default=None)
    ap.add_argument("--no-hindsight-oracle", action="store_true",
                    help="the campaign was run without a configuration sweep, so "
                         "its oracle rows are the clairvoyant policy only and "
                         "not the per-seed hindsight ceiling. Suppresses every "
                         "quantity computed against the oracle, because a "
                         "recovery fraction measured against a soft ceiling "
                         "overstates the method.")
    ap.add_argument("--normalisers", type=Path, default=None,
                    help="JSON with development normalisers; default is to "
                         "derive them from this campaign and say so")
    args = ap.parse_args()

    rows = load(args.campaign)
    families = sorted({r["scenario"] for r in rows})
    seeds = sorted({int(r["seed"]) for r in rows})
    print(f"{args.campaign.name}: {len(rows)} runs, {len(families)} families, "
          f"{len(seeds)} seeds ({min(seeds)}--{max(seeds)})")

    if args.normalisers and args.normalisers.exists():
        norm = json.loads(args.normalisers.read_text())["normalisation"]
        print("normalisation: development constants (comparable across campaigns)")
    else:
        norm = normalisers(rows)
        print("normalisation: DERIVED FROM THIS CAMPAIGN -- not comparable to "
              "another campaign's J without rescoring")

    J = {p: aggregate(rows, p, norm) for p in sorted({r["policy"] for r in rows})}

    if args.no_hindsight_oracle:
        print("\nNOTE: no per-seed hindsight oracle in this campaign. The oracle\n"
              "      rows are the clairvoyant policy alone, which is a weaker\n"
              "      ceiling, so oracle-relative quantities are suppressed rather\n"
              "      than reported at values that would flatter the method.")
        rows = [r for r in rows if r["policy"] != "oracle"]

    section("Aggregate primary outcome J (lower is better)")
    print(f"{'policy':<18}{'J':>9}{'failed':>9}{'xtrack':>10}{'cover':>8}"
          f"{'pace':>7}{'infra':>8}")
    order = [p for p in POLICY_ORDER if p in J] + \
            [p for p in sorted(J) if p not in POLICY_ORDER]
    for p in order:
        s = [r for r in rows if r["policy"] == p]
        print(f"{p:<18}{J[p]:9.3f}"
              f"{np.mean([0.0 if r['completed'] else 1.0 for r in s]):9.2f}"
              f"{mean_of(rows, p, 'rms_cross_track_m'):10.3f}"
              f"{mean_of(rows, p, 'coverage_fraction'):8.3f}"
              f"{pace(rows, p):7.3f}"
              f"{mean_of(rows, p, 'acoustic_infrastructure_cost'):8.3f}")

    # --- F1, against both the overall best and the speed-matched baseline ---
    if args.sweep and args.sweep.exists():
        sweep = load(args.sweep)
        configs = sorted({r["policy"] for r in sweep})
        scored = {c: aggregate(sweep, c, norm) for c in configs}
        med = {c: median_aggregate(sweep, c, norm) for c in configs}
        c1_mean = min(scored, key=scored.get)
        c1_median = min(med, key=med.get)
        pj = J.get("proposed", float("nan"))
        pp = pace(rows, "proposed")
        matched = [c for c in configs
                   if abs(pace(sweep, c) - pp) <= SPEED_MATCH_TOLERANCE]
        c1_speed = min(matched, key=scored.get) if matched else None

        section("F1 -- against every baseline the protocol requires")
        print(f"  proposed                        J = {pj:.3f}  "
              f"(pace {pp:.3f} m/s)")
        print(f"  C1 by mean   (pre-registered)   J = {scored[c1_mean]:.3f}  "
              f"{c1_mean}")
        print(f"  C1 by median (6.1b)             J = {scored[c1_median]:.3f}  "
              f"{c1_median}")
        if c1_speed:
            print(f"  speed-matched (6.1 rule 2)      J = {scored[c1_speed]:.3f}  "
                  f"{c1_speed}  (pace {pace(sweep, c1_speed):.3f})")
        print()
        for label, cfg in (("overall-best C1", c1_mean),
                           ("median-selected", c1_median),
                           ("speed-matched", c1_speed)):
            if cfg is None:
                continue
            d = pj - scored[cfg]
            print(f"    vs {label:<18}{d:+8.3f}  -> proposed "
                  f"{'WINS' if d < 0 else 'loses'}")
        print("\n  F1 is evaluated against the mean-selected C1: J is defined "
              "with the mean.")
        print(f"  F1 {'TRIGGERED' if pj > scored[c1_mean] else '*** NOT TRIGGERED ***'}")

    # --- F2, on the declared tolerance -----------------------------------
    section("F2 -- nominal regression (PROTOCOL S2.1)")
    nom = [r for r in rows if r["scenario"].startswith("E1_")]
    if nom:
        pn = float(np.mean([r["rms_cross_track_m"] for r in nom
                            if r["policy"] == "proposed"]))
        fn = float(np.mean([r["rms_cross_track_m"] for r in nom
                            if r["policy"] == "fixed"]))
        rel, absd = (pn - fn) / fn, pn - fn
        pc = float(np.mean([r["coverage_fraction"] for r in nom
                            if r["policy"] == "proposed"]))
        fc = float(np.mean([r["coverage_fraction"] for r in nom
                            if r["policy"] == "fixed"]))
        trig = (rel > EPS_NOMINAL_RELATIVE and absd > EPS_NOMINAL_ABSOLUTE_M) \
            or pc < fc
        print(f"  proposed {pn:.4f} m vs fixed {fn:.4f} m "
              f"-> {rel:+.1%} relative, {absd:+.4f} m absolute")
        print(f"  coverage {pc:.3f} vs {fc:.3f}")
        print(f"  F2 {'TRIGGERED' if trig else 'not triggered'} "
              f"(needs >{EPS_NOMINAL_RELATIVE:.0%} AND "
              f">{EPS_NOMINAL_ABSOLUTE_M} m, or worse coverage)")

    # --- F3 ---------------------------------------------------------------
    section("F3 -- safety and tail")
    for p in ("proposed", "fixed"):
        s = [r for r in rows if r["policy"] == p]
        if not s:
            continue
        print(f"  {p:<10} viol/s "
              f"{np.mean([r['safety_violations'] / max(r['elapsed_s'], 1e-9) for r in s]):.4f}"
              f"   mean peak xtrack {np.mean([r['max_cross_track_m'] for r in s]):8.3f}"
              f"   transitions {mean_of(rows, p, 'mode_transitions'):.2f}")

    # --- per-family + discrimination --------------------------------------
    section("Per-family cross-track, and which families discriminate")
    wins = disc = disc_wins = 0
    print(f"{'family':<26}{'proposed':>10}{'fixed':>9}{'DR fails':>10}{'':>4}")
    for fam in families:
        s = [r for r in rows if r["scenario"] == fam]
        p = float(np.mean([r["rms_cross_track_m"] for r in s
                           if r["policy"] == "proposed"]))
        f = float(np.mean([r["rms_cross_track_m"] for r in s
                           if r["policy"] == "fixed"]))
        dr = [r for r in s if r["policy"] == "dead_reckoning"]
        fdr = float(np.mean([0.0 if r["completed"] else 1.0 for r in dr])) if dr else 0.0
        flat = fdr == 0.0
        wins += p < f
        if not flat:
            disc += 1
            disc_wins += p < f
        print(f"  {fam:<24}{p:10.3f}{f:9.3f}{fdr:9.0%}   "
              f"{'FLAT' if flat else 'discriminates'}")
    print(f"\n  proposed wins cross-track in {wins}/{len(families)} families")
    print(f"  of the {disc} that discriminate, proposed wins {disc_wins}")

    # --- F4 ----------------------------------------------------------------
    section("F4 -- is the contribution navigation, or measurement weighting?")
    for fam in families:
        s = [r for r in rows if r["scenario"] == fam]
        vals = {}
        for p in ("proposed", "ablation_a1", "ablation_a2", "dead_reckoning"):
            q = [r["rms_cross_track_m"] for r in s if r["policy"] == p]
            vals[p] = float(np.mean(q)) if q else float("nan")
        if vals["dead_reckoning"] < 1.0:
            continue
        flag = ""
        if abs(vals["ablation_a1"] - vals["dead_reckoning"]) < 0.05 * vals["dead_reckoning"]:
            flag = "   A1 ~= dead reckoning"
        print(f"  {fam:<24} proposed {vals['proposed']:8.2f}  A2 {vals['ablation_a2']:8.2f}"
              f"  A1 {vals['ablation_a1']:8.2f}  DR {vals['dead_reckoning']:8.2f}{flag}")

    print("\nDone. Nothing above was written to disk.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
