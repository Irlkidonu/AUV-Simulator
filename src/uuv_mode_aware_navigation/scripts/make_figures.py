#!/usr/bin/env python3
"""Generate the manuscript figures from a campaign CSV.

Figures F2, F3, F4, and F6 of EVALUATION_METRICS_SPEC.md section 6. F4 (bracket
recovery) and the A1 panel of F6 are the two a sceptical reviewer will look for,
and neither may be cut.

Usage::

    PYTHONPATH=. python3 scripts/make_figures.py results/development.csv
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from uuv_mode_aware_navigation.analysis import (  # noqa: E402
    oracle_recovery,
    oracle_recovery_report,
    paired_difference,
    summarise,
)

# Order and labels as the manuscript reports them. The oracle is labelled an
# oracle everywhere it appears (fairness rule R3).
LABELS = {
    "proposed": "Proposed",
    "fixed": "Fixed policy (C1)",
    "residual_only": "Residual-only (C2)",
    "covariance_only": "Covariance-only (C3)",
    "dead_reckoning": "Dead reckoning (C4)",
    "oracle": "ORACLE (C5)",
    "ablation_a1": "A1: no actions",
    "ablation_a2": "A2: no mission actions",
}
ORDER = list(LABELS)


def _save(fig, out: Path) -> None:
    """Write each figure as vector PDF and as raster PNG.

    The PDF is what the manuscript includes: it stays sharp at any zoom and is
    what MDPI asks for. The PNG is kept alongside for quick inspection and for
    contexts that cannot display vector art, at 300 dpi rather than 200, which
    is the floor the publisher sets for raster submissions.
    """
    out = Path(out)
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(out.with_suffix(".png"), dpi=300, bbox_inches="tight")
    plt.close(fig)


def load(path: Path) -> list[dict]:
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key, value in list(row.items()):
            if key in ("scenario", "policy"):
                continue
            if value in ("", "None"):
                row[key] = float("nan")
                continue
            if value in ("True", "False"):
                row[key] = value == "True"
                continue
            try:
                row[key] = float(value)
            except ValueError:
                pass
    return rows


def figure_primary_panel(rows, out: Path) -> None:
    """F2: the three primary metrics side by side, per policy.

    Drawn together deliberately. Two of the three were, in an earlier version of
    the evaluator, identically zero for every method across 400 runs -- and that
    was invisible in a figure showing only cross-track error. Plotting all three
    makes a dead metric impossible to miss.
    """
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 4.0))
    panels = [
        ("failed", "Failed-mission rate (P1)", "#d62728"),
        ("coverage_fraction", "Survey coverage (higher better)", "#2ca02c"),
        ("rms_cross_track_m", "RMS cross-track (m) (P2)", "#1f77b4"),
    ]
    for ax, (metric, title, colour) in zip(axes, panels):
        if metric == "failed":
            values = {
                p: 1.0 - v
                for p, v in summarise(
                    [{**r, "f": 1.0 if r["completed"] else 0.0} for r in rows], "f"
                ).items()
            }
        else:
            values = summarise(rows, metric)
        present = [p for p in ORDER if p in values]
        ax.barh(range(len(present)), [values[p] for p in present], color=colour)
        ax.set_yticks(range(len(present)))
        ax.set_yticklabels([LABELS[p] for p in present], fontsize=7)
        ax.set_title(title, fontsize=9)
        ax.invert_yaxis()
        spread = max(values.values()) - min(values.values())
        if spread < 1e-9:
            ax.text(
                0.5, 0.5, "IDENTICAL FOR EVERY METHOD\nthis metric carries no information",
                transform=ax.transAxes, ha="center", va="center",
                fontsize=9, color="#d62728", fontweight="bold",
            )
    fig.suptitle("Primary metrics (EVALUATION_METRICS_SPEC §1)", fontsize=10)
    fig.tight_layout()
    _save(fig, out)
    plt.close(fig)


def figure_by_scenario(rows, out: Path) -> None:
    """F3: primary outcome by scenario, with the C1/C5 bracket marked."""
    scenarios = sorted({r["scenario"] for r in rows})
    fig, axes = plt.subplots(
        1, len(scenarios), figsize=(3.1 * len(scenarios), 4.2), sharey=True
    )
    axes = np.atleast_1d(axes)
    for ax, scenario in zip(axes, scenarios):
        subset = [r for r in rows if r["scenario"] == scenario]
        means = summarise(subset, "rms_cross_track_m")
        present = [p for p in ORDER if p in means]
        values = [means[p] for p in present]
        colours = [
            "#1f77b4" if p == "proposed"
            else "#d62728" if p == "oracle"
            else "#7f7f7f"
            for p in present
        ]
        ax.barh(range(len(present)), values, color=colours)
        ax.set_yticks(range(len(present)))
        ax.set_yticklabels([LABELS[p] for p in present], fontsize=7)
        if "fixed" in means and "oracle" in means:
            ax.axvline(means["fixed"], ls="--", lw=1, color="#7f7f7f")
            ax.axvline(means["oracle"], ls="--", lw=1, color="#d62728")
        ax.set_title(scenario, fontsize=9)
        ax.set_xlabel("RMS cross-track (m)", fontsize=8)
        ax.invert_yaxis()
    fig.suptitle(
        "Mission-level outcome by scenario (dashed: fixed-policy and oracle bracket)",
        fontsize=10,
    )
    fig.tight_layout()
    _save(fig, out)
    plt.close(fig)


#: Categorical slots, in fixed order. Validated for colour-vision deficiency:
#: every pair clears an OKLab dE of 8 under deuteranopia, protanopia and
#: tritanopia, and 15 under normal vision. The tightest pair is proposed against
#: oracle under deuteranopia at 9.9, which clears the floor but not comfortably,
#: so marker shape carries identity as well as hue and no series is
#: distinguishable by colour alone.
SERIES = {
    "fixed": ("#2a78d6", "s", "Fixed C1 (hindsight-tuned)"),
    "proposed": ("#eb6834", "o", "Proposed"),
    "oracle": ("#1baf7a", "D", "Oracle C5 (not deployable)"),
}
GRID = "#d8d8d4"
INK = "#3a3a38"
MUTED = "#6f6f6a"


def _family_label(name: str) -> str:
    """`E10_current_steady` -> `E10 current steady`."""
    head, _, tail = name.partition("_")
    return f"{head} {tail.replace('_', ' ')}" if tail else head


def _family_order(name: str) -> int:
    m = re.match(r"E(\d+)", name)
    return int(m.group(1)) if m else 999


def figure_bracket(rows, out: Path) -> None:
    """F4: where the proposed method sits between the fixed policy and the oracle.

    Split into two panels rather than one. Cross-track error spans four orders
    of magnitude across the failure matrix -- a few centimetres where dead
    reckoning completes unaided, tens of metres where it cannot -- so a single
    linear axis compresses every recoverable family into an unreadable stripe
    against zero, and a log axis would misrepresent differences of a millimetre
    as visually equal to differences of a metre.

    The split is the one the study already draws: families in which dead
    reckoning fails at least one run, and families in which it does not. That is
    a property of the scenario rather than of any result, so the panelling
    cannot be accused of being chosen to suit the outcome.

    Recovery is annotated per family, which is the only valid way to form it.
    Families whose bracket is degenerate -- where the oracle's privileged
    information bought nothing -- are drawn and labelled rather than dropped.
    """
    report = oracle_recovery_report(rows)
    families = sorted({r["scenario"] for r in rows}, key=_family_order)

    def fails_dead_reckoning(fam: str) -> bool:
        dr = [r for r in rows
              if r["scenario"] == fam and r["policy"] == "dead_reckoning"]
        return any(not r["completed"] for r in dr)

    discriminating = [f for f in families if fails_dead_reckoning(f)]
    flat = [f for f in families if not fails_dead_reckoning(f)]
    panels = [(flat, "Dead reckoning completes every run"),
              (discriminating, "Dead reckoning fails at least one run")]
    panels = [(fams, title) for fams, title in panels if fams]

    heights = [max(len(f), 1) for f, _ in panels]
    fig, axes = plt.subplots(
        len(panels), 1, figsize=(7.2, 0.34 * sum(heights) + 1.6),
        gridspec_kw={"height_ratios": heights},
    )
    axes = np.atleast_1d(axes)

    for ax, (fams, title) in zip(axes, panels):
        for i, fam in enumerate(fams):
            subset = [r for r in rows if r["scenario"] == fam]
            means = summarise(subset, "rms_cross_track_m")
            if not all(k in means for k in SERIES):
                continue
            lo, hi = sorted((means["fixed"], means["oracle"]))
            ax.plot([lo, hi], [i, i], color=GRID, lw=5,
                    solid_capstyle="round", zorder=1)
            for key, (colour, marker, label) in SERIES.items():
                ax.plot(means[key], i, marker, color=colour,
                        ms=8 if marker != "o" else 9,
                        markeredgecolor="white", markeredgewidth=1.0,
                        zorder=3, label=label if (ax is axes[0] and i == 0) else "")
            if fam in report["degenerate"]:
                ax.annotate("bracket degenerate", (means["proposed"], i),
                            textcoords="offset points", xytext=(9, 0),
                            va="center", fontsize=7, color=MUTED)
            elif fam in report["per_scenario"]:
                ax.annotate(f"{report['per_scenario'][fam]:.0%}",
                            (means["proposed"], i), textcoords="offset points",
                            xytext=(0, 9), ha="center", fontsize=7, color=MUTED)
        ax.set_yticks(range(len(fams)))
        ax.set_yticklabels([_family_label(f) for f in fams], fontsize=8)
        ax.set_ylim(len(fams) - 0.5, -0.5)
        ax.set_title(title, fontsize=8.5, color=MUTED, loc="left", pad=6)
        ax.grid(axis="x", color=GRID, lw=0.6, zorder=0)
        ax.set_axisbelow(True)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.spines["bottom"].set_color(GRID)
        ax.tick_params(colors=INK, length=0)
        ax.margins(x=0.12)

    axes[-1].set_xlabel("RMS cross-track error (m)", fontsize=9, color=INK)
    axes[0].legend(fontsize=8, loc="lower right", frameon=False,
                   labelcolor=INK)
    fig.tight_layout()
    _save(fig, out)


def figure_ablations(rows, out: Path) -> None:
    """F6: the ablation ladder, which is the evidence F4 is evaluated on.

    Three panels rather than two, because the second panel previously plotted
    mean altitude in metres beside mission time rescaled to hundreds of seconds,
    on a shared axis. Two measures of different units on one scale is a
    dual-axis chart in disguise: the visual comparison between the bars is an
    artefact of the rescaling factor, and a reader cannot recover the real
    relationship. Each measure now has its own axis and its own panel.
    """
    means = summarise(rows, "rms_cross_track_m")
    ladder = [k for k in ("dead_reckoning", "ablation_a1", "ablation_a2",
                          "proposed", "fixed") if k in means]

    fig, axes = plt.subplots(1, 3, figsize=(9.6, 3.4))
    ax1, ax2, ax3 = axes

    # Panel 1: the ladder. One measure, one axis, ordered worst to best.
    bars = ax1.bar(range(len(ladder)), [means[k] for k in ladder],
                   color=[SERIES["proposed"][0] if k == "proposed"
                          else SERIES["fixed"][0] if k == "fixed"
                          else "#b9b8b2" for k in ladder],
                   width=0.68)
    ax1.set_xticks(range(len(ladder)))
    ax1.set_xticklabels([LABELS[k] for k in ladder], fontsize=7,
                        rotation=25, ha="right")
    ax1.set_ylabel("RMS cross-track (m)", fontsize=9, color=INK)
    ax1.set_title("Tier ablation", fontsize=9, color=MUTED, loc="left")
    ax2.set_title("Mission cost", fontsize=9, color=MUTED, loc="left")
    for b, k in zip(bars, ladder):
        ax1.annotate(f"{means[k]:.2f}", (b.get_x() + b.get_width() / 2,
                     b.get_height()), textcoords="offset points",
                     xytext=(0, 3), ha="center", fontsize=7, color=MUTED)

    # Panels 2 and 3: one measure each, native units, no rescaling.
    for ax, metric, label in (
        (ax2, "mean_altitude_m", "Mean altitude (m)"),
        (ax3, "elapsed_s", "Mission time (s)"),
    ):
        vals = summarise(rows, metric)
        keys = [k for k in ("proposed", "fixed") if k in vals]
        b = ax.bar(range(len(keys)), [vals[k] for k in keys],
                   color=[SERIES[k][0] for k in keys], width=0.5)
        ax.set_xticks(range(len(keys)))
        ax.set_xticklabels([LABELS[k] for k in keys], fontsize=8)
        ax.set_ylabel(label, fontsize=9, color=INK)
        for bar, k in zip(b, keys):
            ax.annotate(f"{vals[k]:.2f}", (bar.get_x() + bar.get_width() / 2,
                        bar.get_height()), textcoords="offset points",
                        xytext=(0, 3), ha="center", fontsize=7, color=MUTED)

    for ax in axes:
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.tick_params(colors=INK, length=0)

    fig.tight_layout()
    _save(fig, out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=str)
    parser.add_argument("--outdir", type=str, default="results/figures")
    args = parser.parse_args()

    rows = load(Path(args.csv))
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    figure_primary_panel(rows, outdir / "F2_primary_metrics.png")
    figure_by_scenario(rows, outdir / "F3_outcomes_by_scenario.png")
    figure_bracket(rows, outdir / "F4_bracket_recovery.png")
    figure_ablations(rows, outdir / "F6_ablations.png")

    print(f"figures written to {outdir}")
    report = oracle_recovery_report(rows)
    for scenario, value in sorted(report["per_scenario"].items()):
        print(f"  recovery {scenario:<20}{value:6.2f}")
    for scenario in report["degenerate"]:
        print(f"  recovery {scenario:<20}    -- degenerate bracket")
    for metric in ("rms_cross_track_m", "coverage_fraction"):
        for reference in ("fixed", "covariance_only", "ablation_a1"):
            try:
                print("  " + paired_difference(
                    rows, metric, "proposed", reference
                ).describe())
            except ValueError:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
