#!/usr/bin/env python3
"""Generate paired uncertainty and Pareto reports from an existing CSV."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from uuv_mode_aware_navigation.analysis import (
    aggregate_outcome,
    frontier_report,
    paired_difference,
    survey_productivity,
)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def build_report(
    rows,
    reference: str,
    methods: list[str],
    metrics: list[str],
    sweep_rows=None,
) -> dict:
    report = {
        "aggregate_J": aggregate_outcome(rows),
        "survey_productivity_m2ps": survey_productivity(rows),
        "paired": [],
    }
    for method in methods:
        for metric in metrics:
            comparison = paired_difference(rows, metric, method, reference)
            report["paired"].append({
                "metric": comparison.metric,
                "method": comparison.method,
                "reference": comparison.reference,
                "n": comparison.n,
                "mean_difference": comparison.mean_difference,
                "confidence_95": [comparison.lower, comparison.upper],
                "higher_is_better": comparison.higher_is_better,
                "interval_excludes_zero": comparison.significant,
            })
    if sweep_rows is not None:
        report["pareto"] = frontier_report(sweep_rows, rows)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--reference", default="fixed")
    parser.add_argument("--method", action="append", dest="methods", required=True)
    parser.add_argument(
        "--metric", action="append", dest="metrics",
        default=["rms_cross_track_m"],
    )
    parser.add_argument("--sweep", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = build_report(
        read_rows(args.input),
        reference=args.reference,
        methods=args.methods,
        metrics=args.metrics,
        sweep_rows=read_rows(args.sweep) if args.sweep else None,
    )
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(payload)
    else:
        print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

