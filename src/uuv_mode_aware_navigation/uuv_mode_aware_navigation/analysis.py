"""Aggregation and statistics (EVALUATION_METRICS_SPEC.md section 5).

Paired by construction
----------------------
For a fixed seed every policy receives an identical measurement realisation, so
comparisons are **paired per seed**. Paired differences are reported rather than
independent-sample tests, and uncertainty comes from a bootstrap over those
differences.

What this module deliberately does not do
-----------------------------------------
It does not select seeds, drop outliers, or search for a favourable subset. All
held-out seeds are reported. The aggregate outcome ``J`` and its weights are
fixed before held-out execution and are inputs here, not choices made after
seeing results.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence

import numpy as np

__all__ = [
    "oracle_recovery_report",
    "survey_productivity",
    "pareto_frontier",
    "frontier_report",
    "PairedComparison",
    "paired_difference",
    "bootstrap_interval",
    "oracle_recovery",
    "aggregate_outcome",
    "summarise",
    "write_csv",
]

#: Weights for the aggregate primary outcome J. Fixed at freeze, before any
#: held-out execution. Normalisation constants come from development data only.
DEFAULT_WEIGHTS = {
    "failed_mission_rate": 1.0,
    "rms_cross_track_m": 1.0,
    "safety_violation_rate": 1.0,
}

#: Metrics for which a larger value is the better outcome. Everything else in
#: this module assumes lower is better.
HIGHER_IS_BETTER = frozenset({"coverage_fraction", "swath_coverage"})


@dataclass(frozen=True)
class PairedComparison:
    """One method against a reference, paired per seed."""

    metric: str
    method: str
    reference: str
    n: int
    mean_difference: float
    lower: float
    upper: float
    higher_is_better: bool = False

    @property
    def significant(self) -> bool:
        """Whether the bootstrap interval excludes zero."""
        return (self.lower > 0.0) or (self.upper < 0.0)

    def describe(self) -> str:
        favourable = (
            self.mean_difference > 0
            if self.higher_is_better
            else self.mean_difference < 0
        )
        direction = "better" if favourable else "worse"
        marker = "" if self.significant else "  (interval spans zero)"
        return (
            f"{self.method} vs {self.reference} on {self.metric}: "
            f"{self.mean_difference:+.3f} [{self.lower:+.3f}, {self.upper:+.3f}] "
            f"-- {direction}{marker}"
        )


def bootstrap_interval(
    values: Sequence[float],
    confidence: float = 0.95,
    resamples: int = 10_000,
    seed: int = 20_900_001,
) -> tuple[float, float]:
    """Percentile bootstrap interval. Deterministic given ``seed``."""
    data = np.asarray(list(values), dtype=float)
    if data.size == 0:
        return (float("nan"), float("nan"))
    if data.size == 1:
        return (float(data[0]), float(data[0]))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, data.size, size=(resamples, data.size))
    means = data[idx].mean(axis=1)
    alpha = (1.0 - confidence) / 2.0
    return (
        float(np.quantile(means, alpha)),
        float(np.quantile(means, 1.0 - alpha)),
    )


def paired_difference(
    rows: Sequence[Mapping],
    metric: str,
    method: str,
    reference: str,
    confidence: float = 0.95,
) -> PairedComparison:
    """Per-seed difference ``method - reference`` on ``metric``.

    Negative means the method is better for metrics where lower is better.
    """
    by_seed: dict[tuple, dict[str, float]] = {}
    for row in rows:
        key = (row["scenario"], row["seed"])
        by_seed.setdefault(key, {})[row["policy"]] = float(row[metric])

    diffs = [
        v[method] - v[reference]
        for v in by_seed.values()
        if method in v and reference in v
    ]
    if not diffs:
        raise ValueError(f"no paired observations for {method} vs {reference}")
    lower, upper = bootstrap_interval(diffs, confidence)
    return PairedComparison(
        metric=metric,
        method=method,
        reference=reference,
        n=len(diffs),
        mean_difference=float(np.mean(diffs)),
        lower=lower,
        upper=upper,
        higher_is_better=metric in HIGHER_IS_BETTER,
    )


#: A bracket narrower than this (in the metric's units) carries no information:
#: the oracle's privileged knowledge bought it nothing, so there is no headroom
#: to recover and the ratio is undefined rather than large.
MINIMUM_BRACKET = 1e-3


def oracle_recovery(
    rows: Sequence[Mapping],
    metric: str = "rms_cross_track_m",
    method: str = "proposed",
    fixed: str = "fixed",
    oracle: str = "oracle",
) -> Optional[float]:
    """Fraction of the oracle's benefit that automatic inference recovers.

    ``(fixed - method) / (fixed - oracle)``, computed **per scenario** and then
    averaged over the scenarios where the bracket is non-degenerate.

    Computing it on pooled means instead is invalid, and not subtly so. It is a
    ratio of differences of means across heterogeneous conditions: scenarios
    where the oracle is *worse* than the fixed policy contribute a negative
    denominator, which flips the sign of the whole ratio. In the 10-seed
    development campaign that pooling produced a recovery of 2.12 and a spurious
    "the method beat the oracle" alarm, while every individual scenario was
    correctly bracketed.

    Scenarios with a degenerate bracket -- where perfect information changes
    nothing -- are excluded and counted separately by
    :func:`oracle_recovery_report`, never silently folded in.

    Publishing this fraction is the strongest available answer to "did you
    cripple the baseline?": it shows how much headroom existed and that the
    method did not claim all of it. A per-scenario value above 1.0 means the
    method beat an oracle running the same policy with perfect information,
    which is evidence of a defect to be investigated and reported -- never a
    result.
    """
    report = oracle_recovery_report(rows, metric, method, fixed, oracle)
    return report.get("mean")


def oracle_recovery_report(
    rows: Sequence[Mapping],
    metric: str = "rms_cross_track_m",
    method: str = "proposed",
    fixed: str = "fixed",
    oracle: str = "oracle",
) -> dict:
    """Per-scenario recovery plus the bookkeeping the manuscript must report."""
    per_scenario: dict[str, float] = {}
    degenerate: list[str] = []
    for scenario in sorted({r["scenario"] for r in rows}):
        subset = [r for r in rows if r["scenario"] == scenario]
        means = summarise(subset, metric)
        if not all(k in means for k in (method, fixed, oracle)):
            continue
        bracket = means[fixed] - means[oracle]
        if bracket <= MINIMUM_BRACKET:
            degenerate.append(scenario)
            continue
        per_scenario[scenario] = (means[fixed] - means[method]) / bracket

    out: dict = {"per_scenario": per_scenario, "degenerate": degenerate}
    if per_scenario:
        out["mean"] = float(np.mean(list(per_scenario.values())))
    return out


def summarise(rows: Sequence[Mapping], metric: str) -> dict[str, float]:
    """Mean of ``metric`` per policy, across all scenarios and seeds."""
    grouped: dict[str, list[float]] = {}
    for row in rows:
        grouped.setdefault(row["policy"], []).append(float(row[metric]))
    return {k: float(np.mean(v)) for k, v in grouped.items()}


def aggregate_outcome(
    rows: Sequence[Mapping],
    weights: Mapping[str, float] = DEFAULT_WEIGHTS,
    normalisation: Optional[Mapping[str, float]] = None,
) -> dict[str, float]:
    """The predeclared aggregate primary outcome ``J``, per policy.

    ``J = w1*norm(P1) + w2*norm(P2) + w3*norm(P3)`` over failed-mission rate,
    RMS cross-track error, and safety-violation rate. Normalisation constants are
    computed from development data and passed in; they are never re-derived from
    the held-out set.

    Each scenario family contributes equally (PROTOCOL.md section 6.1: the
    weighting must make no family dominate, and the compound family must not be
    weighted below any single-fault family). Pooling runs instead lets the three
    single-fault families -- in which, correctly, nothing needs managing -- outvote
    the compound family three to one, and dilutes a real effect below the noise
    floor. That is a defect of the aggregation, not of the method.
    """
    per_policy: dict[str, dict[str, list[float]]] = {}
    for row in rows:
        bucket = per_policy.setdefault(row["policy"], {})
        bucket.setdefault("failed_mission_rate", []).append(
            0.0 if row["completed"] else 1.0
        )
        bucket.setdefault("rms_cross_track_m", []).append(
            float(row["rms_cross_track_m"])
        )
        elapsed = max(float(row["elapsed_s"]), 1e-9)
        bucket.setdefault("safety_violation_rate", []).append(
            float(row["safety_violations"]) / elapsed
        )

    if normalisation is None:
        normalisation = {}
        for key in weights:
            pooled = [v for b in per_policy.values() for v in b.get(key, [])]
            scale = float(np.mean(pooled)) if pooled else 0.0
            normalisation[key] = scale if abs(scale) > 1e-12 else 1.0

    scenarios = sorted({r["scenario"] for r in rows})
    out: dict[str, float] = {}
    for policy in per_policy:
        per_family: list[float] = []
        for scenario in scenarios:
            subset = [
                r for r in rows
                if r["policy"] == policy and r["scenario"] == scenario
            ]
            if not subset:
                continue
            total = 0.0
            for key, weight in weights.items():
                values = [_component(r, key) for r in subset]
                total += weight * float(np.mean(values)) / normalisation[key]
            per_family.append(total)
        out[policy] = float(np.mean(per_family)) if per_family else float("nan")
    return out


def _component(row: Mapping, key: str) -> float:
    """One primary-metric component of a single run."""
    if key == "failed_mission_rate":
        return 0.0 if row["completed"] else 1.0
    if key == "safety_violation_rate":
        return float(row["safety_violations"]) / max(float(row["elapsed_s"]), 1e-9)
    return float(row[key])


def write_csv(rows: Iterable[Mapping], path: str | Path) -> Path:
    """Write per-run outcomes. Every run is reported; none are filtered."""
    rows = list(rows)
    if not rows:
        raise ValueError("nothing to write")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return path


# ---------------------------------------------------------------------------
# The static Pareto frontier
# ---------------------------------------------------------------------------
def survey_productivity(rows: Sequence[Mapping]) -> dict[str, float]:
    """Seabed area imaged per second, per policy.

    Swath width scales with altitude and ground speed is path length over
    elapsed time, so their product is the rate at which a survey actually
    covers ground. Both factors are already declared as secondary metrics
    ``S2`` and ``S3``; this combines them into the single quantity an operator
    cares about, and introduces no new measurement.
    """
    grouped: dict[str, list[float]] = {}
    for row in rows:
        elapsed = max(float(row["elapsed_s"]), 1e-9)
        rate = float(row["mean_altitude_m"]) * float(row["path_length_m"]) / elapsed
        grouped.setdefault(row["policy"], []).append(rate)
    return {k: float(np.mean(v)) for k, v in grouped.items()}


def pareto_frontier(
    points: Mapping[str, tuple[float, float]]
) -> list[str]:
    """Names of the non-dominated points, given ``{name: (cost, benefit)}``.

    A point is dominated when another is no worse on both axes and strictly
    better on one. Lower ``cost`` and higher ``benefit`` are preferred.
    """
    out = []
    for name, (cost, benefit) in points.items():
        beaten = any(
            (c <= cost and b >= benefit) and (c < cost or b > benefit)
            for other, (c, b) in points.items()
            if other != name
        )
        if not beaten:
            out.append(name)
    return sorted(out, key=lambda n: -points[n][1])


def frontier_report(
    sweep_rows: Sequence[Mapping],
    policy_rows: Sequence[Mapping],
    weights: Mapping[str, float] = DEFAULT_WEIGHTS,
) -> dict:
    """Compare adaptive policies against the frontier of *all* static configurations.

    Why a frontier rather than a single best baseline
    -------------------------------------------------
    Navigation quality and survey productivity are separate axes, and the
    predeclared aggregate ``J`` measures only the first. Reducing eighteen static
    configurations to one "best" by ``J`` therefore discards the second axis
    entirely, and does so at the point where it matters most: in the development
    campaign the top two configurations were separated by 0.3% of ``J`` -- one
    failed run in eighty -- while differing by a factor of two in mission time.
    The tie-break between them was effectively arbitrary, and because the slower
    of the two accumulates roughly 0.11 m more cross-track error from the ambient
    current, that arbitrary choice injected a constant apparent improvement into
    every comparison downstream, including scenarios in which nothing was
    degraded and there was nothing for a manager to do.

    Publishing the whole frontier removes both problems without altering any
    predeclared quantity. ``J`` is unchanged and still reported as declared;
    productivity is built from secondary metrics that were already required to be
    reported; and no configuration is selected away, so a reader who prefers a
    different operating point can locate it in the table and read off its score.

    The resulting claim is stronger than "we beat the tuned baseline" and much
    harder to game: *no static configuration attains this navigation outcome at
    any productivity.* If that is false, the frontier shows it immediately.
    """
    static_j = aggregate_outcome(sweep_rows, weights)
    static_p = survey_productivity(sweep_rows)
    statics = {k: (static_j[k], static_p[k]) for k in static_j}

    policy_j = aggregate_outcome(policy_rows, weights)
    policy_p = survey_productivity(policy_rows)

    frontier = pareto_frontier(statics)
    best_static_j = min(static_j.values())
    max_static_p = max(static_p.values())

    verdicts = {}
    for name in policy_j:
        j, p = policy_j[name], policy_p[name]
        dominated_by = [
            s for s, (sj, sp) in statics.items()
            if (sj <= j and sp >= p) and (sj < j or sp > p)
        ]
        verdicts[name] = {
            "J": j,
            "productivity": p,
            "dominated_by": dominated_by,
            # The claim that matters: no static configuration reaches this
            # navigation quality, at any productivity whatsoever.
            "beats_every_static_on_J": j < best_static_j,
            "productivity_fraction_of_max": p / max_static_p if max_static_p else float("nan"),
        }
    return {
        "static_points": statics,
        "frontier": frontier,
        "best_static_J": best_static_j,
        "max_static_productivity": max_static_p,
        "policies": verdicts,
    }


def mean_ground_speed(rows: Sequence[Mapping]) -> dict[str, float]:
    """Realised ground speed (m/s) per policy: path flown over time taken.

    The *commanded* speed is not enough. A policy that holds station, or that
    saturates its thrusters crabbing into a current, travels more slowly than it
    asked to, and it is the realised speed that sets how long the vehicle is
    exposed to whatever is degrading it.
    """
    grouped: dict[str, list[float]] = {}
    for row in rows:
        elapsed = max(float(row["elapsed_s"]), 1e-9)
        grouped.setdefault(row["policy"], []).append(
            float(row["path_length_m"]) / elapsed
        )
    return {k: float(np.mean(v)) for k, v in grouped.items()}


def speed_matched_baseline(
    sweep_rows: Sequence[Mapping],
    policy_rows: Sequence[Mapping],
    policy: str = "proposed",
    weights: Mapping[str, float] = DEFAULT_WEIGHTS,
    tolerance: float = 0.05,
) -> dict:
    """Best static configuration among those flying at the policy's own speed.

    Why this is necessary
    ---------------------
    ``rms_cross_track_m`` is not comparable across policies that travel at
    different speeds, and the direction of the bias reverses between scenario
    families. Holding every other factor fixed and varying only commanded speed,
    the development campaign measured 0.188 m at 0.25 m/s against 0.101 m at
    0.50 m/s in nominal conditions -- faster is better -- and 36.22 m against
    78.04 m in the compound-failure family -- faster is worse.

    Both directions are real physics rather than artefacts. Time-driven error
    sources such as inertial bias and a stale current estimate accumulate for
    twice as long when the vehicle flies at half the speed over the same path,
    which penalises slow flight when nothing is wrong. Outage *windows*, on the
    other hand, are fixed in time, so a faster vehicle travels twice as far off
    track before aiding returns, which penalises fast flight when something is.

    The consequence is that a manager which selects a different speed from the
    tuned static baseline will appear better or worse than it is, by a factor of
    roughly two, with the sign depending on the family. The aggregate over a
    failure matrix is then a weighted average of two opposing artefacts and means
    very little. This function removes the confound by asking a narrower and
    answerable question: *among the static configurations that fly at the speed
    the manager actually chose, is any of them better than the manager?*

    Anything left after matching is attributable to reconfiguration during the
    run, which is the paper's actual claim.

    Returns the matched baseline, its score, the policy's score, and the number
    of static configurations that fell inside the speed window -- the last so a
    reader can see whether the match had anything to choose from.
    """
    policy_speeds = mean_ground_speed(policy_rows)
    if policy not in policy_speeds:
        raise KeyError(f"{policy!r} is not in the supplied policy rows")
    target = policy_speeds[policy]

    static_speeds = mean_ground_speed(sweep_rows)
    matched = {
        name: speed
        for name, speed in static_speeds.items()
        if abs(speed - target) <= tolerance
    }
    if not matched:
        # Report the failure rather than silently widening the window: an empty
        # match means the manager flies at a speed no static configuration
        # achieves, which is itself a result and must not be papered over.
        nearest = min(static_speeds, key=lambda n: abs(static_speeds[n] - target))
        return {
            "policy": policy,
            "policy_speed_mps": target,
            "matched_count": 0,
            "nearest_static": nearest,
            "nearest_static_speed_mps": static_speeds[nearest],
            "baseline": None,
        }

    static_scores = aggregate_outcome(
        [r for r in sweep_rows if r["policy"] in matched], weights=weights
    )
    policy_scores = aggregate_outcome(policy_rows, weights=weights)
    best = min(static_scores, key=lambda n: static_scores[n])
    return {
        "policy": policy,
        "policy_speed_mps": target,
        "policy_J": policy_scores[policy],
        "baseline": best,
        "baseline_speed_mps": static_speeds[best],
        "baseline_J": static_scores[best],
        "matched_count": len(matched),
        "improvement": static_scores[best] - policy_scores[policy],
        "beats_matched_baseline": policy_scores[policy] < static_scores[best],
    }
