#!/usr/bin/env python3
"""Execute the frozen platform-v2 terrain-matching feasibility spike once."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from uuv_mode_aware_navigation.localization import TerrainMatcher
from uuv_mode_aware_navigation.maps import BathymetryMap
from uuv_mode_aware_navigation.sensor_models import AltimeterModel, AltimeterProfile


IDENTIFIER = "p2v2_p6_trn_spike_v1"
SEED_ROOT = 22_200_000
SAMPLES_PER_STRATUM = 50
NOISE_LEVELS_M = (0.02, 0.05, 0.10)
MAP_ERROR_LEVELS_M = (0.0, 0.02, 0.05)
HEADINGS_RAD = tuple(np.linspace(0.0, np.pi, 5, endpoint=False))
CHI2_99_2D = 9.210340371976184


def _seed(label: str, index: int = 0) -> int:
    digest = hashlib.sha256(f"{SEED_ROOT}:{label}:{index}".encode()).digest()
    return int.from_bytes(digest[:8], "little")


class FlatMap:
    metres_per_cell = 0.10

    def sample(self, x_m, y_m):
        return np.zeros(np.broadcast(np.asarray(x_m), np.asarray(y_m)).shape) - 20.0

    def gradient_vector(self, x_m, y_m):
        shape = np.broadcast(np.asarray(x_m), np.asarray(y_m)).shape
        return np.zeros(shape), np.zeros(shape)


class RepetitiveMap:
    metres_per_cell = 0.10
    period_m = 1.0

    def sample(self, x_m, y_m):
        x = np.asarray(x_m)
        y = np.asarray(y_m)
        return -20.0 + 0.35 * np.sin(2.0 * np.pi * x) + 0.05 * np.cos(2.0 * np.pi * y)

    def gradient_vector(self, x_m, y_m):
        x = np.asarray(x_m)
        y = np.asarray(y_m)
        return 0.7 * np.pi * np.cos(2.0 * np.pi * x), -0.1 * np.pi * np.sin(2.0 * np.pi * y)


@dataclass(frozen=True)
class PerturbedReference:
    """Bathymetry plus a deterministic, correlated vertical map-error field."""

    base: object
    sigma_m: float
    phases: np.ndarray
    metres_per_cell: float = 0.10

    def _unit_error(self, x_m, y_m):
        x = np.asarray(x_m, dtype=float)
        y = np.asarray(y_m, dtype=float)
        # Wavelengths exceed the declared 1 m minimum correlation length.
        raw = (
            np.sin(2.0 * np.pi * x / 4.7 + self.phases[0])
            + np.cos(2.0 * np.pi * y / 5.9 + self.phases[1])
            + 0.7 * np.sin(2.0 * np.pi * (x + y) / 7.3 + self.phases[2])
        )
        return raw / math.sqrt(1.0 + 1.0 + 0.7**2)

    def sample(self, x_m, y_m):
        return self.base.sample(x_m, y_m) + self.sigma_m * self._unit_error(x_m, y_m)

    def gradient_vector(self, x_m, y_m):
        step = self.metres_per_cell
        gx = (self.sample(np.asarray(x_m) + step, y_m) - self.sample(np.asarray(x_m) - step, y_m)) / (2.0 * step)
        gy = (self.sample(x_m, np.asarray(y_m) + step) - self.sample(x_m, np.asarray(y_m) - step)) / (2.0 * step)
        return gx, gy


def _track(origin: np.ndarray, heading: float) -> np.ndarray:
    along = np.arange(49, dtype=float) * 0.25
    return origin + along[:, None] * np.array([math.cos(heading), math.sin(heading)])


def _initial(rng: np.random.Generator, truth: np.ndarray) -> np.ndarray:
    radius = 2.0 * math.sqrt(float(rng.random()))
    angle = float(rng.uniform(0.0, 2.0 * np.pi))
    return truth + radius * np.array([math.cos(angle), math.sin(angle)])


def _record(result, truth: np.ndarray) -> dict:
    row = {
        "success": bool(result.success),
        "reason": result.reason,
        "normalized_rms": float(result.normalized_rms),
        "likelihood_ratio": float(result.best_second_likelihood_ratio),
        "minimum_information_eigenvalue": float(result.minimum_information_eigenvalue),
        "runtime_ms": float(result.runtime_ms),
    }
    if not result.success:
        row.update(error_m=None, ellipse_nees=None, ellipse_contains_truth=None, false_convergence=False)
        return row
    error = np.asarray(result.position_xy_m) - truth
    nees = float(error @ np.linalg.solve(result.covariance_m2, error))
    error_m = float(np.linalg.norm(error))
    row.update(
        error_m=error_m,
        ellipse_nees=nees,
        ellipse_contains_truth=bool(nees <= CHI2_99_2D),
        covariance_eigenvalues_m2=np.linalg.eigvalsh(result.covariance_m2).tolist(),
        false_convergence=bool(error_m > 0.50 or nees > CHI2_99_2D),
    )
    return row


def _summary(rows: list[dict]) -> dict:
    successes = [r for r in rows if r["success"]]
    errors = np.asarray([r["error_m"] for r in successes], dtype=float)
    runtimes = np.asarray([r["runtime_ms"] for r in rows], dtype=float)
    false_count = sum(bool(r["false_convergence"]) for r in rows)
    coverage = ([r["ellipse_contains_truth"] for r in successes])
    return {
        "total": len(rows),
        "successes": len(successes),
        "fix_rate": len(successes) / len(rows),
        "false_convergences": false_count,
        "false_convergence_rate": false_count / len(rows),
        "median_error_m": float(np.median(errors)) if len(errors) else None,
        "p95_error_m": float(np.percentile(errors, 95)) if len(errors) else None,
        "maximum_error_m": float(np.max(errors)) if len(errors) else None,
        "ellipse_99_coverage": float(np.mean(coverage)) if coverage else None,
        "median_runtime_ms": float(np.median(runtimes)),
        "p95_runtime_ms": float(np.percentile(runtimes, 95)),
        "rejection_reasons": {reason: sum(r["reason"] == reason for r in rows) for reason in sorted({r["reason"] for r in rows})},
    }


def run() -> dict:
    truth_map = BathymetryMap(metres_per_cell=0.10)
    strata: dict[str, dict] = {}
    raw: dict[str, list[dict]] = {}
    all_rows: list[dict] = []

    for noise in NOISE_LEVELS_M:
        for map_error in MAP_ERROR_LEVELS_M:
            label = f"informative_noise_{noise:.2f}_map_{map_error:.2f}"
            rng = np.random.default_rng(_seed(label))
            phases = rng.uniform(0.0, 2.0 * np.pi, size=3)
            reference = PerturbedReference(truth_map, map_error, phases)
            matcher = TerrainMatcher(map_sigma_m=map_error)
            rows = []
            for index in range(SAMPLES_PER_STRATUM):
                heading = HEADINGS_RAD[index % len(HEADINGS_RAD)]
                # Structured outer relief, with small deterministic position
                # variation; the central survey patch is intentionally gentle.
                truth = np.array([30.0, 20.0]) + rng.uniform(-1.0, 1.0, size=2)
                xy = _track(truth, heading)
                profile = AltimeterModel(noise).sample_profile(
                    truth_map, xy, np.full(49, -17.0), rng
                )
                rows.append(_record(matcher.match(reference, profile, _initial(rng, truth)), truth))
            raw[label] = rows
            strata[label] = _summary(rows)
            all_rows.extend(rows)

    flat_rows = []
    repetitive_rows = []
    flat = FlatMap()
    repetitive = RepetitiveMap()
    for label, terrain, rows in (
        ("flat", flat, flat_rows), ("repetitive", repetitive, repetitive_rows)
    ):
        rng = np.random.default_rng(_seed(label))
        matcher = TerrainMatcher()
        for index in range(SAMPLES_PER_STRATUM):
            heading = HEADINGS_RAD[index % len(HEADINGS_RAD)]
            truth = rng.uniform(-0.4, 0.4, size=2)
            xy = _track(truth, heading)
            profile = AltimeterModel(0.02).sample_profile(
                terrain, xy, np.full(49, -17.0), rng
            )
            rows.append(_record(matcher.match(terrain, profile, _initial(rng, truth)), truth))
        raw[label] = rows
        strata[label] = _summary(rows)
        all_rows.extend(rows)

    reference = strata["informative_noise_0.02_map_0.00"]
    informative = [v for k, v in strata.items() if k.startswith("informative_")]
    covariance_eigenvalues_positive = all(
        all(value > 0.0 for value in row.get("covariance_eigenvalues_m2", []))
        for rows in raw.values() for row in rows if row["success"]
    )
    criteria = {
        "informative_reference_fix_rate_at_least_0_90": reference["fix_rate"] >= 0.90,
        "informative_reference_median_error_below_0_10_m": reference["median_error_m"] is not None and reference["median_error_m"] < 0.10,
        "informative_reference_p95_error_below_0_25_m": reference["p95_error_m"] is not None and reference["p95_error_m"] < 0.25,
        "informative_false_convergence_below_0_01_each_stratum": all(v["false_convergence_rate"] < 0.01 for v in informative),
        "flat_success_exactly_zero": strata["flat"]["successes"] == 0,
        "repetitive_false_convergence_below_0_05": strata["repetitive"]["false_convergence_rate"] < 0.05,
        "covariance_positive_definite": covariance_eigenvalues_positive,
        "informative_ellipse_coverage_0_90_to_0_99_each_stratum": all(v["ellipse_99_coverage"] is not None and 0.90 <= v["ellipse_99_coverage"] <= 0.99 for v in informative),
        "runtime_median_below_50_ms": float(np.median([r["runtime_ms"] for r in all_rows])) < 50.0,
        "runtime_p95_below_100_ms": float(np.percentile([r["runtime_ms"] for r in all_rows], 95)) < 100.0,
    }
    return {
        "identifier": IDENTIFIER,
        "status": "FEASIBILITY PASS" if all(criteria.values()) else "FAIL",
        "seed_root": SEED_ROOT,
        "samples_per_stratum": SAMPLES_PER_STRATUM,
        "strata": strata,
        "criteria": {k: bool(v) for k, v in criteria.items()},
        "overall_runtime_ms": {
            "median": float(np.median([r["runtime_ms"] for r in all_rows])),
            "p95": float(np.percentile([r["runtime_ms"] for r in all_rows], 95)),
        },
        "raw": raw,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    started = time.time()
    result = run()
    result["wall_time_s"] = time.time() - started
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "raw"}, indent=2, sort_keys=True))
    return 0 if result["status"] == "FEASIBILITY PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
