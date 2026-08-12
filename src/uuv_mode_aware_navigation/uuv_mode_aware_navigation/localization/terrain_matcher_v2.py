"""Multi-hypothesis terrain matcher with explicit ambiguity confidence.

This platform-v2 implementation is separate from the immutable P6-v1 matcher.
It treats local observability and global uniqueness as distinct requirements.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from typing import Protocol

import numpy as np

from ..sensor_models.altimeter import AltimeterProfile


class BathymetricReference(Protocol):
    metres_per_cell: float
    def sample(self, x_m, y_m): ...
    def gradient_vector(self, x_m, y_m): ...


@dataclass(frozen=True)
class TerrainMatchV2:
    success: bool
    reason: str
    position_xy_m: np.ndarray | None
    covariance_m2: np.ndarray | None
    normalized_rms: float
    posterior_mass: float
    hypothesis_delta_chi2: float
    minimum_information_eigenvalue: float
    coarse_basins: int
    refined_basins: int
    window_disagreement_m: float
    samples_used: int
    candidates_evaluated: int
    runtime_ms: float


@dataclass(frozen=True)
class _Candidate:
    position: np.ndarray
    cost: float


@dataclass(frozen=True)
class TerrainMatcherV2:
    search_radius_m: float = 3.5
    coarse_step_m: float = 0.20
    refinement_radius_m: float = 0.25
    fine_step_m: float = 0.025
    basin_cluster_m: float = 0.35
    ambiguity_separation_m: float = 0.50
    search_boundary_margin_m: float = 1.0
    maximum_refined_basins: int = 32
    minimum_hypothesis_delta_chi2: float = 13.815510557964274
    minimum_posterior_mass: float = 0.99
    minimum_samples: int = 20
    minimum_information_eigenvalue: float = 5.0
    maximum_normalized_rms: float = 3.0
    map_sigma_m: float = 0.0
    map_correlation_length_m: float = 1.0
    maximum_window_disagreement_m: float = 0.25
    maximum_window_disagreement_nees: float = 9.210340371976184
    covariance_scale: float = 1.0

    def __post_init__(self) -> None:
        positive = (
            self.search_radius_m, self.coarse_step_m,
            self.refinement_radius_m, self.fine_step_m,
            self.basin_cluster_m, self.ambiguity_separation_m,
            self.search_boundary_margin_m, self.minimum_hypothesis_delta_chi2,
            self.minimum_posterior_mass, self.map_correlation_length_m,
            self.maximum_window_disagreement_m,
            self.maximum_window_disagreement_nees, self.covariance_scale,
        )
        if any(not np.isfinite(v) or v <= 0.0 for v in positive):
            raise ValueError("matcher parameters must be finite and positive")
        if self.minimum_posterior_mass > 1.0:
            raise ValueError("minimum_posterior_mass cannot exceed one")
        if self.maximum_refined_basins < 2 or self.minimum_samples < 4:
            raise ValueError("insufficient hypothesis or sample support")
        if self.map_sigma_m < 0.0:
            raise ValueError("map_sigma_m cannot be negative")

    @staticmethod
    def _disk(radius: float, step: float) -> np.ndarray:
        axis = np.arange(-radius, radius + 0.5 * step, step)
        xx, yy = np.meshgrid(axis, axis, indexing="xy")
        offsets = np.column_stack((xx.ravel(), yy.ravel()))
        return offsets[np.linalg.norm(offsets, axis=1) <= radius + 1e-12]

    @staticmethod
    def _costs(reference, profile: AltimeterProfile, origins: np.ndarray) -> np.ndarray:
        xy = origins[:, None, :] + profile.relative_xy_m[None, :, :]
        predicted = profile.vehicle_depth_m[None, :] - reference.sample(xy[:, :, 0], xy[:, :, 1])
        residual = profile.range_m[None, :] - predicted
        return np.mean(residual * residual, axis=1)

    def _local_minima(self, origins: np.ndarray, costs: np.ndarray) -> list[_Candidate]:
        # Integer lattice keys avoid fragile floating-point coordinate equality.
        scaled = np.rint(origins / self.coarse_step_m).astype(np.int64)
        lookup = {tuple(key): index for index, key in enumerate(scaled)}
        minima: list[_Candidate] = []
        for index, key in enumerate(scaled):
            neighbours = []
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbour = lookup.get((int(key[0] + dx), int(key[1] + dy)))
                    if neighbour is not None:
                        neighbours.append(costs[neighbour])
            if neighbours and costs[index] < min(neighbours):
                minima.append(_Candidate(origins[index].copy(), float(costs[index])))
        if not minima:
            index = int(np.argmin(costs))
            minima = [_Candidate(origins[index].copy(), float(costs[index]))]
        minima.sort(key=lambda item: item.cost)
        clustered: list[_Candidate] = []
        for candidate in minima:
            if all(np.linalg.norm(candidate.position - kept.position) >= self.basin_cluster_m for kept in clustered):
                clustered.append(candidate)
            if len(clustered) == self.maximum_refined_basins:
                break
        return clustered

    def _refine(self, reference, profile: AltimeterProfile, basins: list[_Candidate]):
        offsets = self._disk(self.refinement_radius_m, self.fine_step_m)
        refined: list[_Candidate] = []
        evaluated = 0
        for basin in basins:
            origins = basin.position + offsets
            costs = self._costs(reference, profile, origins)
            evaluated += len(origins)
            index = int(np.argmin(costs))
            candidate = _Candidate(origins[index].copy(), float(costs[index]))
            prior = next((i for i, item in enumerate(refined) if np.linalg.norm(item.position - candidate.position) < self.basin_cluster_m), None)
            if prior is None:
                refined.append(candidate)
            elif candidate.cost < refined[prior].cost:
                refined[prior] = candidate
        refined.sort(key=lambda item: item.cost)
        return refined, evaluated

    def _variance(self, profile: AltimeterProfile) -> float:
        spacing = float(np.median(np.linalg.norm(np.diff(profile.relative_xy_m, axis=0), axis=1)))
        rho = math.exp(-spacing / self.map_correlation_length_m)
        inflation = (1.0 + rho) / max(1.0 - rho, 1e-9)
        return profile.sigma_m**2 + self.map_sigma_m**2 * inflation

    def _single(self, reference, profile: AltimeterProfile, initial: np.ndarray):
        offsets = self._disk(self.search_radius_m, self.coarse_step_m)
        coarse_origins = initial + offsets
        coarse_costs = self._costs(reference, profile, coarse_origins)
        basins = self._local_minima(coarse_origins, coarse_costs)
        refined, fine_evaluated = self._refine(reference, profile, basins)
        best = refined[0]
        variance = max(self._variance(profile), 1e-15)
        n = len(profile.range_m)
        chi2 = np.asarray([n * item.cost / variance for item in refined])
        delta = chi2 - chi2[0]
        weights = np.exp(-0.5 * np.clip(delta, 0.0, 1500.0))
        posterior = float(weights[0] / np.sum(weights))
        separated = [
            (item, float(d)) for item, d in zip(refined[1:], delta[1:])
            if np.linalg.norm(item.position - best.position) >= self.ambiguity_separation_m
        ]
        second_delta = min((d for _, d in separated), default=math.inf)

        xy = best.position + profile.relative_xy_m
        gx, gy = reference.gradient_vector(xy[:, 0], xy[:, 1])
        jacobian = -np.column_stack((gx, gy))
        information = jacobian.T @ jacobian / variance
        min_eig = float(np.linalg.eigvalsh(information)[0])
        normalized_rms = math.sqrt(best.cost / max(profile.sigma_m**2 + self.map_sigma_m**2, 1e-15))
        covariance = None
        if min_eig > 0.0:
            covariance = np.linalg.inv(information) + np.eye(2) * self.fine_step_m**2 / 12.0

        reason = "accepted"
        if min_eig < self.minimum_information_eigenvalue:
            reason = "unobservable"
        elif self.search_radius_m - np.linalg.norm(best.position - initial) < self.search_boundary_margin_m:
            reason = "search_boundary"
        elif second_delta < self.minimum_hypothesis_delta_chi2 or posterior < self.minimum_posterior_mass:
            reason = "ambiguous"
        elif normalized_rms > self.maximum_normalized_rms:
            reason = "model_mismatch"
        return {
            "success": reason == "accepted", "reason": reason, "best": best,
            "covariance": covariance, "normalized_rms": normalized_rms,
            "posterior": posterior, "delta": second_delta, "min_eig": min_eig,
            "coarse_basins": len(basins), "refined_basins": len(refined),
            "evaluated": len(coarse_origins) + fine_evaluated,
            "jacobian": jacobian,
        }

    @staticmethod
    def _window(profile: AltimeterProfile, start: int, stop: int) -> AltimeterProfile:
        return AltimeterProfile(
            profile.relative_xy_m[start:stop].copy(), profile.range_m[start:stop].copy(),
            profile.vehicle_depth_m[start:stop].copy(), profile.sigma_m,
        )

    def match(self, reference: BathymetricReference, profile: AltimeterProfile, initial_xy_m: np.ndarray) -> TerrainMatchV2:
        started = time.perf_counter()
        initial = np.asarray(initial_xy_m, dtype=float)
        if initial.shape != (2,):
            raise ValueError("initial_xy_m must contain x and y")
        if len(profile.range_m) < 49:
            return self._failure("insufficient_samples", started, len(profile.range_m))

        full = self._single(reference, profile, initial)
        if not full["success"]:
            return self._from_single(full, started, len(profile.range_m))
        first = self._single(reference, self._window(profile, 0, 31), initial)
        second = self._single(reference, self._window(profile, 18, 49), initial)
        evaluated = full["evaluated"] + first["evaluated"] + second["evaluated"]
        if not first["success"] or not second["success"]:
            failed = replace(self._from_single(full, started, len(profile.range_m)),
                             success=False, reason="profile_inconsistent",
                             candidates_evaluated=evaluated)
            return failed

        estimates = np.vstack((full["best"].position, first["best"].position, second["best"].position))
        disagreement = float(np.linalg.norm(first["best"].position - second["best"].position))
        combined = first["covariance"] + second["covariance"]
        difference = first["best"].position - second["best"].position
        disagreement_nees = float(difference @ np.linalg.solve(combined, difference))
        if disagreement > self.maximum_window_disagreement_m or disagreement_nees > self.maximum_window_disagreement_nees:
            return replace(self._from_single(full, started, len(profile.range_m)),
                           success=False, reason="profile_inconsistent",
                           window_disagreement_m=disagreement,
                           candidates_evaluated=evaluated)

        covariance = full["covariance"].copy()
        covariance += np.cov(estimates, rowvar=False, ddof=1) / len(estimates)
        # Segment sandwich: score variation across three contiguous thirds.
        xy = full["best"].position + profile.relative_xy_m
        predicted = profile.vehicle_depth_m - reference.sample(xy[:, 0], xy[:, 1])
        residual = profile.range_m - predicted
        scores = []
        for indices in np.array_split(np.arange(len(residual)), 3):
            scores.append(full["jacobian"][indices].T @ residual[indices])
        meat = sum(np.outer(score, score) for score in scores)
        h_inv = np.linalg.inv(full["jacobian"].T @ full["jacobian"])
        covariance += h_inv @ meat @ h_inv
        covariance *= self.covariance_scale
        return TerrainMatchV2(
            True, "accepted", full["best"].position.copy(), covariance,
            full["normalized_rms"], full["posterior"], full["delta"],
            full["min_eig"], full["coarse_basins"], full["refined_basins"],
            disagreement, len(profile.range_m), evaluated,
            (time.perf_counter() - started) * 1000.0,
        )

    def _from_single(self, item, started: float, samples: int) -> TerrainMatchV2:
        return TerrainMatchV2(
            False, item["reason"], None, None, item["normalized_rms"],
            item["posterior"], item["delta"], item["min_eig"],
            item["coarse_basins"], item["refined_basins"], math.inf, samples,
            item["evaluated"], (time.perf_counter() - started) * 1000.0,
        )

    @staticmethod
    def _failure(reason: str, started: float, samples: int) -> TerrainMatchV2:
        return TerrainMatchV2(
            False, reason, None, None, math.inf, 0.0, 0.0, 0.0, 0, 0,
            math.inf, samples, 0, (time.perf_counter() - started) * 1000.0,
        )
