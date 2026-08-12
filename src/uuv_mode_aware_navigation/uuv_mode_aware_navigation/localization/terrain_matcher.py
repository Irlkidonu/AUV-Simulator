"""Coarse-to-fine bathymetric profile matcher with ambiguity rejection."""

from __future__ import annotations

from dataclasses import dataclass
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
class TerrainMatch:
    success: bool
    reason: str
    position_xy_m: np.ndarray | None
    covariance_m2: np.ndarray | None
    normalized_rms: float
    best_second_likelihood_ratio: float
    minimum_information_eigenvalue: float
    samples_used: int
    candidates_evaluated: int
    runtime_ms: float


@dataclass(frozen=True)
class TerrainMatcher:
    search_radius_m: float = 2.5
    coarse_step_m: float = 0.20
    refinement_radius_m: float = 0.25
    fine_step_m: float = 0.025
    ambiguity_separation_m: float = 0.50
    #: Minimum chi-square separation between the best and the best spatially
    #: distinct hypothesis. 9.21 is the 99% threshold for two position degrees
    #: of freedom; unlike a raw cost ratio it retains meaning as noise changes.
    minimum_hypothesis_delta_chi2: float = 9.210340371976184
    minimum_samples: int = 20
    minimum_information_eigenvalue: float = 5.0
    maximum_normalized_rms: float = 3.0
    map_sigma_m: float = 0.0
    map_correlation_length_m: float = 1.0

    def __post_init__(self) -> None:
        positive = (
            self.search_radius_m, self.coarse_step_m,
            self.refinement_radius_m, self.fine_step_m,
            self.ambiguity_separation_m, self.minimum_hypothesis_delta_chi2,
        )
        if any(not np.isfinite(v) or v <= 0.0 for v in positive):
            raise ValueError("matcher geometry and thresholds must be positive")
        if self.map_sigma_m < 0.0 or self.map_correlation_length_m <= 0.0:
            raise ValueError("map uncertainty parameters are invalid")

    @staticmethod
    def _disk(radius: float, step: float) -> np.ndarray:
        axis = np.arange(-radius, radius + 0.5 * step, step)
        xx, yy = np.meshgrid(axis, axis, indexing="xy")
        offsets = np.column_stack((xx.ravel(), yy.ravel()))
        return offsets[np.linalg.norm(offsets, axis=1) <= radius + 1e-12]

    @staticmethod
    def _costs(reference, profile: AltimeterProfile, origins: np.ndarray) -> np.ndarray:
        xy = origins[:, None, :] + profile.relative_xy_m[None, :, :]
        predicted = (
            profile.vehicle_depth_m[None, :]
            - reference.sample(xy[:, :, 0], xy[:, :, 1])
        )
        residual = profile.range_m[None, :] - predicted
        return np.mean(residual * residual, axis=1)

    def match(
        self,
        reference: BathymetricReference,
        profile: AltimeterProfile,
        initial_xy_m: np.ndarray,
    ) -> TerrainMatch:
        started = time.perf_counter()
        initial = np.asarray(initial_xy_m, dtype=float)
        if initial.shape != (2,):
            raise ValueError("initial_xy_m must contain x and y")
        if len(profile.range_m) < self.minimum_samples:
            return self._failure("insufficient_samples", started, len(profile.range_m), 0)

        coarse_origins = initial + self._disk(
            self.search_radius_m, self.coarse_step_m
        )
        coarse_costs = self._costs(reference, profile, coarse_origins)
        coarse_best = coarse_origins[int(np.argmin(coarse_costs))]
        fine_origins = coarse_best + self._disk(
            self.refinement_radius_m, self.fine_step_m
        )
        fine_costs = self._costs(reference, profile, fine_origins)
        best_index = int(np.argmin(fine_costs))
        best = fine_origins[best_index]
        best_cost = float(fine_costs[best_index])
        evaluated = len(coarse_origins) + len(fine_origins)

        all_origins = np.vstack((coarse_origins, fine_origins))
        all_costs = np.concatenate((coarse_costs, fine_costs))
        separated = np.linalg.norm(all_origins - best, axis=1) >= self.ambiguity_separation_m
        second_cost = float(np.min(all_costs[separated])) if np.any(separated) else math.inf
        likelihood_ratio = second_cost / max(best_cost, 1e-15)
        hypothesis_delta_chi2 = (
            len(profile.range_m) * (second_cost - best_cost)
            / max(profile.sigma_m ** 2 + self.map_sigma_m ** 2, 1e-15)
        )

        xy = best + profile.relative_xy_m
        gx, gy = reference.gradient_vector(xy[:, 0], xy[:, 1])
        jacobian = -np.column_stack((gx, gy))

        # Correlated map error contributes fewer independent samples than sensor
        # noise. The exponential-correlation inflation is fixed from the declared
        # 0.25 m spacing and correlation length, not fitted to spike results.
        spacing = float(np.median(np.linalg.norm(np.diff(profile.relative_xy_m, axis=0), axis=1)))
        rho = math.exp(-spacing / self.map_correlation_length_m)
        correlation_inflation = (1.0 + rho) / max(1.0 - rho, 1e-9)
        effective_variance = (
            profile.sigma_m ** 2
            + self.map_sigma_m ** 2 * correlation_inflation
        )
        information = jacobian.T @ jacobian / max(effective_variance, 1e-15)
        eigenvalues = np.linalg.eigvalsh(information)
        minimum_eigenvalue = float(eigenvalues[0])
        normalized_rms = math.sqrt(best_cost / max(
            profile.sigma_m ** 2 + self.map_sigma_m ** 2, 1e-15
        ))

        if minimum_eigenvalue < self.minimum_information_eigenvalue:
            return self._failure(
                "unobservable", started, len(profile.range_m), evaluated,
                normalized_rms, likelihood_ratio, minimum_eigenvalue,
            )
        if hypothesis_delta_chi2 < self.minimum_hypothesis_delta_chi2:
            return self._failure(
                "ambiguous", started, len(profile.range_m), evaluated,
                normalized_rms, likelihood_ratio, minimum_eigenvalue,
            )
        if normalized_rms > self.maximum_normalized_rms:
            return self._failure(
                "model_mismatch", started, len(profile.range_m), evaluated,
                normalized_rms, likelihood_ratio, minimum_eigenvalue,
            )

        covariance = np.linalg.inv(information)
        covariance += np.eye(2) * (self.fine_step_m ** 2 / 12.0)
        return TerrainMatch(
            True, "accepted", best.copy(), covariance, normalized_rms,
            likelihood_ratio, minimum_eigenvalue, len(profile.range_m),
            evaluated, (time.perf_counter() - started) * 1000.0,
        )

    @staticmethod
    def _failure(
        reason: str,
        started: float,
        samples: int,
        evaluated: int,
        normalized_rms: float = math.inf,
        ratio: float = 0.0,
        eigenvalue: float = 0.0,
    ) -> TerrainMatch:
        return TerrainMatch(
            False, reason, None, None, normalized_rms, ratio, eigenvalue,
            samples, evaluated, (time.perf_counter() - started) * 1000.0,
        )
