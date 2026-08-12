"""Nadir single-beam altimeter for platform-v2 terrain matching."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


class DepthMap(Protocol):
    def sample(self, x_m, y_m): ...


@dataclass(frozen=True)
class AltimeterProfile:
    relative_xy_m: np.ndarray
    range_m: np.ndarray
    vehicle_depth_m: np.ndarray
    sigma_m: float

    def __post_init__(self) -> None:
        n = len(self.range_m)
        if self.relative_xy_m.shape != (n, 2):
            raise ValueError("relative_xy_m must be Nx2")
        if self.vehicle_depth_m.shape != (n,):
            raise ValueError("vehicle_depth_m must have N entries")
        if self.sigma_m <= 0.0:
            raise ValueError("sigma_m must be positive")


@dataclass(frozen=True)
class AltimeterModel:
    sigma_m: float = 0.02
    maximum_range_m: float = 50.0

    def __post_init__(self) -> None:
        if self.sigma_m <= 0.0 or self.maximum_range_m <= 0.0:
            raise ValueError("altimeter sigma and maximum range must be positive")

    def sample_profile(
        self,
        truth_map: DepthMap,
        world_xy_m: np.ndarray,
        vehicle_depth_m: np.ndarray,
        rng: np.random.Generator,
    ) -> AltimeterProfile:
        xy = np.asarray(world_xy_m, dtype=float)
        depth = np.asarray(vehicle_depth_m, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or depth.shape != (len(xy),):
            raise ValueError("world_xy_m must be Nx2 and vehicle_depth_m must be N")
        truth = depth - truth_map.sample(xy[:, 0], xy[:, 1])
        if np.any(truth <= 0.0) or np.any(truth > self.maximum_range_m):
            raise ValueError("seabed lies outside the altimeter range")
        measured = truth + rng.normal(0.0, self.sigma_m, len(truth))
        return AltimeterProfile(
            relative_xy_m=xy - xy[0],
            range_m=measured,
            vehicle_depth_m=depth.copy(),
            sigma_m=self.sigma_m,
        )

