"""Georeferenced bathymetry access for platform-v2 navigation components."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..seabed import depth_at, gradient_at


@dataclass(frozen=True)
class BathymetryPatch:
    """Regular world-aligned depth grid and its coordinate vectors."""

    x_m: np.ndarray
    y_m: np.ndarray
    depth_m: np.ndarray


@dataclass(frozen=True)
class BathymetryMap:
    """Map-frame wrapper over the existing deterministic seabed model.

    Map coordinates are pixel-like continuous indices. World coordinates use
    metres in the same horizontal frame as the vehicle; depth is negative down.
    """

    origin_x_m: float = -60.0
    origin_y_m: float = -60.0
    metres_per_cell: float = 0.5

    def __post_init__(self) -> None:
        if not np.isfinite(self.metres_per_cell) or self.metres_per_cell <= 0.0:
            raise ValueError("metres_per_cell must be finite and positive")

    def world_to_map(self, x_m, y_m):
        col = (np.asarray(x_m, dtype=float) - self.origin_x_m) / self.metres_per_cell
        row = (np.asarray(y_m, dtype=float) - self.origin_y_m) / self.metres_per_cell
        return col, row

    def map_to_world(self, col, row):
        x_m = self.origin_x_m + np.asarray(col, dtype=float) * self.metres_per_cell
        y_m = self.origin_y_m + np.asarray(row, dtype=float) * self.metres_per_cell
        return x_m, y_m

    def sample(self, x_m, y_m):
        """Return seabed depth at world coordinate(s), in metres."""
        return depth_at(x_m, y_m)

    def gradient(self, x_m: float, y_m: float) -> float:
        """Return local slope magnitude in metres per metre."""
        return gradient_at(x_m, y_m, step=self.metres_per_cell)

    def gradient_vector(self, x_m, y_m):
        """Return `(d depth/dx, d depth/dy)` by centred differences."""
        step = self.metres_per_cell
        gx = (self.sample(np.asarray(x_m) + step, y_m)
              - self.sample(np.asarray(x_m) - step, y_m)) / (2.0 * step)
        gy = (self.sample(x_m, np.asarray(y_m) + step)
              - self.sample(x_m, np.asarray(y_m) - step)) / (2.0 * step)
        return gx, gy

    def patch(
        self,
        centre_x_m: float,
        centre_y_m: float,
        extent_m: float,
    ) -> BathymetryPatch:
        """Sample a square patch including both declared boundary coordinates."""
        if not np.isfinite(extent_m) or extent_m < 0.0:
            raise ValueError("extent_m must be finite and non-negative")
        half_cells = int(np.ceil(extent_m / self.metres_per_cell))
        offsets = np.arange(-half_cells, half_cells + 1, dtype=float)
        x_m = float(centre_x_m) + offsets * self.metres_per_cell
        y_m = float(centre_y_m) + offsets * self.metres_per_cell
        xx, yy = np.meshgrid(x_m, y_m, indexing="xy")
        return BathymetryPatch(x_m=x_m, y_m=y_m, depth_m=self.sample(xx, yy))
