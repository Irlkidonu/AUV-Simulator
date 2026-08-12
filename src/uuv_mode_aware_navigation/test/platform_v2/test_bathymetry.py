"""Tests for the platform-v2 georeferenced bathymetry service."""

from __future__ import annotations

import numpy as np
import pytest

from uuv_mode_aware_navigation.maps import BathymetryMap
from uuv_mode_aware_navigation.seabed import depth_at, gradient_at


def test_world_map_transform_round_trip() -> None:
    bathy = BathymetryMap(origin_x_m=-80.0, origin_y_m=15.0, metres_per_cell=0.25)
    x = np.array([-12.5, 0.0, 31.125])
    y = np.array([22.0, -4.5, 11.25])
    col, row = bathy.world_to_map(x, y)
    recovered_x, recovered_y = bathy.map_to_world(col, row)
    np.testing.assert_allclose(recovered_x, x, rtol=0.0, atol=1e-14)
    np.testing.assert_allclose(recovered_y, y, rtol=0.0, atol=1e-14)


def test_sampling_matches_existing_seabed_definition() -> None:
    bathy = BathymetryMap(metres_per_cell=0.25)
    x, y = 4.25, -7.75
    assert bathy.sample(x, y) == depth_at(x, y)
    assert bathy.gradient(x, y) == gradient_at(x, y, step=0.25)


def test_patch_matches_point_sampling() -> None:
    bathy = BathymetryMap(metres_per_cell=0.5)
    patch = bathy.patch(centre_x_m=2.0, centre_y_m=-3.0, extent_m=1.0)
    assert patch.depth_m.shape == (5, 5)
    for row, y_m in enumerate(patch.y_m):
        for col, x_m in enumerate(patch.x_m):
            assert patch.depth_m[row, col] == bathy.sample(x_m, y_m)


@pytest.mark.parametrize("resolution", [0.0, -0.5, float("nan")])
def test_invalid_resolution_is_rejected(resolution: float) -> None:
    with pytest.raises(ValueError, match="positive"):
        BathymetryMap(metres_per_cell=resolution)
