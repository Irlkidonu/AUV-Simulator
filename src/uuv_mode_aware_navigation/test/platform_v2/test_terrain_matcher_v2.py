"""Development tests for multi-hypothesis TRN ambiguity handling."""

import numpy as np

from uuv_mode_aware_navigation.localization import TerrainMatcherV2
from uuv_mode_aware_navigation.maps import BathymetryMap
from uuv_mode_aware_navigation.sensor_models import AltimeterModel, AltimeterProfile


class FlatMap:
    metres_per_cell = 0.10
    def sample(self, x, y):
        return np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape) - 20.0
    def gradient_vector(self, x, y):
        shape = np.broadcast(np.asarray(x), np.asarray(y)).shape
        return np.zeros(shape), np.zeros(shape)


class RepetitiveMap:
    metres_per_cell = 0.10
    period_m = 1.0
    def sample(self, x, y):
        return -20.0 + 0.4 * np.sin(2.0 * np.pi * np.asarray(x))
    def gradient_vector(self, x, y):
        gx = 0.8 * np.pi * np.cos(2.0 * np.pi * np.asarray(x))
        return gx, np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape)


def _track(origin, heading):
    distance = np.arange(49) * 0.25
    direction = np.array([np.cos(heading), np.sin(heading)])
    return np.asarray(origin) + distance[:, None] * direction


def _profile(terrain, truth, heading, seed):
    xy = _track(truth, heading)
    return AltimeterModel(0.02).sample_profile(
        terrain, xy, np.full(49, -17.0), np.random.default_rng(seed)
    )


def test_repetitive_alias_is_not_accepted() -> None:
    terrain = RepetitiveMap()
    truth = np.array([0.0, 0.0])
    result = TerrainMatcherV2().match(
        terrain, _profile(terrain, truth, 0.0, 22_219_001),
        truth + np.array([1.7, 0.2]),
    )
    assert not result.success
    assert result.reason in {
        "ambiguous", "profile_inconsistent", "search_boundary", "unobservable"
    }


def test_flat_map_remains_unobservable() -> None:
    terrain = FlatMap()
    truth = np.array([0.0, 0.0])
    result = TerrainMatcherV2().match(
        terrain, _profile(terrain, truth, 0.7, 22_219_002), truth + np.array([0.4, -0.3])
    )
    assert not result.success
    assert result.reason == "unobservable"


def test_structured_relief_retains_an_accurate_fix() -> None:
    terrain = BathymetryMap(metres_per_cell=0.10)
    truth = np.array([30.0, 20.0])
    result = TerrainMatcherV2().match(
        terrain, _profile(terrain, truth, 1.2, 22_219_003),
        truth + np.array([0.8, -0.6]),
    )
    assert result.success, result.reason
    assert np.linalg.norm(result.position_xy_m - truth) < 0.10
    assert np.all(np.linalg.eigvalsh(result.covariance_m2) > 0.0)
