"""Characterisation tests for the platform-v2 TRN feasibility implementation."""

import numpy as np

from uuv_mode_aware_navigation.localization import TerrainMatcher
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
        return -20.0 + 0.4 * np.sin(2.0 * np.pi * np.asarray(x) / self.period_m)
    def gradient_vector(self, x, y):
        gx = 0.8 * np.pi * np.cos(2.0 * np.pi * np.asarray(x) / self.period_m)
        return gx, np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape)


def _track(origin=(-4.0, -3.0), heading=0.7):
    distance = np.arange(49) * 0.25
    direction = np.array([np.cos(heading), np.sin(heading)])
    return np.asarray(origin) + distance[:, None] * direction


def test_informative_map_recovers_initial_error() -> None:
    bathy = BathymetryMap(metres_per_cell=0.10)
    # The central survey patch is deliberately gentle and is not an
    # informative TRN reference in every heading.  Exercise the matcher's
    # positive path on the structured outer relief; flat and repetitive maps
    # below separately exercise honest unavailability and ambiguity.
    truth = np.array([30.0, 20.0])
    xy = _track(truth)
    profile = AltimeterModel(0.02).sample_profile(
        bathy, xy, np.full(49, -17.0), np.random.default_rng(22_199_001)
    )
    result = TerrainMatcher().match(bathy, profile, truth + np.array([1.2, -0.8]))
    assert result.success, result.reason
    assert np.linalg.norm(result.position_xy_m - truth) < 0.10
    assert np.all(np.linalg.eigvalsh(result.covariance_m2) > 0.0)


def test_flat_map_is_unobservable() -> None:
    relative = _track((0.0, 0.0))
    profile = AltimeterProfile(
        relative - relative[0], np.full(49, 3.0), np.full(49, -17.0), 0.02
    )
    result = TerrainMatcher().match(FlatMap(), profile, np.array([0.5, -0.5]))
    assert not result.success
    assert result.reason == "unobservable"


def test_repetitive_map_is_rejected_as_ambiguous() -> None:
    terrain = RepetitiveMap()
    truth = np.array([0.0, 0.0])
    xy = _track(truth, heading=0.0)
    profile = AltimeterModel(0.02).sample_profile(
        terrain, xy, np.full(49, -17.0), np.random.default_rng(22_199_002)
    )
    result = TerrainMatcher().match(terrain, profile, np.array([0.3, 0.0]))
    assert not result.success
    assert result.reason in {"ambiguous", "unobservable"}
