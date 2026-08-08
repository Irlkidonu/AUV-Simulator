"""Optical feedback: image formation and image-only quality estimation.

These tests exist because "optical feedback" is in the paper's title, and the
claim has to be checkable rather than asserted. Two of them guard failure modes
that were present in the first implementation and that no other test caught.
"""

import numpy as np
import pytest

from uuv_mode_aware_navigation.imaging import (
    OpticalFeedback,
    analyse_image,
    render_patch,
    seabed_texture,
)
from uuv_mode_aware_navigation.optics import (
    CAMERA_COAXIAL,
    CAMERA_OFFAXIS,
    CONFIGURATIONS,
    LIDAR,
    WaterState,
    channel_response,
)
from uuv_mode_aware_navigation.sensors import (
    FaultKind,
    SensorSuite,
    optical_loss_schedule,
)


def _dataset(turbidities, altitudes, texture_seeds, seed):
    rng = np.random.default_rng(seed)
    textures = [seabed_texture(seed=s) for s in texture_seeds]
    features, quality = [], []
    for c in turbidities:
        for altitude in altitudes:
            for config in CONFIGURATIONS:
                for texture in textures:
                    water = WaterState(c=float(c))
                    frame = render_patch(water, altitude, config, texture, rng)
                    features.append(analyse_image(frame))
                    quality.append(channel_response(water, altitude, config).quality)
    return features, quality


@pytest.fixture(scope="module")
def fitted():
    features, quality = _dataset(
        np.linspace(0.15, 2.2, 15),
        (1.0, 1.5, 2.0, 2.5, 3.0, 4.0),
        [20_000_101 + i for i in range(4)],
        20_000_201,
    )
    return OpticalFeedback().fit(features, quality)


# ---------------------------------------------------------------------------
# Image formation
# ---------------------------------------------------------------------------
def test_rendered_image_degrades_with_turbidity():
    texture = seabed_texture()
    rng = np.random.default_rng(7)
    values = [
        analyse_image(
            render_patch(WaterState(c=c), 3.0, CAMERA_OFFAXIS, texture, rng)
        ).structure_to_noise
        for c in (0.2, 0.5, 0.8, 1.2)
    ]
    assert values == sorted(values, reverse=True), (
        f"structure-to-noise must fall as water clouds: {values}"
    )


def test_offaxis_lighting_beats_coaxial_at_the_same_water_and_altitude():
    """The geometry claim, visible in the image rather than only in the model."""
    texture = seabed_texture()
    rng = np.random.default_rng(11)
    water, altitude = WaterState(c=0.6), 2.0
    off = analyse_image(render_patch(water, altitude, CAMERA_OFFAXIS, texture, rng))
    co = analyse_image(render_patch(water, altitude, CAMERA_COAXIAL, texture, rng))
    assert off.structure_contrast > co.structure_contrast


def test_render_is_deterministic_without_a_generator():
    texture = seabed_texture()
    a = render_patch(WaterState(c=0.7), 2.0, LIDAR, texture)
    b = render_patch(WaterState(c=0.7), 2.0, LIDAR, texture)
    assert np.array_equal(a, b)


# ---------------------------------------------------------------------------
# The metric trap
# ---------------------------------------------------------------------------
def test_a_noise_only_frame_is_not_scored_as_high_contrast(fitted):
    """Regression test for a metric that inverted exactly where it mattered.

    Beyond the imaging range the return decays below the detector floor and the
    frame is noise on a small mean. Measuring contrast as ``std / mean`` then
    *rises*, and the first implementation reported an apparent contrast of 1.46
    for a laser channel at ten attenuation lengths -- a reading that would have
    told the manager to keep using a dead sensor.
    """
    rng = np.random.default_rng(23)
    mean_level = 1e-4
    noise_frame = np.abs(rng.normal(mean_level, mean_level * 0.5, (96, 96)))

    structured = seabed_texture()

    noise_features = analyse_image(noise_frame)
    real_features = analyse_image(structured)

    assert noise_features.structure_to_noise < real_features.structure_to_noise
    assert fitted.predict(noise_frame) < 0.5, (
        "a frame containing no scene was rated usable"
    )


# ---------------------------------------------------------------------------
# Agreement with the analytic index -- the bridge claim
# ---------------------------------------------------------------------------
def test_image_only_estimate_tracks_the_analytic_index(fitted):
    """The claim licensing a headless campaign: pixels recover the index.

    Evaluated on turbidities, altitudes, textures and a generator seed that were
    used neither for fitting nor for choosing the feature expansion. If this
    degrades, the headless campaign and the Gazebo demonstrator are no longer
    studying the same system, and the manuscript must say so.
    """
    features, quality = _dataset(
        np.linspace(0.25, 2.15, 13),
        (1.1, 1.8, 2.7, 3.6),
        [40_000_700 + i for i in range(3)],
        40_000_777,
    )
    report = fitted.agreement(features, quality)
    assert report["r_squared"] > 0.85, report
    assert report["rmse"] < 0.15, report


def test_estimator_never_sees_the_water_state(fitted):
    """Rule N2, for the optical path: prediction takes pixels and nothing else."""
    import inspect

    params = set(inspect.signature(OpticalFeedback.predict).parameters)
    assert params == {"self", "image"}, params
    params = set(inspect.signature(analyse_image).parameters)
    assert "water" not in params and "c" not in params


# ---------------------------------------------------------------------------
# The closed loop
# ---------------------------------------------------------------------------
def test_sensor_suite_reports_image_derived_quality_when_feedback_is_enabled(fitted):
    """With feedback on, the observable must come from a frame, not the model."""
    water, altitude = WaterState(c=0.9), 2.5
    truth = np.array([0.0, 0.0, -17.5])

    plain = SensorSuite(seed=20_000_303)
    fed = SensorSuite(seed=20_000_303, optical_feedback=fitted)

    a = plain.sample(0.0, truth, np.zeros(3), np.zeros(3), altitude, water)
    b = fed.sample(0.0, truth, np.zeros(3), np.zeros(3), altitude, water)

    analytic = channel_response(water, altitude, CAMERA_OFFAXIS).quality
    assert a.optical_quality == pytest.approx(analytic)
    assert b.optical_quality != pytest.approx(analytic), (
        "feedback enabled but the reported quality is still the analytic index"
    )
    assert 0.0 <= b.optical_quality <= 1.0


def test_blackout_reports_zero_quality_not_a_stale_estimate(fitted):
    """A sensor producing no frame must report no quality."""
    suite = SensorSuite(
        schedule=optical_loss_schedule(5.0, 20.0),
        seed=20_000_404,
        optical_feedback=fitted,
    )
    reading = suite.sample(
        10.0, np.array([0.0, 0.0, -17.0]), np.zeros(3), np.zeros(3),
        3.0, WaterState(c=0.2),
    )
    assert reading.optical_position_m is None
    assert reading.optical_quality == 0.0
