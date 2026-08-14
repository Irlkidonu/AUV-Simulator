"""The underwater optical observation model: M4.2 and M4.7.

Pixels in, pixels out. These tests never construct an image from a pose, and
the isolation test guarantees the module could not do so even if asked.
"""

from __future__ import annotations

import numpy as np
import pytest

from uuv_sim_physics.sensors import WaterColumn, contrast_metrics, degrade

SEVERITIES = (0.0, 0.20, 0.60, 1.20, 2.00)


@pytest.fixture
def scene() -> np.ndarray:
    """A dark background with a bright patch, standing in for a lit dock."""
    rng = np.random.default_rng(7)
    image = (rng.uniform(0.05, 0.15, (96, 128, 3)) * 255).astype(np.uint8)
    image[40:56, 50:78] = 250
    return image


def test_clear_water_is_close_to_identity(scene) -> None:
    assert np.array_equal(degrade(scene, WaterColumn(beam_attenuation=0.0)), scene)


def test_visibility_decreases_monotonically_with_severity(scene) -> None:
    """M4.7: severity up, optical visibility down. No exceptions."""
    metrics = [contrast_metrics(degrade(scene, WaterColumn(beam_attenuation=c)))
               for c in SEVERITIES]
    for key in ("rms_contrast", "michelson_contrast", "edge_strength"):
        series = [m[key] for m in metrics]
        assert all(a >= b - 1e-12 for a, b in zip(series, series[1:])), \
            f"{key} is not monotonically decreasing: {series}"
    assert metrics[0]["rms_contrast"] > 10 * metrics[-2]["rms_contrast"]


def test_image_tends_to_the_veiling_light(scene) -> None:
    """As transmittance -> 0 the observation becomes the backscatter colour."""
    water = WaterColumn(beam_attenuation=6.0)
    out = degrade(scene, water).astype(float) / 255.0
    expected = np.asarray(water.veiling_rgb)
    assert np.allclose(out.reshape(-1, 3).mean(axis=0), expected, atol=0.02)


def test_transmittance_follows_beer_lambert() -> None:
    water = WaterColumn(beam_attenuation=0.5)
    assert water.transmittance(0.0) == pytest.approx(1.0)
    assert water.transmittance(2.0) == pytest.approx(np.exp(-1.0))
    assert water.optical_depth(4.0) == pytest.approx(2.0)


def test_range_dependence_is_supported(scene) -> None:
    """A per-pixel depth buffer makes the model range-dependent."""
    near = degrade(scene, WaterColumn(beam_attenuation=0.6), range_m=1.0)
    far = degrade(scene, WaterColumn(beam_attenuation=0.6), range_m=8.0)
    assert contrast_metrics(near)["rms_contrast"] > \
        contrast_metrics(far)["rms_contrast"]

    depth = np.full(scene.shape[:2], 1.0)
    depth[:, 64:] = 8.0
    graded = degrade(scene, WaterColumn(beam_attenuation=0.6), range_m=depth)
    left = contrast_metrics(graded[:, :64])["rms_contrast"]
    right = contrast_metrics(graded[:, 64:])["rms_contrast"]
    assert left > right, "the nearer half should be clearer"


def test_dtype_and_shape_are_preserved(scene) -> None:
    out = degrade(scene, WaterColumn(beam_attenuation=0.4))
    assert out.shape == scene.shape and out.dtype == scene.dtype
    floats = scene.astype(float) / 255.0
    assert degrade(floats, WaterColumn(beam_attenuation=0.4)).dtype == floats.dtype


def test_rejects_non_rgb_input() -> None:
    with pytest.raises(ValueError):
        degrade(np.zeros((16, 16)), WaterColumn())


def test_no_ntu_claim_anywhere_in_the_module() -> None:
    """The severity parameter is c in m^-1; no NTU calibration is claimed."""
    from uuv_sim_physics.sensors import optical
    source = __import__("pathlib").Path(optical.__file__).read_text()
    lowered = source.lower()
    assert "ntu" not in lowered or "no ntu mapping is claimed" in lowered
    assert "beam attenuation coefficient" in lowered
