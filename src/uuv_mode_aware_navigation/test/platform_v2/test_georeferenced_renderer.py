"""Scientific properties of the platform-v2 pose-dependent renderer."""

from __future__ import annotations

import numpy as np

from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS, WaterState
from uuv_mode_aware_navigation.rendering import CameraModel, CameraPose
from uuv_mode_aware_navigation.rendering.georeferenced import (
    GeoreferencedRenderer,
    WorldTexture,
)


def _renderer(noise=False):
    return GeoreferencedRenderer(
        world=WorldTexture.generate(size_px=512, metres_per_pixel=0.04),
        camera=CameraModel(width_px=96, height_px=96),
        add_sensor_noise=noise,
    )


def test_render_is_bitwise_deterministic() -> None:
    renderer = _renderer(noise=True)
    pose = CameraPose(1.0, -2.0, 3.0, 0.1)
    first = renderer.render(pose, WaterState(c=0.2), CAMERA_OFFAXIS)
    second = renderer.render(pose, WaterState(c=0.2), CAMERA_OFFAXIS)
    np.testing.assert_array_equal(first, second)


def test_translation_changes_visible_content() -> None:
    renderer = _renderer()
    a = renderer.clear_scene(CameraPose(0.0, 0.0, 3.0))
    b = renderer.clear_scene(CameraPose(0.5, 0.0, 3.0))
    assert float(np.mean(np.abs(a - b))) > 0.005


def test_yaw_rotates_footprint_content() -> None:
    renderer = _renderer()
    a = renderer.clear_scene(CameraPose(0.0, 0.0, 3.0, 0.0))
    b = renderer.clear_scene(CameraPose(0.0, 0.0, 3.0, np.pi / 2.0))
    np.testing.assert_allclose(b, np.rot90(a, 1), atol=2e-3)


def test_altitude_scales_footprint() -> None:
    renderer = _renderer()
    near = renderer.clear_scene(CameraPose(0.0, 0.0, 1.0))
    far = renderer.clear_scene(CameraPose(0.0, 0.0, 3.0))
    assert not np.array_equal(near, far)
    assert float(np.std(near)) != float(np.std(far))

