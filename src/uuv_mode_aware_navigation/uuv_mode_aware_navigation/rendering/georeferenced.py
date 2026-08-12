"""Pose-dependent renderer over a persistent world-coordinate texture."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..imaging import apply_water_column, seabed_texture
from ..optics import ChannelConfig, WaterState
from .base import CameraModel, CameraPose


class FootprintOutsideWorld(ValueError):
    """The requested camera footprint is not covered by the world texture.

    A distinct type, because leaving the surveyed patch is a *physical*
    condition -- there is no georeferenced imagery out there -- and callers
    should be able to treat it as loss of optical aiding rather than as a
    programming error. It subclasses ``ValueError`` so existing handlers that
    catch ``ValueError`` keep working unchanged.
    """


@dataclass(frozen=True)
class WorldTexture:
    """Deterministic texture indexed by horizontal world coordinates."""

    pixels: np.ndarray
    metres_per_pixel: float
    origin_x_m: float
    origin_y_m: float

    @classmethod
    def generate(
        cls,
        size_px: int = 1024,
        metres_per_pixel: float = 0.04,
        seed: int = 22_000_101,
    ) -> "WorldTexture":
        if size_px < 64:
            raise ValueError("world texture must be at least 64 pixels")
        if metres_per_pixel <= 0.0:
            raise ValueError("metres_per_pixel must be positive")
        pixels = seabed_texture(size=size_px, seed=seed, roughness=1.45)
        half_extent = 0.5 * (size_px - 1) * metres_per_pixel
        return cls(pixels, metres_per_pixel, -half_extent, -half_extent)

    def covers(self, x_m, y_m) -> bool:
        """Whether every supplied world coordinate lies inside the texture."""
        x = (np.asarray(x_m, dtype=float) - self.origin_x_m) / self.metres_per_pixel
        y = (np.asarray(y_m, dtype=float) - self.origin_y_m) / self.metres_per_pixel
        rows, cols = self.pixels.shape
        return not (np.any(x < 0.0) or np.any(x > cols - 1)
                    or np.any(y < 0.0) or np.any(y > rows - 1))

    def sample(self, x_m: np.ndarray, y_m: np.ndarray) -> np.ndarray:
        """Bilinearly sample world coordinates; reject out-of-map footprints."""
        x = (np.asarray(x_m, dtype=float) - self.origin_x_m) / self.metres_per_pixel
        y = (np.asarray(y_m, dtype=float) - self.origin_y_m) / self.metres_per_pixel
        rows, cols = self.pixels.shape
        if np.any(x < 0.0) or np.any(x > cols - 1) or np.any(y < 0.0) or np.any(y > rows - 1):
            raise FootprintOutsideWorld(
                "requested camera footprint leaves the world texture")
        x0 = np.floor(x).astype(int)
        y0 = np.floor(y).astype(int)
        x1 = np.minimum(x0 + 1, cols - 1)
        y1 = np.minimum(y0 + 1, rows - 1)
        wx = x - x0
        wy = y - y0
        return (
            (1.0 - wx) * (1.0 - wy) * self.pixels[y0, x0]
            + wx * (1.0 - wy) * self.pixels[y0, x1]
            + (1.0 - wx) * wy * self.pixels[y1, x0]
            + wx * wy * self.pixels[y1, x1]
        )


@dataclass
class GeoreferencedRenderer:
    """Nadir camera with pose-dependent footprint and existing water physics."""

    world: WorldTexture = field(default_factory=WorldTexture.generate)
    camera: CameraModel = field(default_factory=CameraModel)
    sensor_seed: int = 22_000_301
    add_sensor_noise: bool = True

    def clear_scene(self, pose: CameraPose) -> np.ndarray:
        if pose.altitude_m <= 0.0:
            raise ValueError("altitude_m must be positive")
        width_m = 2.0 * pose.altitude_m * np.tan(self.camera.horizontal_fov_rad / 2.0)
        height_m = width_m * self.camera.height_px / self.camera.width_px
        u = np.linspace(-width_m / 2.0, width_m / 2.0, self.camera.width_px)
        v = np.linspace(-height_m / 2.0, height_m / 2.0, self.camera.height_px)
        uu, vv = np.meshgrid(u, v, indexing="xy")
        c, s = np.cos(pose.yaw_rad), np.sin(pose.yaw_rad)
        x = pose.x_m + c * uu - s * vv
        y = pose.y_m + s * uu + c * vv
        return self.world.sample(x, y)

    def _rng(self, pose: CameraPose) -> np.random.Generator | None:
        if not self.add_sensor_noise:
            return None
        quantised = (
            int(round(pose.x_m * 1000.0)),
            int(round(pose.y_m * 1000.0)),
            int(round(pose.altitude_m * 1000.0)),
            int(round(pose.yaw_rad * 10000.0)),
        )
        seed = self.sensor_seed
        for value in quantised:
            seed = (seed * 1_000_003 + value) % (2**32)
        return np.random.default_rng(seed)

    def render(
        self,
        pose: CameraPose,
        water: WaterState,
        config: ChannelConfig,
    ) -> np.ndarray:
        return apply_water_column(
            self.clear_scene(pose), water, pose.altitude_m, config,
            rng=self._rng(pose),
        )

