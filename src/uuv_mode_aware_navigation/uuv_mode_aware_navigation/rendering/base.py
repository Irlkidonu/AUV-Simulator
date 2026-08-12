"""Public renderer contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..optics import ChannelConfig, WaterState


@dataclass(frozen=True)
class CameraPose:
    x_m: float
    y_m: float
    altitude_m: float
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class CameraModel:
    width_px: int = 192
    height_px: int = 192
    horizontal_fov_rad: float = np.deg2rad(60.0)

    def __post_init__(self) -> None:
        if self.width_px < 16 or self.height_px < 16:
            raise ValueError("camera dimensions must be at least 16 pixels")
        if not 0.0 < self.horizontal_fov_rad < np.pi:
            raise ValueError("horizontal_fov_rad must lie in (0, pi)")


class SceneRenderer(Protocol):
    def render(
        self,
        pose: CameraPose,
        water: WaterState,
        config: ChannelConfig,
    ) -> np.ndarray:
        ...

