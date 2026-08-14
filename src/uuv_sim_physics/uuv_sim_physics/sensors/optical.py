"""Underwater optical observation model, applied to the rendered image.

    Gazebo scene -> camera renderer -> camera/raw -> this -> camera/underwater

This module takes **pixels in and pixels out**. It has no access to vehicle
pose, dock pose or any simulator state, and it must never acquire any: it is
listed in ``privileged.OBSERVATION_PRODUCERS``, so adding a privileged import
here fails ``test_ground_truth_isolation.py``. That is the point. An
"underwater image" reconstructed from where the dock actually is would be an
answer wearing a photograph's clothes.

Model
-----
Standard single-scattering image-formation model for a linear camera:

    I(x) = J(x) * t(x)  +  B * (1 - t(x))

with transmittance from Beer-Lambert over the path length to the scene point,

    t(x) = exp(-c * z(x))

``J`` is the rendered radiance, ``B`` the veiling-light (backscatter) colour,
``c`` the beam attenuation coefficient in m^-1 and ``z`` the path length. Two
further effects are applied because they dominate the appearance of artificial
lights in murk and are cheap:

* **forward scattering / bloom** -- a Gaussian blur whose radius grows with
  optical depth, mixed in proportionally, which is what turns an LED into a
  halo;
* **contrast compression** toward the veiling light, the visible consequence of
  the additive term above.

Depth is not available from a monochrome colour image, so ``z`` defaults to a
uniform standoff supplied by the caller. Where a depth buffer is available it
can be passed in and the model becomes range-dependent per pixel. This is a
documented approximation, not a hidden one.

Parameters are **continuous**. The clear / medium / turbid levels a research
study might define are not encoded here; this module supplies the mechanism and
nothing else.

No NTU mapping is claimed. ``c`` is a beam attenuation coefficient in m^-1;
converting it to NTU would require a calibration this project has not done.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = ["WaterColumn", "degrade", "contrast_metrics"]


@dataclass(frozen=True)
class WaterColumn:
    """Continuous parameters of the optical observation model.

    ``beam_attenuation`` is the severity control. Everything else shapes how the
    severity manifests and is normally left alone.
    """

    #: Beam attenuation coefficient c = a + b, m^-1. The severity parameter.
    beam_attenuation: float = 0.20
    #: Veiling light colour, linear RGB in [0, 1]. Blue-green, as water is.
    veiling_rgb: tuple[float, float, float] = (0.12, 0.28, 0.32)
    #: Share of attenuation that is scattering rather than absorption, b/c.
    #: Drives bloom: absorption dims, scattering smears.
    scattering_fraction: float = 0.62
    #: Blur sigma in pixels at one optical depth.
    bloom_sigma_px_per_tau: float = 2.4
    #: Ceiling on the bloom mix, so a very murky image blurs but does not vanish.
    bloom_max_mix: float = 0.75
    #: Default scene path length when no depth buffer is supplied, metres.
    default_range_m: float = 3.0

    def optical_depth(self, range_m: float | np.ndarray) -> np.ndarray:
        """tau = c * z, in attenuation lengths."""
        return self.beam_attenuation * np.asarray(range_m, dtype=float)

    def transmittance(self, range_m: float | np.ndarray) -> np.ndarray:
        return np.exp(-self.optical_depth(range_m))


def _gaussian_blur(image: np.ndarray, sigma: float) -> np.ndarray:
    """Separable Gaussian. Written out to avoid a SciPy dependency."""
    if sigma <= 1e-3:
        return image
    radius = max(1, int(round(3.0 * sigma)))
    offsets = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * (offsets / sigma) ** 2)
    kernel /= kernel.sum()

    padded = np.pad(image, ((radius, radius), (radius, radius), (0, 0)),
                    mode="edge")
    # Zeros, not np.empty_like: this accumulates, and uninitialised memory made
    # the transform silently non-deterministic between runs.
    intermediate = np.zeros((image.shape[0], padded.shape[1], image.shape[2]))
    for index, weight in enumerate(kernel):
        intermediate += weight * padded[index:index + image.shape[0], :, :]
    out = np.zeros(image.shape, dtype=float)
    for index, weight in enumerate(kernel):
        out += weight * intermediate[:, index:index + image.shape[1], :]
    return out


def degrade(image: np.ndarray, water: WaterColumn,
            range_m: float | np.ndarray | None = None) -> np.ndarray:
    """Apply the underwater observation model to a rendered RGB image.

    ``image`` is uint8 or float RGB, shape (H, W, 3). Returns the same dtype.
    Nothing but pixels enters this function.
    """
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected an (H, W, 3) RGB image, got {image.shape}")

    was_integer = np.issubdtype(image.dtype, np.integer)
    scene = image.astype(float) / (255.0 if was_integer else 1.0)

    path = water.default_range_m if range_m is None else range_m
    tau = water.optical_depth(path)
    transmittance = np.exp(-np.asarray(tau, dtype=float))
    if transmittance.ndim == 2:
        transmittance = transmittance[..., None]

    # Forward scattering first: bloom is a property of the light reaching the
    # sensor, so it is applied to the scene radiance before the veiling term
    # washes it out.
    tau_scalar = float(np.mean(tau))
    sigma = water.bloom_sigma_px_per_tau * tau_scalar * water.scattering_fraction
    if sigma > 1e-3:
        mix = min(water.bloom_max_mix,
                  water.scattering_fraction * (1.0 - float(np.exp(-tau_scalar))))
        scene = (1.0 - mix) * scene + mix * _gaussian_blur(scene, sigma)

    veiling = np.asarray(water.veiling_rgb, dtype=float).reshape(1, 1, 3)
    out = scene * transmittance + veiling * (1.0 - transmittance)

    out = np.clip(out, 0.0, 1.0)
    return (out * 255.0).astype(np.uint8) if was_integer else out


# --- infrastructure metrics -------------------------------------------------
# Used by the turbidity sanity test. Image-domain only; no scene knowledge.

def contrast_metrics(image: np.ndarray) -> dict:
    """Simple visibility statistics for an RGB image."""
    data = image.astype(float) / (255.0 if np.issubdtype(image.dtype, np.integer)
                                  else 1.0)
    luminance = data @ np.array([0.2126, 0.7152, 0.0722])

    # Michelson contrast on robust percentiles, so one bright pixel cannot
    # dominate the figure.
    high, low = np.percentile(luminance, 99.0), np.percentile(luminance, 1.0)
    michelson = (high - low) / (high + low + 1e-9)

    gy, gx = np.gradient(luminance)
    edge_strength = float(np.sqrt(gx ** 2 + gy ** 2).mean())

    return {"rms_contrast": float(luminance.std()),
            "michelson_contrast": float(michelson),
            "edge_strength": edge_strength,
            "mean_luminance": float(luminance.mean()),
            "p99_luminance": float(high)}
