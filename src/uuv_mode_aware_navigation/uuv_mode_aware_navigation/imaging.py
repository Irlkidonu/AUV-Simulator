"""Optical feedback: estimating water condition from the image itself.

The title of this study says *optical feedback*, and that word has to mean
something specific. It means the vehicle looks at the picture it already has and
works out, unaided, whether that picture can still support navigation. It does
not mean a turbidity number handed to the navigator by the simulator.

That distinction is not pedantry. An earlier version of this package computed
``optical_quality`` inside the propagation model and passed it straight to the
manager. Every decision the manager made was then downstream of a quantity no
real vehicle can measure, and the closed loop the paper claims to study was open
at exactly the point of interest. Nothing in the reported numbers would have been
wrong, and the study would still not have been about what it said it was about.

What this module provides
-------------------------
1. :func:`render_patch` — an image formation model, so a seabed view can be
   synthesised at a given water state and altitude. It reuses the propagation
   functions in :mod:`optics` rather than restating them, so the image and the
   availability prediction cannot drift apart.
2. :func:`analyse_image` — three no-reference image statistics, computed from
   pixels alone.
3. :class:`OpticalFeedback` — a fitted map from those statistics to the same
   [0, 1] navigation quality index the propagation model produces.

The same :class:`OpticalFeedback` instance runs on synthetic patches in the
headless campaign and on rendered camera frames in the Gazebo demonstrator. That
is deliberate: it is the single component shared by both halves of the study, and
its agreement with the analytic model is what licenses using the analytic model
for the 640-run campaign in the first place.

Image formation
---------------
The standard three-component decomposition of light reaching an underwater
camera — direct, forward-scattered, and backscattered [R6] — reduces to

    I = blur(J, sigma(tau)) * t  +  A(c) * (1 - t)

where ``J`` is the seabed radiance, ``t = exp(-c * L)`` is transmission over the
two-way path, ``A(c)`` is the veiling light produced by backscatter from the
volume shared by the illumination and the field of view, and the blur width grows
with optical depth as forward scattering redistributes small-angle energy.

Two properties of that expression drive everything downstream. Attenuation scales
the scene *modulation* toward zero, and veiling light adds a spatially smooth
*offset*. Contrast therefore collapses from both ends at once, which is why
backscatter and not absorption sets the practical imaging limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from .optics import (
    ALTITUDE_NOMINAL_M,
    CAMERA_OFFAXIS,
    CONTRAST_FLOOR,
    SEABED_REFLECTANCE,
    WATER_LEVELS,
    ChannelConfig,
    WaterState,
    backscatter_radiance,
    channel_response,
    path_length,
    signal_radiance,
    slant_range,
)

__all__ = [
    "ImageFeatures",
    "seabed_texture",
    "render_patch",
    "apply_water_column",
    "analyse_image",
    "OpticalFeedback",
    "FORWARD_SCATTER_BLUR_PER_TAU",
    "REFERENCE_RADIANCE",
]

#: Growth of the forward-scattering point-spread width, in pixels per attenuation
#: length. Forward scattering redistributes small-angle energy, so the image
#: softens with optical depth before it disappears. The value sets how quickly
#: gradient energy decays relative to contrast; it is a property of the renderer,
#: not of the estimator, and the estimator is never given it.
FORWARD_SCATTER_BLUR_PER_TAU = 0.9

#: Detector read noise, as a fraction of a fixed reference radiance. Matches
#: ``optics.READ_NOISE``.
#:
#: It must be referenced to a **constant**, not to the veiling level of the
#: current scene. Scaling noise with backscatter gives a low-backscatter channel
#: an artificially clean image: the collimated laser then retained an apparent
#: contrast of 0.16 at ten attenuation lengths, where its returned signal is
#: ``exp(-10.2)`` of the emitted level and would in reality be far below any
#: detector floor. Read noise is a property of the sensor, and a signal that has
#: decayed beneath it is gone regardless of how little backscatter competes
#: with it.
SENSOR_NOISE = 0.01

#: Patch size used by the headless campaign. Large enough for the statistics to
#: be stable, small enough that fitting is fast.
PATCH_PIXELS = 96

#: Fixed radiance scale against which read noise is defined: the seabed return in
#: the clearest declared water at nominal survey altitude, on the reference
#: camera. A constant of the instrument, not of the scene.
REFERENCE_RADIANCE = signal_radiance(
    WaterState(c=WATER_LEVELS[0]), ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS
)


def seabed_texture(
    size: int = PATCH_PIXELS, seed: int = 20_000_101, roughness: float = 1.6
) -> np.ndarray:
    """A deterministic seabed reflectance patch with broadband spatial statistics.

    Natural seabed has structure at many scales, and an estimator tuned on a
    single-scale texture would be measuring the texture rather than the water.
    The patch is generated by summing octaves of white noise at successive
    scales, giving an approximately power-law spatial spectrum, and is then
    scaled to a mean of ``SEABED_REFLECTANCE``.
    """
    rng = np.random.default_rng(seed)
    field = np.zeros((size, size), dtype=float)
    amplitude = 1.0
    scale = 2
    while scale <= size:
        coarse = rng.normal(0.0, 1.0, (scale, scale))
        # ceil, then crop: `size` need not be a power of two, and integer
        # division silently produces an undersized octave when it is not.
        repeat = int(math.ceil(size / scale))
        octave = np.repeat(np.repeat(coarse, repeat, axis=0), repeat, axis=1)
        field += amplitude * octave[:size, :size]
        amplitude /= roughness
        scale *= 2
    field -= field.min()
    if field.max() > 0:
        field /= field.max()
    # Centre on the declared reflectance with realistic modulation depth.
    return SEABED_REFLECTANCE * (0.4 + 1.2 * field)


def _gaussian_blur(image: np.ndarray, sigma: float) -> np.ndarray:
    """Separable Gaussian blur with edge replication. No SciPy dependency."""
    if sigma <= 1e-6:
        return image
    radius = max(1, int(math.ceil(3.0 * sigma)))
    x = np.arange(-radius, radius + 1, dtype=float)
    kernel = np.exp(-0.5 * (x / sigma) ** 2)
    kernel /= kernel.sum()
    padded = np.pad(image, radius, mode="edge")
    # Separable convolution as two contractions over sliding windows. Identical
    # to convolving each row and column in turn, but without a Python-level loop
    # per line -- which dominated campaign runtime once the renderer ran inside
    # the decision loop.
    rows = sliding_window_view(padded, kernel.size, axis=1)
    out = rows @ kernel
    cols = sliding_window_view(out, kernel.size, axis=0)
    return np.einsum("ijk,k->ij", cols, kernel)


def render_patch(
    water: WaterState,
    altitude_m: float,
    config: ChannelConfig,
    texture: Optional[np.ndarray] = None,
    rng: Optional[np.random.Generator] = None,
    tilt_rad: float = 0.0,
) -> np.ndarray:
    """Render the seabed as this configuration would see it, in this water.

    Physics is taken from :mod:`optics`; nothing is restated here. In particular
    the veiling term uses the same common-volume backscatter integral and the
    same solved normalisation constant as the availability model, so an image
    that looks unusable and an availability prediction of zero come from one set
    of equations rather than two that happen to agree.
    """
    if texture is None:
        texture = seabed_texture()
    return apply_water_column(texture, water, altitude_m, config, rng, tilt_rad)


def apply_water_column(
    scene: np.ndarray,
    water: WaterState,
    altitude_m: float,
    config: ChannelConfig,
    rng: Optional[np.random.Generator] = None,
    tilt_rad: float = 0.0,
) -> np.ndarray:
    """Apply the water column to a clear-water scene.

    ``scene`` is reflectance in [0, 1]: either a synthetic seabed patch or a
    frame rendered by Gazebo, normalised. The same function serves both, which
    is the point -- the demonstrator degrades a real rendered image through
    exactly the model the campaign uses, so the optical feedback estimator sees
    consistent physics whichever half of the study it is running in.
    """
    band = water.for_wavelength(config.spectral_factor)
    c = band.c
    r = slant_range(altitude_m, tilt_rad)
    tau = c * path_length(altitude_m, tilt_rad)
    transmission = math.exp(-tau)

    # Forward scattering softens the image before it extinguishes it.
    blurred = _gaussian_blur(np.asarray(scene, dtype=float),
                             FORWARD_SCATTER_BLUR_PER_TAU * tau)
    direct = blurred * transmission / (r * r)

    # Backscatter veiling: identical expression to the availability model's.
    veiling = backscatter_radiance(water, altitude_m, config, tilt_rad)
    image = direct + veiling

    if rng is not None:
        image = image + rng.normal(
            0.0, SENSOR_NOISE * REFERENCE_RADIANCE, image.shape
        )
    return np.clip(image, 0.0, None)


@dataclass(frozen=True)
class ImageFeatures:
    """No-reference statistics computed from pixels alone.

    Nothing here may depend on knowing the water state, the altitude, or the
    scene. These are exactly the quantities a camera-only observer has.
    """

    #: Modulation that survives smoothing, relative to mean level. Falls as
    #: attenuation reduces the signal and as veiling light raises the floor --
    #: the two mechanisms that limit underwater imaging, entering the same
    #: number from both sides.
    structure_contrast: float
    #: Ratio of smoothed modulation to residual high-frequency energy. Separates
    #: scene structure from sensor noise, which raw contrast cannot do.
    structure_to_noise: float
    #: Absolute smoothed modulation, in units of the instrument's reference
    #: radiance. This is the feature that carries **transmittance**, and omitting
    #: it costs most of the estimator's accuracy: contrast normalised by the mean
    #: is blind to how much light came back at all, yet the navigation quality
    #: index depends on transmittance directly. A camera of known gain and
    #: exposure observes absolute level, so using it requires no extra hardware.
    structure_absolute: float
    #: Dark-channel level: the darkest local value present, relative to the mean.
    #: Rises toward one as veiling light floods the scene, which is the
    #: signature dehazing methods exploit.
    dark_channel: float

    def as_vector(self) -> np.ndarray:
        return np.array(
            [
                self.structure_contrast,
                self.structure_to_noise,
                self.structure_absolute,
                self.dark_channel,
            ],
            dtype=float,
        )


#: Scale separating scene structure from sensor noise, in pixels. Scene content
#: survives smoothing at this width; white detector noise does not.
STRUCTURE_SCALE_PX = 1.5


def analyse_image(image: np.ndarray, patch: int = 8) -> ImageFeatures:
    """Compute the no-reference features from a single-channel image.

    Why the features are scale-separated
    ------------------------------------
    The obvious contrast measure, ``std(I) / mean(I)``, is wrong here and fails
    in the direction that matters most. Beyond the imaging range the returned
    signal decays below the detector floor and the frame becomes noise on a
    small mean -- at which point ``std / mean`` *rises*, reporting an apparent
    contrast of 1.46 for a laser channel at ten attenuation lengths, where in
    truth nothing is visible at all. A metric that scores an empty frame as
    high-contrast would tell the manager to keep using a dead channel.

    Structure and noise are therefore separated by spatial scale before either
    is measured. Scene content, already softened by forward scattering, survives
    smoothing; white detector noise does not. The smoothed image supplies the
    modulation term, and the residual supplies the noise term.
    """
    img = np.asarray(image, dtype=float)
    if img.ndim == 3:  # accept RGB from a real camera
        img = img.mean(axis=2)
    mean = float(img.mean())
    if mean <= 1e-12:
        return ImageFeatures(0.0, 0.0, 0.0, 1.0)

    smooth = _gaussian_blur(img, STRUCTURE_SCALE_PX)
    residual = img - smooth

    structure = float(smooth.std())
    noise = float(residual.std())

    structure_contrast = structure / mean
    structure_to_noise = structure / max(noise, 1e-12)
    structure_absolute = structure / REFERENCE_RADIANCE

    # Dark channel computed on the smoothed image, so a single dark noise pixel
    # cannot masquerade as recovered scene detail.
    h, w = smooth.shape
    ph, pw = max(1, h // patch), max(1, w // patch)
    trimmed = smooth[: ph * patch, : pw * patch]
    blocks = trimmed.reshape(ph, patch, pw, patch)
    dark = float(blocks.min(axis=(1, 3)).mean() / mean)

    return ImageFeatures(
        structure_contrast, structure_to_noise, structure_absolute, dark
    )


class OpticalFeedback:
    """Estimates the navigation quality index from an image alone.

    The map is fitted on development data by least squares in log space, which
    keeps it monotone in contrast and gradient energy without constraining the
    fit by hand. Fitting is on synthetic patches whose water state is known;
    *using* it requires only pixels.

    What the fit is allowed to see
    ------------------------------
    Training pairs an image with the analytic quality index for the water state
    that produced it. The estimator is therefore learning to recover a quantity
    the propagation model defines, from pixels — which is precisely the claim
    that has to hold for the headless campaign and the Gazebo demonstrator to be
    studying the same system. :meth:`agreement` reports how well it does, and
    that number belongs in the paper whatever it turns out to be.
    """

    def __init__(self) -> None:
        self.coefficients: Optional[np.ndarray] = None
        self._residual_std: float = float("nan")

    @staticmethod
    def _design(features: Sequence[ImageFeatures]) -> np.ndarray:
        """Feature expansion. Log terms plus four interactions.

        The interactions are not decoration. Navigation quality is a *product* of
        contrast and the square root of transmittance, so in log space the
        governing relationship is bilinear, and a purely additive model cannot
        represent it. Adding the four cross terms below moved agreement with the
        analytic index from 0.79 to 0.89 on textures and altitudes withheld from
        the fit, with only a four-point train/withheld gap on 1080 samples.
        """
        rows = []
        for f in features:
            lc = math.log(max(f.structure_contrast, 1e-6))
            ls = math.log(max(f.structure_to_noise, 1e-6))
            la = math.log(max(f.structure_absolute, 1e-9))
            dk = f.dark_channel
            rows.append([1.0, lc, ls, la, dk, lc * la, ls * ls, la * la, lc * dk])
        return np.asarray(rows, dtype=float)

    def fit(
        self, features: Sequence[ImageFeatures], quality: Sequence[float]
    ) -> "OpticalFeedback":
        design = self._design(features)
        target = np.asarray(list(quality), dtype=float)
        coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
        self.coefficients = coefficients
        self._residual_std = float(np.std(design @ coefficients - target))
        return self

    def predict_features(self, features: ImageFeatures) -> float:
        if self.coefficients is None:
            raise RuntimeError("OpticalFeedback used before fit()")
        value = float((self._design([features]) @ self.coefficients)[0])
        return float(np.clip(value, 0.0, 1.0))

    def predict(self, image: np.ndarray) -> float:
        """Quality index in [0, 1], from pixels only."""
        return self.predict_features(analyse_image(image))

    def agreement(
        self, features: Sequence[ImageFeatures], quality: Sequence[float]
    ) -> dict:
        """Coefficient of determination and error spread against the analytic index."""
        predicted = np.array([self.predict_features(f) for f in features])
        target = np.asarray(list(quality), dtype=float)
        ss_res = float(((predicted - target) ** 2).sum())
        ss_tot = float(((target - target.mean()) ** 2).sum())
        return {
            "r_squared": 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
            "rmse": float(np.sqrt(((predicted - target) ** 2).mean())),
            "max_error": float(np.abs(predicted - target).max()),
            "n": len(target),
        }

    def to_dict(self) -> dict:
        return {
            "coefficients": None if self.coefficients is None
            else [float(v) for v in self.coefficients],
            "residual_std": self._residual_std,
        }

    @classmethod
    def from_dict(cls, payload: dict) -> "OpticalFeedback":
        obj = cls()
        if payload.get("coefficients") is not None:
            obj.coefficients = np.asarray(payload["coefficients"], dtype=float)
        obj._residual_std = float(payload.get("residual_std", float("nan")))
        return obj
