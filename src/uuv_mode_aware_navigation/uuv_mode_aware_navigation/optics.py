"""Underwater optical propagation and channel-availability model.

Reference implementation of ``method/OPTICAL_PROPAGATION_SPEC.md``.

One physics core serves all three optical configurations carried by the vehicle --
a camera with coaxial lighting, the same camera with off-axis lighting, and an
active green-laser profiler. They are not three simulations; they are three
read-outs of the same radiative-transfer model under different source/receiver
geometry.

Design rules this module exists to enforce
------------------------------------------
1. **No prescribed error terms.** Measurement noise, geometric bias, and dropout
   are *derived* from propagation geometry and water optical properties. There is
   no hand-tuned bias vector anywhere in this file. Every free parameter is a
   physical quantity with a published range or an explicitly declared design
   choice (see the spec, section 9).
2. **No privileged information leaves this module.** The water state (turbidity
   index ``t``, beam attenuation ``c``, optical depth ``tau``) is *hidden state*.
   It is consumed here and reported to the evaluator. The only optical quantity a
   navigation consumer may read is the image-derived ``quality`` score.
3. **Determinism.** Given the same seed and inputs, outputs are bit-identical.
   No global RNG is touched.

This module has no ROS dependency by design: the statistical campaign runs
headless, and ROS is only a thin wrapper for the qualitative demonstration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Sequence

import numpy as np

__all__ = [
    "WaterState",
    "ChannelConfig",
    "ChannelResponse",
    "CAMERA_COAXIAL",
    "CAMERA_OFFAXIS",
    "LIDAR",
    "CONFIGURATIONS",
    "WATER_LEVELS",
    "attenuation_from_turbidity",
    "slant_range",
    "path_length",
    "optical_depth",
    "near_field_cutoff",
    "signal_radiance",
    "backscatter_radiance",
    "backscatter_integral",
    "channel_response",
]

# ---------------------------------------------------------------------------
# Section 9.1 -- channel range limits. LITERATURE VALUES, not design choices.
#
#   coaxial camera        1-2 attenuation lengths      [R3]
#   source-separated      ~3 attenuation lengths       [R3]
#   laser line scan       5-6 attenuation lengths      [R2]
#   range-gated pulsed    up to 7 attenuation lengths  [R3]
#
# Mid-range values are used; the published ranges are reported in the manuscript.
# ---------------------------------------------------------------------------
TAU_MAX_COAXIAL = 1.5
TAU_MAX_OFFAXIS = 3.0
TAU_MAX_LIDAR = 5.5

# ---------------------------------------------------------------------------
# Section 9.2 -- water. Declared levels spanning the published band [R5].
# Paper 2 makes no claim of correspondence to a named Jerlov water type.
# ---------------------------------------------------------------------------
WATER_LEVELS: tuple[float, ...] = (0.20, 0.60, 1.20, 2.00)  # beam attenuation c, m^-1

BACKSCATTER_RATIO = 0.0183       # B = b_b/b; Petzold, conventionally used [R4]
BACKSCATTER_RATIO_FIELD = 0.013  # field geometric mean [R4] -- sensitivity alternative

# Scattering share of beam attenuation, b/c. Marine particulate-dominated water
# is strongly scattering-dominated. The absolute backscattering coefficient is
# b_b = B * b = B * SCATTERING_FRACTION * c -- note it scales WITH c, so clear
# water produces little backscatter, as it must.
SCATTERING_FRACTION = 0.8

# Green 532 nm attenuates less than the camera's band average. Applied as a
# multiplier on c for the laser channel only.
LIDAR_SPECTRAL_FACTOR = 0.85

# ---------------------------------------------------------------------------
# Section 9.3 -- geometry. DECLARED DESIGN CHOICES, labelled as such.
# ---------------------------------------------------------------------------
ALTITUDE_NOMINAL_M = 3.0
ALTITUDE_LOW_M = 1.0
THETA_CAMERA_RAD = math.radians(30.0)   # half-angle of the existing 60 deg FOV
THETA_LIGHT_RAD = math.radians(30.0)
BASELINE_COAXIAL_M = 0.02               # lamp effectively at the lens
BASELINE_OFFAXIS_M = 0.35               # within BlueROV2-class frame width
SEABED_REFLECTANCE = 0.20               # declared; low sensitivity (spec 9.3)

# Detection thresholds and scale factors. Declared design values; the
# availability boundary is dominated by tau vs tau_max, not by these.
CONTRAST_FLOOR = 0.05          # below this, no usable feature extraction
AVAILABILITY_SHARPNESS = 4.0   # logistic steepness in attenuation lengths
READ_NOISE = 0.01              # detector read noise, normalised units

# Position-fix uncertainty reference: the 1-sigma error of a feature-based fix
# at nominal survey altitude in clear water. Declared design value, plausible
# for fiducial / visual-odometry positioning at metre standoff.
#
# The fix error then scales with (a) standoff range, since angular feature error
# maps to position error through range, (b) inverse contrast, since low contrast
# degrades feature localisation, and (c) inverse sqrt of two-way transmittance,
# since an attenuated return carries proportionally fewer photons. Note that
# contrast alone is NOT sufficient: off-axis lighting keeps contrast high in
# degraded water by excluding the near field, but the signal is still heavily
# attenuated, and the fix must get noisier accordingly.
POSITION_NOISE_REF_M = 0.02
POSITION_NOISE_FLOOR_M = 0.005
BIAS_GEOMETRIC_SCALE = 0.5     # dimensionless; see bias model below

_QUADRATURE_POINTS = 256

# ``np.trapezoid`` is NumPy >= 2.0; ``np.trapz`` is the older spelling. Support
# both so the package runs on distribution NumPy as well as a modern venv.
_trapezoid = getattr(np, "trapezoid", None) or np.trapz


# ---------------------------------------------------------------------------
# Water state
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class WaterState:
    """Hidden water state. Never exposed to a navigation consumer.

    Attributes
    ----------
    c:
        Beam attenuation coefficient ``c = a + b`` in m^-1. This is the imaging
        quantity, NOT the diffuse downwelling coefficient ``K_d``. Confusing the
        two understates imaging degradation by roughly an order of magnitude
        (see spec section 3.2).
    backscatter_ratio:
        ``B = b_b / b``, the particulate backscatter fraction.
    """

    c: float
    backscatter_ratio: float = BACKSCATTER_RATIO

    @classmethod
    def from_turbidity(
        cls,
        t: float,
        levels: Sequence[float] = WATER_LEVELS,
        backscatter_ratio: float = BACKSCATTER_RATIO,
    ) -> "WaterState":
        """Build a water state from a scenario turbidity index ``t`` in [0, 1].

        ``t`` interpolates piecewise-linearly across the declared levels
        W0..W3. It is a scenario control, not a physical quantity, and it is
        hidden state exactly like ``c``.
        """
        if not levels:
            raise ValueError("levels must be non-empty")
        t = float(np.clip(t, 0.0, 1.0))
        positions = np.linspace(0.0, 1.0, len(levels))
        c = float(np.interp(t, positions, np.asarray(levels, dtype=float)))
        return cls(c=c, backscatter_ratio=backscatter_ratio)

    def for_wavelength(self, spectral_factor: float) -> "WaterState":
        """Return this water state as seen by a different spectral band."""
        return replace(self, c=self.c * float(spectral_factor))


# ---------------------------------------------------------------------------
# Channel configuration
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ChannelConfig:
    """A sensing configuration the mode manager may select.

    ``tau_max`` is a literature value (spec 9.1). ``power_w`` and ``rate_hz`` are
    what stop "always use the laser" from being a free winning strategy: the
    laser sees furthest but scans slowly and draws more power, and both costs are
    priced by the mission-cost budget.
    """

    name: str
    tau_max: float
    baseline_m: float
    theta_camera_rad: float = THETA_CAMERA_RAD
    theta_light_rad: float = THETA_LIGHT_RAD
    spectral_factor: float = 1.0
    rate_hz: float = 15.0
    power_w: float = 10.0
    active: bool = False  # True for the laser: its own collimated source


CAMERA_COAXIAL = ChannelConfig(
    name="camera_coaxial",
    tau_max=TAU_MAX_COAXIAL,
    baseline_m=BASELINE_COAXIAL_M,
    rate_hz=15.0,
    power_w=10.0,
)

CAMERA_OFFAXIS = ChannelConfig(
    name="camera_offaxis",
    tau_max=TAU_MAX_OFFAXIS,
    baseline_m=BASELINE_OFFAXIS_M,
    rate_hz=15.0,
    power_w=12.0,
)

LIDAR = ChannelConfig(
    name="lidar",
    tau_max=TAU_MAX_LIDAR,
    baseline_m=0.20,
    theta_light_rad=math.radians(1.0),  # collimated beam: tiny common volume
    spectral_factor=LIDAR_SPECTRAL_FACTOR,
    rate_hz=2.0,                        # scanning -> low effective fix rate
    power_w=45.0,
    active=True,
)

CONFIGURATIONS: tuple[ChannelConfig, ...] = (CAMERA_COAXIAL, CAMERA_OFFAXIS, LIDAR)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def slant_range(altitude_m: float, tilt_rad: float = 0.0) -> float:
    """One-way range to the seabed for a downward-looking sensor."""
    if altitude_m <= 0.0:
        raise ValueError("altitude must be positive")
    denom = math.cos(tilt_rad)
    if denom <= 1e-6:
        raise ValueError("tilt too close to horizontal")
    return altitude_m / denom


def path_length(altitude_m: float, tilt_rad: float = 0.0) -> float:
    """Two-way optical path: source -> seabed -> receiver.

    The factor of two is why altitude is such a strong lever. Optical depth is
    ``2*c*h``, so halving altitude halves tau and viability improves as
    ``exp(-2*c*dh)``.
    """
    return 2.0 * slant_range(altitude_m, tilt_rad)


def optical_depth(c: float, altitude_m: float, tilt_rad: float = 0.0) -> float:
    """Optical depth tau = c * L, in attenuation lengths."""
    return float(c) * path_length(altitude_m, tilt_rad)


def near_field_cutoff(
    baseline_m: float,
    theta_camera_rad: float = THETA_CAMERA_RAD,
    theta_light_rad: float = THETA_LIGHT_RAD,
) -> float:
    """Range at which the lamp cone first intersects the field of view.

    Everything closer than this contributes no backscatter at all -- and that
    excluded region is exactly where the ``1/r^2`` weighting was largest. This is
    the entire mechanism by which off-axis lighting works.
    """
    spread = math.tan(theta_camera_rad) + math.tan(theta_light_rad)
    if spread <= 0.0:
        return float("inf")
    return max(float(baseline_m) / spread, 0.0)


# ---------------------------------------------------------------------------
# Radiometry
# ---------------------------------------------------------------------------
def backscatter_integral(c: float, r_min: float, r_max: float) -> float:
    """Integrate ``exp(-2*c*r) / r^2`` over the common volume.

    The ``1/r^2`` weighting makes the near field dominant: most backscatter comes
    from particles close to the vehicle, brightly lit by its own lamps. Evaluated
    by log-spaced trapezoidal quadrature so the integrable region near ``r_min``
    is resolved.
    """
    r_min = max(float(r_min), 1e-4)
    r_max = float(r_max)
    if r_max <= r_min:
        return 0.0
    r = np.geomspace(r_min, r_max, _QUADRATURE_POINTS)
    integrand = np.exp(-2.0 * float(c) * r) / (r * r)
    return float(_trapezoid(integrand, r))


def _logistic(x: float) -> float:
    """Overflow-safe logistic. Extreme turbidity must return 0, not raise."""
    if x >= 0.0:
        z = math.exp(-x)
        return 1.0 / (1.0 + z)
    z = math.exp(x)
    return z / (1.0 + z)


def _backscatter_coefficient(water: WaterState) -> float:
    """Absolute backscattering coefficient b_b, in m^-1."""
    return water.backscatter_ratio * SCATTERING_FRACTION * water.c


def _compute_backscatter_normalisation() -> float:
    """Normalisation tying the contrast criterion to a published range limit.

    The radiometric model is written in relative units, so it needs one scale
    factor. Rather than choose it, it is *solved* so that a coaxial camera hits
    the contrast floor exactly at its published limit of ``TAU_MAX_COAXIAL``
    attenuation lengths [R3].

    This removes a free parameter and makes the two availability criteria --
    optical depth versus published limit, and contrast versus detection floor --
    consistent by construction instead of competing. Off-axis and laser
    configurations then reach further purely because their geometry admits less
    backscatter, which is the physical result the ladder in the spec describes.
    """
    altitude = ALTITUDE_NOMINAL_M
    c = TAU_MAX_COAXIAL / (2.0 * altitude)
    r = altitude
    signal = SEABED_REFLECTANCE * math.exp(-c * 2.0 * altitude) / (r * r)
    r_min = near_field_cutoff(BASELINE_COAXIAL_M)
    raw = (
        BACKSCATTER_RATIO
        * SCATTERING_FRACTION
        * c
        * backscatter_integral(c, r_min, r)
    )
    if raw <= 0.0:
        return 1.0
    return signal * (1.0 - CONTRAST_FLOOR) / (raw * (1.0 + CONTRAST_FLOOR))


BACKSCATTER_NORMALISATION = _compute_backscatter_normalisation()


def _compute_quality_normalisation() -> float:
    """Scale making clear water at survey altitude read as near-perfect quality."""
    altitude = ALTITUDE_NOMINAL_M
    c = WATER_LEVELS[0]
    tau = 2.0 * c * altitude
    return max(math.sqrt(math.exp(-tau)) / 0.95, 1e-9)


QUALITY_NORMALISATION = _compute_quality_normalisation()


@dataclass(frozen=True)
class ChannelResponse:
    """What one optical configuration delivers this tick.

    ``quality`` is the ONLY field a navigation consumer may read. Everything
    else is either an evaluator quantity or is applied to the measurement by the
    sensor layer.
    """

    name: str
    available: bool
    tau: float
    contrast: float
    snr: float
    sigma_m: float
    bias_m: np.ndarray
    rate_hz: float
    power_w: float
    quality: float
    p_available: float

    #: Fields a navigation consumer (mode manager, estimator, controller) may read.
    NAVIGATION_FIELDS = ("name", "available", "quality", "rate_hz", "power_w")

    def navigation_view(self) -> dict:
        """Return only what a navigation consumer is permitted to see.

        Enforces protocol rule N2 in code rather than by convention. The water
        state and its derivatives -- ``tau``, ``contrast``, ``snr``, ``sigma_m``,
        ``bias_m``, ``p_available`` -- are evaluator or sensor-layer quantities
        and must never reach a decision-making node. Anything that consumes this
        model on the navigation side goes through this method.
        """
        return {field: getattr(self, field) for field in self.NAVIGATION_FIELDS}


def signal_radiance(
    water: WaterState,
    altitude_m: float,
    config: ChannelConfig,
    reflectance: float = SEABED_REFLECTANCE,
    tilt_rad: float = 0.0,
) -> float:
    """Radiance reflected from the seabed and surviving the two-way path.

    Factored out so that the availability model and the image renderer in
    :mod:`imaging` compute it from one expression rather than two. They were
    briefly written separately, and the two disagreed by the backscatter
    normalisation constant and by the collimated-source solid-angle factor --
    which made rendered images look unusable at altitudes where the analytic
    model reported a quality of 0.91. Two implementations of one equation is one
    too many.
    """
    band = water.for_wavelength(config.spectral_factor)
    r = slant_range(altitude_m, tilt_rad)
    return (
        reflectance
        * math.exp(-band.c * path_length(altitude_m, tilt_rad))
        / (r * r)
    )


def backscatter_radiance(
    water: WaterState,
    altitude_m: float,
    config: ChannelConfig,
    tilt_rad: float = 0.0,
) -> float:
    """Veiling radiance from the volume shared by illumination and field of view.

    Collimated active sources illuminate a far smaller common volume; the
    solid-angle ratio is what suppresses their backscatter, alongside gating.
    """
    band = water.for_wavelength(config.spectral_factor)
    r = slant_range(altitude_m, tilt_rad)
    r_min = near_field_cutoff(
        config.baseline_m, config.theta_camera_rad, config.theta_light_rad
    )
    volume = backscatter_integral(band.c, r_min, r)
    solid_angle = (math.tan(config.theta_light_rad) ** 2) / (
        math.tan(THETA_LIGHT_RAD) ** 2
    )
    return (
        BACKSCATTER_NORMALISATION
        * _backscatter_coefficient(band)
        * volume
        * max(solid_angle, 1e-6)
    )


def channel_response(
    water: WaterState,
    altitude_m: float,
    config: ChannelConfig,
    *,
    tilt_rad: float = 0.0,
    reflectance: float = SEABED_REFLECTANCE,
    rng: np.random.Generator | None = None,
    bias_direction: Sequence[float] = (1.0, 0.0, 0.0),
) -> ChannelResponse:
    """Evaluate one optical configuration against the current water and altitude.

    Returns availability, measurement covariance, and a *derived* geometric bias.
    """
    band = water.for_wavelength(config.spectral_factor)
    c = band.c
    r = slant_range(altitude_m, tilt_rad)
    tau = optical_depth(c, altitude_m, tilt_rad)

    e_signal = signal_radiance(water, altitude_m, config, reflectance, tilt_rad)
    e_backscatter = backscatter_radiance(water, altitude_m, config, tilt_rad)

    total = e_signal + e_backscatter
    contrast = (e_signal - e_backscatter) / total if total > 0.0 else 0.0
    contrast = float(np.clip(contrast, -1.0, 1.0))
    snr = e_signal / math.sqrt(max(total + READ_NOISE * READ_NOISE, 1e-12))

    # --- availability: soft boundary in attenuation lengths, gated by contrast ---
    margin = config.tau_max - tau
    p_available = _logistic(AVAILABILITY_SHARPNESS * margin)
    if contrast < CONTRAST_FLOOR:
        p_available = 0.0
    generator = rng if rng is not None else np.random.default_rng(0)
    available = bool(generator.random() < p_available)

    transmittance = math.exp(-tau)

    # --- measurement noise: derived from range, contrast, and transmittance ---
    sigma = (
        POSITION_NOISE_REF_M
        * (r / ALTITUDE_NOMINAL_M)
        / (
            max(contrast, CONTRAST_FLOOR)
            * math.sqrt(max(transmittance, 1e-9))
        )
    )
    sigma = float(max(sigma, POSITION_NOISE_FLOOR_M))

    # --- geometric bias, DERIVED (spec 6.2) --------------------------------
    # An off-axis source lights the scene asymmetrically, so the residual veil
    # has a centroid displaced from the optical axis, shifting the apparent
    # centroid of tracked features.
    #
    # NOTE ON MONOTONICITY: bias is NOT monotone in the baseline. It vanishes at
    # d = 0 (a symmetric veil cannot displace a centroid) and again as d grows
    # large (the veil itself is suppressed). It peaks at intermediate baseline.
    # Off-axis lighting therefore trades a large symmetric contrast gain for a
    # smaller geometric bias -- a real trade, not a free win.
    veil_fraction = e_backscatter / total if total > 0.0 else 0.0
    footprint = r * math.tan(config.theta_camera_rad)
    asymmetry = float(np.clip(config.baseline_m / max(footprint, 1e-6), 0.0, 1.0))
    bias_magnitude = BIAS_GEOMETRIC_SCALE * veil_fraction * asymmetry * footprint

    direction = np.asarray(bias_direction, dtype=float)
    norm = np.linalg.norm(direction)
    direction = direction / norm if norm > 0.0 else np.zeros(3)
    bias = bias_magnitude * direction

    # --- the only quantity a navigation consumer may read ------------------
    # Quality is an image statistic, NOT a property of the pose fix. It is
    # measurable whenever the camera is running, including on ticks where no fix
    # was obtained -- which is exactly when the manager needs it most, since that
    # is when it must predict whether a *different* configuration would work.
    # Zeroing it on failed ticks would make it a proxy for availability and
    # destroy its value as a counterfactual predictor.
    #
    # It tracks FEATURE DETECTABILITY, which needs both contrast and signal
    # strength. Contrast alone is not enough and is actively misleading here:
    # off-axis lighting holds contrast near 0.97 even in degraded water, because
    # excluding the near field removes the veil regardless of how little light
    # returns. A contrast-only quality therefore barely moves across the entire
    # operating range, leaving the manager blind to the water state. What truly
    # collapses is the returning signal, so transmittance carries the
    # information, and detectable features scale with its square root.
    detectability = max(contrast, 0.0) * math.sqrt(max(transmittance, 0.0))
    quality = float(np.clip(detectability / QUALITY_NORMALISATION, 0.0, 1.0))

    return ChannelResponse(
        name=config.name,
        available=available,
        tau=float(tau),
        contrast=contrast,
        snr=float(snr),
        sigma_m=sigma,
        bias_m=bias,
        rate_hz=config.rate_hz,
        power_w=config.power_w,
        quality=quality,
        p_available=float(p_available),
    )


def attenuation_from_turbidity(
    t: float, levels: Sequence[float] = WATER_LEVELS
) -> float:
    """Convenience wrapper: scenario turbidity index -> beam attenuation c."""
    return WaterState.from_turbidity(t, levels).c
