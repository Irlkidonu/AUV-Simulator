"""Acoustic positioning techniques and the noise that degrades them.

The study previously modelled one acoustic capability: a single seabed beacon
returning a range, either available or switched off by a fault schedule. That is
thin for a paper claiming multi-modal sensing, and it left one of the three
environments the vehicle is supposed to cope with -- an acoustically noisy one --
entirely unrepresented.

This module adds the two techniques an operational vehicle would actually choose
between, and the physics that decides when each is usable.

What distinguishes the techniques
---------------------------------
They differ in what they measure, not merely in how well:

* **Single-beacon ranging** returns one range. That constrains position to a
  sphere, so it fixes nothing on its own; paired with the pressure depth it
  constrains position to a circle, and it takes vehicle motion or a second
  beacon to resolve where on that circle the vehicle is.
* **LBL** interrogates three or more surveyed seabed transponders and trilaterates
  a full position. It is the most accurate option available here, but it works
  only inside the array, its accuracy collapses as the geometry degenerates near
  and outside the array edge, and it costs several round trips per fix.
* **USBL** measures range *and* bearing from a single transceiver on a surface
  vessel. One interrogation gives a position, but bearing error converts to
  cross-range error in proportion to slant range, so its accuracy degrades with
  distance in a way the seabed techniques do not.

That is a genuine decision, and the conditions that decide it -- array geometry,
slant range, ambient noise -- are all things the vehicle can observe.

Noise
-----
Ambient noise is modelled as a spectral level in dB re 1 uPa^2/Hz, the standard
unit, and enters through the sonar equation. Signal excess is

    SNR = SL - TL - (NL - 10 log10 B)

with transmission loss ``TL = 20 log10 r + alpha r``: spherical spreading plus
absorption. Time-of-arrival precision improves as the square root of signal
power, which is why range error here scales as ``10^(-SNR/20)`` rather than
being a constant. Below a detection threshold the interrogation simply fails,
which is what makes a noisy environment a *capability* question rather than a
precision one.

Absorption is taken at a single representative value rather than from a full
Francois--Garrison model. The study operates at one frequency and over ranges of
tens of metres, where absorption contributes about a decibel in total; carrying
the full frequency dependence would add parameters without changing any decision.
This is stated as a simplification rather than left implicit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

__all__ = [
    "AcousticTechnique",
    "SINGLE_BEACON",
    "LBL",
    "USBL",
    "ACOUSTIC_TECHNIQUES",
    "TERRAIN_RELATIVE",
    "terrain_relative_response",
    "NoiseState",
    "AcousticResponse",
    "acoustic_response",
    "transmission_loss_db",
    "signal_to_noise_db",
    "dilution_of_precision",
]

#: Speed of sound (m/s). A single value: the study spans a few tens of metres of
#: depth, over which a real profile varies by well under one percent, and no
#: decision in the manager depends on ray bending.
SOUND_SPEED_MPS: float = 1500.0

#: Absorption coefficient (dB/m) at the modelled frequency. Representative of a
#: mid-frequency transponder band in shallow seawater.
ABSORPTION_DB_PER_M: float = 0.03

#: Source level (dB re 1 uPa at 1 m) of the interrogated transponder.
SOURCE_LEVEL_DB: float = 185.0

#: Receiver bandwidth (Hz). Enters the sonar equation through the noise power in
#: the band, and sets the time-resolution floor below.
BANDWIDTH_HZ: float = 4000.0

#: Signal-to-noise ratio below which an interrogation does not produce a usable
#: arrival at all.
DETECTION_THRESHOLD_DB: float = 10.0

#: Ambient spectral noise levels (dB re 1 uPa^2/Hz). The quiet and loud values
#: bracket the sea-state and shipping range that Wenz's curves describe for
#: coastal water at mid frequency; the vehicle is never told which it is in.
NOISE_QUIET_DB: float = 40.0
NOISE_MODERATE_DB: float = 55.0
NOISE_LOUD_DB: float = 70.0


@dataclass(frozen=True)
class NoiseState:
    """Ambient acoustic conditions. Evaluator-side truth, like ``WaterState``."""

    spectral_level_db: float = NOISE_QUIET_DB

    @property
    def band_level_db(self) -> float:
        """Noise power in the receiver band."""
        return self.spectral_level_db + 10.0 * math.log10(BANDWIDTH_HZ)


@dataclass(frozen=True)
class AcousticTechnique:
    """One acoustic positioning technique the manager may select."""

    name: str
    #: Number of transponder interrogations needed per fix. Each costs a
    #: two-way travel time, so this sets the update rate.
    interrogations_per_fix: int
    #: Whether a fix constrains all three position axes (LBL, USBL) or only
    #: range to a single point (single beacon).
    gives_position: bool
    #: One-sigma bearing error (rad), zero for techniques that measure no
    #: bearing. For USBL this is what converts slant range into cross-range
    #: error and makes accuracy degrade with distance.
    bearing_sigma_rad: float = 0.0
    #: Time resolution floor (s) of the arrival estimator, independent of SNR.
    #: Sets the best achievable range precision through ``c * sigma_t``.
    timing_floor_s: float = 2.0e-5
    #: Transponder positions. Empty means "use the scenario's single beacon".
    transponders: tuple[tuple[float, float, float], ...] = ()
    #: Whether the technique needs a surface vessel overhead.
    requires_surface_asset: bool = False
    #: What infrastructure the technique depends on. Techniques sharing a
    #: dependency fail together, and that is what makes a switch between them
    #: worthless and a switch across them valuable.
    #:
    #:   "surface"      a transceiver held at the surface: USBL. Standard in
    #:                  offshore oil and gas and in ship-tended survey, absent
    #:                  the moment the vessel leaves station.
    #:   "seabed_array" transponders placed and surveyed on the bottom: LBL and
    #:                  single-beacon ranging.
    #:   "none"         terrain matching, which listens to nothing and needs
    #:                  only its own echo sounder and a prior map.
    infrastructure: str = "seabed_array"
    #: Terrain-relative navigation: a fix comes from matching measured
    #: bathymetry against a prior map rather than from interrogating a
    #: transponder. Accuracy is set by terrain relief, not by range or geometry,
    #: and the technique has an entirely different failure mode from the others
    #: -- it needs a textured seabed rather than deployed infrastructure.
    terrain_relative: bool = False
    #: One-sigma depth measurement error of the echo sounder, in metres. With
    #: the terrain gradient this sets the position accuracy of a match.
    depth_sigma_m: float = 0.05
    #: Below this terrain gradient (m/m) a match is not identifiable: a flat
    #: seabed looks the same everywhere, so no correlation peak exists. This is
    #: TRN's analogue of the turbidity limit on an optical channel.
    minimum_gradient: float = 0.02
    #: Maximum altitude at which the echo sounder resolves terrain well enough
    #: to match, in metres.
    maximum_altitude_m: float = 60.0

    @property
    def fix_period_s(self) -> float:
        """Minimum time between fixes, set by the round trips a fix requires."""
        return 2.0 * float(self.interrogations_per_fix)


#: A single seabed transponder returning range only. Always the fallback: it
#: needs no deployed array and no surface asset.
SINGLE_BEACON = AcousticTechnique(
    name="single_beacon",
    interrogations_per_fix=1,
    gives_position=False,
)

#: Terrain-relative navigation. A multibeam or profiling sonar measures the
#: bathymetry under the vehicle and it is correlated against a prior map to fix
#: horizontal position.
#:
#: It is grouped with the acoustic techniques because it answers the same
#: question -- how does this vehicle obtain an absolute fix without optics --
#: and because it is an acoustic measurement. It differs from the other three in
#: what it requires: no deployed transponders and no surface asset, but a prior
#: map and a seabed with relief.
#:
#: Two interrogations per fix: a match needs a short along-track profile rather
#: than a single ping, because one depth sample is consistent with every point
#: on an isobath.
#:
#: This technique was absent from the first version of this study, and its
#: absence made the compound family artificially unrecoverable. A vehicle that
#: has lost both optical aiding and its DVL still has an echo sounder, and
#: terrain matching is the standard response in that situation. Adding it is a
#: correction to the sensor suite, not a new capability invented for the method.
TERRAIN_RELATIVE = AcousticTechnique(
    name="terrain_relative",
    interrogations_per_fix=2,
    gives_position=True,
    terrain_relative=True,
    infrastructure="none",
)

#: Four seabed transponders around the survey area. Positions are the corners of
#: a square enclosing the lawnmower pattern, which is how an LBL array is laid
#: for a survey box.
LBL = AcousticTechnique(
    name="lbl",
    interrogations_per_fix=3,
    gives_position=True,
    transponders=(
        (-25.0, -25.0, -20.0),
        (25.0, -25.0, -20.0),
        (25.0, 25.0, -20.0),
        (-25.0, 25.0, -20.0),
    ),
)

#: A hull-mounted transceiver on a vessel holding station above the survey.
#: One interrogation gives range and bearing, so it is the fastest technique
#: here, at the cost of a bearing error that grows into cross-range error.
USBL = AcousticTechnique(
    name="usbl",
    interrogations_per_fix=1,
    gives_position=True,
    bearing_sigma_rad=math.radians(0.2),
    transponders=((0.0, 0.0, 0.0),),
    requires_surface_asset=True,
    infrastructure="surface",
)

#: The techniques a vehicle in this study may select between.
#:
#: USBL is included, because where a surface asset exists it is the most capable
#: technique here -- that is the offshore oil and gas case, and ship-tended
#: survey. Its availability is a property of the *deployment*, not of the
#: technique, so it is removed by a scenario fault rather than by omission: when
#: the vessel leaves station the technique goes with it, and the vehicle must
#: find its absolute fix somewhere else. That is what E18 exercises.
ACOUSTIC_TECHNIQUES: tuple[AcousticTechnique, ...] = (
    SINGLE_BEACON, LBL, USBL, TERRAIN_RELATIVE,
)


def transmission_loss_db(range_m: float) -> float:
    """Spherical spreading plus absorption."""
    r = max(float(range_m), 1.0)
    return 20.0 * math.log10(r) + ABSORPTION_DB_PER_M * r


def signal_to_noise_db(range_m: float, noise: NoiseState) -> float:
    """Sonar equation: source level, less path loss, less in-band noise."""
    return SOURCE_LEVEL_DB - transmission_loss_db(range_m) - noise.band_level_db


def range_sigma_m(range_m: float, technique: AcousticTechnique,
                  noise: NoiseState) -> float:
    """One-sigma range error, floored by timing resolution.

    Arrival-time precision improves as the square root of signal power, so the
    error falls by a factor of ten for every 20 dB of signal excess. The floor
    is the estimator's own time resolution, which no amount of signal removes.
    """
    snr = signal_to_noise_db(range_m, noise)
    excess = max(snr - DETECTION_THRESHOLD_DB, 0.0)
    sigma_t = technique.timing_floor_s * (10.0 ** (-excess / 20.0) + 1.0)
    return SOUND_SPEED_MPS * sigma_t


def dilution_of_precision(
    position: Sequence[float], transponders: Sequence[Sequence[float]]
) -> float:
    """Geometric dilution of precision for trilateration from these ranges.

    Position error is range error multiplied by this factor. It is near one when
    the transponders surround the vehicle and diverges as they crowd into a
    similar direction, which is what happens at and beyond the edge of an LBL
    array -- and is why LBL is not simply "the accurate one".

    Computed in the horizontal plane only. Depth is measured directly by the
    pressure sensor to a centimetre, so a positioning system is never asked to
    supply it, and including the vertical axis here would report a geometry
    problem that no fix depends on. With seabed transponders and a vehicle a few
    metres above them the vertical geometry is nearly degenerate by
    construction: including it returned a dilution of 6.0 -- at the usability
    limit -- even at the exact centre of the array, which would have made LBL
    unusable everywhere and removed the technique from the study.
    """
    p = np.asarray(position, dtype=float)[:2]
    rows = []
    for t in transponders:
        delta = p - np.asarray(t, dtype=float)[:2]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return float("inf")
        rows.append(delta / distance)
    geometry = np.asarray(rows)
    try:
        covariance = np.linalg.inv(geometry.T @ geometry)
    except np.linalg.LinAlgError:
        return float("inf")
    trace = float(np.trace(covariance))
    return math.sqrt(trace) if trace > 0.0 else float("inf")


@dataclass(frozen=True)
class AcousticResponse:
    """What a technique can deliver under these conditions."""

    available: bool
    #: One-sigma position error (m) if the technique gives a position, else the
    #: one-sigma range error.
    sigma_m: float
    snr_db: float
    dop: float
    reason: str


#: Beyond this the LBL geometry is too degenerate to trust.
MAXIMUM_LBL_DOP: float = 6.0

#: Maximum slant range at which the surface transceiver is usable.
MAXIMUM_USBL_RANGE_M: float = 120.0


def terrain_relative_response(
    technique: AcousticTechnique,
    altitude_m: float,
    terrain_gradient: float,
) -> AcousticResponse:
    """Position accuracy of a bathymetric match, from the terrain it is matching.

    A terrain match localises the vehicle by finding where the measured depth
    profile fits the prior map. Its accuracy follows the slope of the terrain:
    an error of one sigma in measured depth maps to a horizontal error of

        sigma_position = sigma_depth / |grad h|

    which is the Cramer-Rao form of the problem and is the reason the technique
    is useless over a flat seabed. On a plain the gradient is zero, every
    position predicts the same depth, and no amount of averaging identifies
    which one the vehicle is at. On a slope or a ridge field the same instrument
    fixes position to a fraction of a metre.

    This is structurally the same statement as the optical channel's dependence
    on turbidity, and it is what makes terrain relative navigation a genuine
    *alternative* rather than a free upgrade: it fails where the others work and
    works where they fail. Without a failure envelope it would simply dominate
    the action space and the study would measure nothing, which is the defect
    recorded against the acoustic beacon in COMPARATOR_SPEC section 4.1.

    Ambient noise does not enter. The echo sounder is looking straight down at
    a few metres range with an enormous signal excess; what limits the fix is
    the information in the terrain, not the signal-to-noise ratio.
    """
    gradient = float(terrain_gradient)
    if not math.isfinite(gradient) or gradient < technique.minimum_gradient:
        return AcousticResponse(
            False, float("inf"), math.inf, float("inf"), "terrain_featureless"
        )
    if altitude_m > technique.maximum_altitude_m:
        return AcousticResponse(
            False, float("inf"), math.inf, float("inf"), "above_sounder_range"
        )
    sigma = technique.depth_sigma_m / gradient
    return AcousticResponse(True, sigma, math.inf, 1.0, "terrain_match")


def acoustic_response(
    technique: AcousticTechnique,
    position: Sequence[float],
    noise: NoiseState,
    beacon: Optional[Sequence[float]] = None,
    terrain_gradient: Optional[float] = None,
    altitude_m: Optional[float] = None,
) -> AcousticResponse:
    """Evaluate one acoustic technique at this position and noise level.

    Returns availability and accuracy together, because for these techniques
    they are the same question asked at two thresholds: an interrogation that
    cannot be detected gives no fix, and one that is barely detected gives a bad
    one.
    """
    p = np.asarray(position, dtype=float)

    if technique.terrain_relative:
        if terrain_gradient is None:
            return AcousticResponse(False, float("inf"), math.inf,
                                    float("inf"), "no_terrain_information")
        # Altitude above the seabed, from the vehicle depth if not supplied.
        alt = altitude_m if altitude_m is not None else abs(float(p[2]))
        return terrain_relative_response(technique, alt, terrain_gradient)

    if technique.transponders:
        points = [np.asarray(t, dtype=float) for t in technique.transponders]
    elif beacon is not None:
        points = [np.asarray(beacon, dtype=float)]
    else:
        return AcousticResponse(False, float("inf"), -math.inf, float("inf"),
                                "no_transponder")

    ranges = [float(np.linalg.norm(p - t)) for t in points]
    worst_range = max(ranges)
    snr = signal_to_noise_db(worst_range, noise)
    if snr < DETECTION_THRESHOLD_DB:
        return AcousticResponse(False, float("inf"), snr, float("inf"),
                                "below_detection_threshold")

    sigma_r = range_sigma_m(worst_range, technique, noise)

    if technique is USBL or technique.bearing_sigma_rad > 0.0:
        slant = ranges[0]
        if slant > MAXIMUM_USBL_RANGE_M:
            return AcousticResponse(False, float("inf"), snr, 1.0,
                                    "beyond_usbl_range")
        # Bearing error becomes cross-range error in proportion to range. This
        # is the defining limitation of USBL and the reason a deep vehicle is
        # positioned less accurately than a shallow one by the same equipment.
        cross_range = slant * technique.bearing_sigma_rad
        sigma = math.sqrt(sigma_r * sigma_r + 2.0 * cross_range * cross_range)
        return AcousticResponse(True, sigma, snr, 1.0, "usbl_fix")

    if technique.gives_position:
        dop = dilution_of_precision(p, points)
        if not math.isfinite(dop) or dop > MAXIMUM_LBL_DOP:
            return AcousticResponse(False, float("inf"), snr, dop,
                                    "geometry_too_weak")
        return AcousticResponse(True, sigma_r * dop, snr, dop, "lbl_fix")

    return AcousticResponse(True, sigma_r, snr, 1.0, "range_only")


# ---------------------------------------------------------------------------
# Multipath: what actually degrades acoustics in a shallow survey box
#
# The sonar equation above says ambient noise barely matters here, and that is
# correct rather than a modelling failure. Over the tens of metres this study
# spans, signal excess runs from roughly 82 dB in quiet water to 52 dB under
# heavy shipping, all far above any detection threshold, so range *precision*
# sits at the estimator's timing floor whatever the sea state.
#
# What does degrade a shallow acoustic fix is multipath. The direct arrival
# competes with paths reflected from the surface and the seabed, and when the
# receiver locks onto a reflection instead it reports a range that is too long
# -- never too short, because no path is shorter than the direct one. The result
# is a heavy-tailed, one-sided error: mostly excellent ranges with occasional
# large positive outliers.
#
# That distinction matters for the study. A precision loss would call for
# inflating a covariance; an outlier calls for deciding whether to admit the
# measurement at all. It is the case in which gating and covariance weighting
# genuinely differ, and it is why noise is modelled here as an outlier rate
# rather than as a variance.
# ---------------------------------------------------------------------------

#: Outlier probability at the quiet and loud reference noise levels. Rising
#: ambient noise raises the reflected-path floor relative to the detector's
#: threshold, so a reflection is more often mistaken for the direct arrival.
OUTLIER_RATE_QUIET: float = 0.01
OUTLIER_RATE_LOUD: float = 0.25


def multipath_outlier_rate(noise: NoiseState) -> float:
    """Fraction of interrogations that lock onto a reflection.

    Linear in spectral level between the quiet and loud references, clamped
    outside them. A linear interpolation is used rather than a fitted curve
    because the study has no basis for a particular shape, and the claim being
    made is only that noisier water produces more false locks.
    """
    span = NOISE_LOUD_DB - NOISE_QUIET_DB
    fraction = (noise.spectral_level_db - NOISE_QUIET_DB) / span
    fraction = min(max(fraction, 0.0), 1.0)
    return OUTLIER_RATE_QUIET + fraction * (OUTLIER_RATE_LOUD - OUTLIER_RATE_QUIET)


#: Seabed depth (m, negative down) used for the bottom-reflected path.
SEABED_DEPTH_M: float = -20.0


def multipath_excess_m(
    position: Sequence[float],
    transponder: Sequence[float],
    seabed_depth_m: float = SEABED_DEPTH_M,
) -> float:
    """Excess path length of the most confusable reflection.

    Both boundaries are considered and the *shorter* excess is returned, because
    the reflection most easily mistaken for the direct arrival is the one closest
    to it in time.

    Taking only the surface path was wrong for a surface-mounted transceiver. A
    transponder at the surface is its own mirror image, so its surface-bounce
    excess is identically zero, and USBL showed no multipath outliers at any
    noise level while the seabed techniques showed 15--20 m ones. That is not a
    property of USBL; it is the seabed that reflects for a surface asset, and the
    seabed that was missing from the model.
    """
    p = np.asarray(position, dtype=float)
    t = np.asarray(transponder, dtype=float)
    horizontal = float(np.linalg.norm(p[:2] - t[:2]))
    direct = math.hypot(horizontal, float(p[2]) - float(t[2]))

    # Depths below the surface, positive.
    z1, z2 = abs(float(p[2])), abs(float(t[2]))
    depth = abs(float(seabed_depth_m))

    # Image sources: reflected in the surface, and reflected in the seabed.
    surface = math.hypot(horizontal, z1 + z2)
    bottom = math.hypot(horizontal, (2.0 * depth - z1) - z2)

    # A transponder lying exactly on a boundary is its own image in it, so that
    # boundary produces no separate arrival and its excess is identically zero.
    # Zero must therefore be read as "no distinct reflection" and excluded, not
    # taken as the closest one: including it silently removed every multipath
    # outlier from the seabed techniques, whose beacons sit on the bottom.
    excesses = [e for e in (surface - direct, bottom - direct) if e > 1e-6]
    return min(excesses) if excesses else 0.0


def surface_bounce_excess_m(
    position: Sequence[float], transponder: Sequence[float]
) -> float:
    """Extra path length of a surface-reflected arrival, by image source.

    Reflecting a source of depth ``z1`` in a flat surface places its image at
    ``-z1``, so the reflected path from an image source to a receiver at depth
    ``z2`` separated horizontally by ``x`` has length ``hypot(x, z1 + z2)``
    against the direct ``hypot(x, z1 - z2)``. The difference is exact for a flat
    surface and needs no free parameter, which is why it is computed rather than
    assumed: an invented outlier magnitude would be a tuning knob on how badly
    multipath hurts.
    """
    p = np.asarray(position, dtype=float)
    t = np.asarray(transponder, dtype=float)
    horizontal = float(np.linalg.norm(p[:2] - t[:2]))
    z1, z2 = abs(float(p[2])), abs(float(t[2]))
    direct = math.hypot(horizontal, z1 - z2)
    reflected = math.hypot(horizontal, z1 + z2)
    return max(reflected - direct, 0.0)
