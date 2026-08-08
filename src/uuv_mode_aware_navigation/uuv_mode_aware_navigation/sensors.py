"""Sensor suite, fault injection, and the evaluator/navigation information boundary.

Five modalities, satisfying the requirement of IMU, DVL, depth, optical, and at
least one non-optical aiding modality:

===  ===================================  =========================
S1   inertial (INS)                       always available
S2   DVL, body-frame velocity             bottom-lock loss, dropout
S3   pressure depth                       always available
S4   optical (see ``optics.py``)          graded degradation, loss
S5   acoustic beacon range (USBL class)   0.5 Hz, geometry-dependent
===  ===================================  =========================

Rates are part of the experimental design, not an implementation detail. The
acoustic range in particular is interrogated once per 2 s cycle; returning one
per 0.1 s simulation tick, as an earlier version did, turns range-only aiding
into a standalone positioning system that bounds error in every scenario and
removes the optical channel from the study entirely. See ``BeaconGeometry``.

Fault discipline
----------------
Faults are **discrete, scheduled events with unambiguous severity** -- available
or unavailable, or a fixed degradation profile. Severity is not a continuously
tunable knob. This is deliberate: the earlier study in this workspace rested its
headline improvement on a hand-chosen backscatter-bias magnitude, and a reviewer
could reasonably ask whether that magnitude had been selected to flatter the
method. A dropout is a dropout, and nobody can argue about its size.

The schedule is generated from the scenario seed, written to the evaluator truth
record, and **never** exposed to the estimator, manager, or controller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Sequence

import numpy as np

from .estimator import GRAVITY
from .acoustics import (
    multipath_excess_m,
    SINGLE_BEACON,
    AcousticTechnique,
    NoiseState,
    acoustic_response,
    multipath_outlier_rate,
    surface_bounce_excess_m,
)
from .imaging import OpticalFeedback, render_patch, seabed_texture
from .optics import (
    CAMERA_OFFAXIS,
    ChannelConfig,
    ChannelResponse,
    WaterState,
    channel_response,
)

__all__ = [
    "FaultKind",
    "FaultWindow",
    "FaultSchedule",
    "SensorNoise",
    "BeaconGeometry",
    "SensorSuite",
    "SensorReading",
    "dvl_loss_schedule",
    "short_dvl_loss_schedule",
    "total_dvl_loss_schedule",
    "optical_loss_schedule",
    "acoustic_duty_cycle_schedule",
    "coupled_turbidity_dvl_schedule",
    "compound_schedule",
]


class FaultKind(Enum):
    """Injectable faults. Each is binary in effect, not a tunable severity."""

    DVL_BOTTOM_LOCK_LOSS = "dvl_bottom_lock_loss"
    ACOUSTIC_OUTAGE = "acoustic_outage"
    OPTICAL_BLACKOUT = "optical_blackout"
    #: Water-track loss. A separate fault from bottom-lock loss because the two
    #: have different physical causes: bottom track fails on altitude or on an
    #: acoustically soft seabed, water track fails when the insonified layer
    #: holds too few scatterers to return a usable Doppler spectrum. A vehicle
    #: can lose either without losing the other.
    DVL_WATER_TRACK_LOSS = "dvl_water_track_loss"
    #: The surface asset leaves station. Ultra-short-baseline positioning needs
    #: a transceiver held at the surface, so it stops the moment the vessel
    #: departs; nothing on the seabed and nothing aboard the vehicle is
    #: affected. This is a *deployment* fault rather than an instrument fault,
    #: and it is the one that separates the offshore case -- where a support
    #: vessel is on station and USBL is the best technique available -- from the
    #: resident case, where the vehicle must obtain its absolute fix without
    #: anything overhead.
    SURFACE_ASSET_LOSS = "surface_asset_loss"


@dataclass(frozen=True)
class FaultWindow:
    """A scheduled fault, active over ``[start_s, start_s + duration_s)``."""

    kind: FaultKind
    start_s: float
    duration_s: float

    def active(self, t: float) -> bool:
        return self.start_s <= t < (self.start_s + self.duration_s)


@dataclass(frozen=True)
class FaultSchedule:
    """Evaluator-side truth about what was injected and when.

    This object must never be reachable from a decision-making component. It is
    constructed by the scenario, consumed by the sensor layer, and reported to
    the evaluator for scoring detection latency and missed detections.
    """

    windows: tuple[FaultWindow, ...] = ()

    def active(self, kind: FaultKind, t: float) -> bool:
        return any(w.kind is kind and w.active(t) for w in self.windows)

    def any_active(self, t: float) -> bool:
        return any(w.active(t) for w in self.windows)

    def onsets(self, kind: FaultKind) -> tuple[float, ...]:
        return tuple(sorted(w.start_s for w in self.windows if w.kind is kind))


@dataclass(frozen=True)
class SensorNoise:
    """Noise levels. Identical for every method under comparison.

    Fairness rule R2: these are functions of the scenario and seed only, never of
    which navigation method is running.
    """

    accel_mps2: float = 0.021
    accel_bias_mps2: float = 0.004
    dvl_mps: float = 0.0025
    depth_m: float = 0.01
    acoustic_range_m: float = 0.35
    #: Water-track velocity noise, an order of magnitude above bottom track.
    #:
    #: Bottom track resolves a rigid, stationary reflector. Water track resolves
    #: a cloud of suspended scatterers that are themselves moving relative to the
    #: mean flow, so turbulence and shear within the insonified layer enter the
    #: measurement as noise that no amount of averaging removes. Instrument
    #: datasheets quote water-track uncertainties roughly an order of magnitude
    #: above bottom-track for the same configuration, and the ratio here is set
    #: to 10 on that basis rather than tuned.
    #:
    #: Standard deviation of the DVL scale-factor error, drawn once per scenario.
    #:
    #: This term and the misalignment below are what make dead reckoning actually
    #: drift, and their absence was a defect that invalidated the study's premise.
    #: With white velocity noise alone, integrated position error grows as the
    #: square root of time and stays at a few centimetres however far the vehicle
    #: travels: a measured 0.09 m after 1,034 m of survey, which no real vehicle
    #: achieves and which leaves absolute aiding nothing to contribute. Published
    #: DVL-aided dead-reckoning performance is quoted as a percentage of distance
    #: travelled, typically a few tenths of one percent, precisely because the
    #: dominant errors are systematic rather than white.
    #:
    #: 0.3% is within the range instrument datasheets quote for an uncalibrated
    #: installation, and matches the scale-error factor levels the companion
    #: paper's DVL-stress scenarios use.
    dvl_scale_error: float = 0.003
    #: Standard deviation of the residual DVL mounting rotation, radians, drawn
    #: once per scenario.
    #:
    #: A DVL reports velocity in its own frame. Any residual rotation between
    #: that frame and the vehicle frame turns along-track speed into apparent
    #: cross-track speed, so the position error it produces grows *linearly* with
    #: distance travelled. For dead reckoning over a survey this is normally the
    #: dominant error source, which is why the companion paper estimates it.
    #:
    #: Paper 2 does not estimate it. The declared nine-state simplification --
    #: now twelve with the current -- carries no attitude, so alignment and scale
    #: are unmodelled systematic errors here rather than filter states. That is
    #: the correct boundary between the two papers, and it is also what gives
    #: this study its premise: an unaided vehicle drifts, so absolute fixes are
    #: worth managing. One degree is a realistic residual after a routine
    #: installation without a dedicated alignment calibration.
    dvl_misalignment_rad: float = 0.0175
    #: Not modelled: the dependence of water-track return strength on scatterer
    #: concentration. Acoustic backscatter comes from the same suspended load
    #: that scatters light, so a genuine model would couple this noise to the
    #: water state -- and would make turbid water *help* the velocity channel
    #: while it harms the optical one. That coupling is real and is stated as a
    #: limitation, not implemented, because acoustic and optical scattering
    #: weight particle size differently and the study has no basis for choosing
    #: the transfer function.
    dvl_water_track_mps: float = 0.025


@dataclass(frozen=True)
class BeaconGeometry:
    """Acoustic beacon position and the geometry that governs a usable fix.

    Availability is range-and-geometry dependent, which is what makes "hold
    station and wait for a fix" or "divert toward the beacon" real navigational
    decisions rather than free options.
    """

    position: tuple[float, float, float] = (-14.0, -14.0, -20.0)
    maximum_range_m: float = 60.0
    #: Minimum elevation above the horizontal before grazing-incidence multipath
    #: makes the range unusable.
    #:
    #: This is deliberately small. An earlier 10-degree gate -- borrowed from
    #: what would be appropriate for full 3D acoustic positioning -- blocked
    #: every fix on this mission and silently removed the non-optical aiding
    #: modality from the study. The reasoning was wrong for this sensor: a
    #: range-only measurement paired with an always-available pressure depth
    #: constrains horizontal position to a circle, which is precisely what bounds
    #: drift. Elevation governs vertical observability, and vertical is the depth
    #: sensor's job.
    minimum_elevation_rad: float = math.radians(2.0)

    #: Interrogation period. A single-beacon range is a two-way travel-time
    #: measurement, not a continuous signal, and the vehicle must wait out the
    #: acoustic round trip plus the transponder turnaround before the next ping.
    #:
    #: Getting this wrong invalidates the entire study, silently. An earlier
    #: implementation returned a range on **every** 0.1 s simulation tick. Range-
    #: only aiding at 10 Hz with 0.35 m noise, from a beacon always inside its
    #: maximum range, is a near-perfect positioning system on its own: it bounds
    #: position error by itself, in every scenario, whatever the optical channel
    #: is doing. Every comparator then scores the same, the mode manager has
    #: nothing to contribute, and the study's apparent conclusion -- "optical
    #: channel management does not matter" -- would have been an artefact of
    #: assuming permanent acoustic coverage rather than a finding about optics.
    #:
    #: The module docstring has always described this sensor as *intermittent*.
    #: The implementation simply did not match, and no test checked.
    interrogation_period_s: float = 2.0


@dataclass
class SensorReading:
    """One tick of sensor output, split by who is allowed to see what."""

    t: float
    # --- navigation-side measurements ---
    accel_mps2: np.ndarray
    dvl_velocity_mps: Optional[np.ndarray]
    depth_m: Optional[float]
    optical_position_m: Optional[np.ndarray]
    optical_sigma_m: float
    optical_quality: float
    acoustic_range_m: Optional[float]
    dvl_bottom_lock: bool
    #: Velocity relative to the water column, from the DVL's water-track mode.
    #: Differs from ``dvl_velocity_mps`` by the current, which is what makes the
    #: current observable at all.
    dvl_water_velocity_mps: Optional[np.ndarray] = None
    #: Full position fix from a position-fixing acoustic technique (LBL, USBL).
    #: Range-only techniques leave this empty and fill ``acoustic_range_m``.
    acoustic_position_m: Optional[np.ndarray] = None
    #: One-sigma accuracy of whichever acoustic measurement was produced.
    acoustic_sigma_m: float = 0.0
    #: Which technique produced it. Diagnostic, and shown by the demonstrator.
    acoustic_technique: str = "single_beacon"
    # --- evaluator-side truth, never passed to a decision component ---
    true_position_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    optical_bias_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    channel: str = ""


class SensorSuite:
    """Generates measurements from true motion, water state, and a fault schedule.

    Deterministic: all randomness comes from the supplied generator, seeded by the
    scenario. The global NumPy RNG is never touched.
    """

    def __init__(
        self,
        schedule: FaultSchedule = FaultSchedule(),
        noise: SensorNoise = SensorNoise(),
        beacon: BeaconGeometry = BeaconGeometry(),
        seed: int = 20_000_000,
        optical_feedback: Optional[OpticalFeedback] = None,
    ) -> None:
        self.schedule = schedule
        self.noise = noise
        self.beacon = beacon
        #: When set, ``optical_quality`` is estimated from a rendered frame
        #: rather than read from the propagation model. See _reported_quality.
        self.optical_feedback = optical_feedback
        self._texture = seabed_texture(seed=seed % 1_000_003) 
        self._rng = np.random.default_rng(seed)
        # A constant accelerometer bias is what makes dead reckoning genuinely
        # diverge rather than perform a random walk. Drawn once per scenario.
        self._accel_bias = self._rng.normal(0.0, noise.accel_bias_mps2, 3)
        self._last_acoustic_cycle = -1
        # Systematic DVL errors, drawn once per scenario and constant thereafter.
        # Constancy is the point: a scale factor and a mounting rotation that
        # were redrawn each tick would average out and behave as extra white
        # noise, producing no drift at all.
        self._dvl_scale = 1.0 + self._rng.normal(0.0, noise.dvl_scale_error)
        yaw = self._rng.normal(0.0, noise.dvl_misalignment_rad)
        c, s = math.cos(yaw), math.sin(yaw)
        #: Residual yaw rotation between the DVL frame and the vehicle frame.
        #: Yaw only: the study flies level, so roll and pitch misalignment
        #: project onto the vertical channel, which the pressure sensor observes
        #: directly and therefore bounds. Horizontal is where it is not bounded.
        self._dvl_rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def _apply_dvl_errors(self, velocity: np.ndarray) -> np.ndarray:
        """Apply the scenario's fixed scale factor and mounting rotation.

        Both DVL modes see the same distortion because both are produced by the
        same transducer head. That shared error is why the current estimate,
        which is a difference of the two, is far more accurate than either
        velocity: the common rotation and scale largely cancel in the
        subtraction, while they do not cancel in the dead-reckoned position.
        """
        return self._dvl_scale * (self._dvl_rotation @ np.asarray(velocity, dtype=float))

    # -- individual channels ------------------------------------------------
    def _inertial(self, true_accel: np.ndarray) -> np.ndarray:
        """Specific force, as a real accelerometer reports it.

        An accelerometer measures ``f = a - g``, so at rest it reads +9.81 m/s^2
        upward rather than zero. Returning bare acceleration here while the
        filter compensated for gravity injected a phantom 9.81 m/s^2 into every
        predict step and diverged the estimate by kilometres.
        """
        return (
            np.asarray(true_accel, dtype=float)
            - GRAVITY
            + self._accel_bias
            + self._rng.normal(0.0, self.noise.accel_mps2, 3)
        )

    def _dvl(
        self, t: float, true_velocity: np.ndarray
    ) -> tuple[Optional[np.ndarray], bool]:
        if self.schedule.active(FaultKind.DVL_BOTTOM_LOCK_LOSS, t):
            return None, False
        return (
            self._apply_dvl_errors(true_velocity)
            + self._rng.normal(0.0, self.noise.dvl_mps, 3),
            True,
        )

    def _dvl_water_track(
        self, t: float, true_velocity: np.ndarray, true_current: np.ndarray
    ) -> Optional[np.ndarray]:
        """Velocity relative to the water column.

        Bottom track ranges on the seabed and therefore measures velocity over
        ground. Water track ranges on suspended scatterers drifting with the
        flow, and therefore measures velocity through the water. The difference
        between the two is the current, which is how a vehicle carrying a single
        DVL observes the flow it is swimming in without any external reference.

        The current subtracted here is the true one because this is the sensor
        layer, which sits on the evaluator's side of the boundary. Nothing
        downstream receives it: the filter sees only this velocity and, when
        bottom lock is available, the ground velocity, and has to infer the
        difference itself.
        """
        if self.schedule.active(FaultKind.DVL_WATER_TRACK_LOSS, t):
            return None
        relative = np.asarray(true_velocity, dtype=float) - np.asarray(
            true_current, dtype=float
        )
        return self._apply_dvl_errors(relative) + self._rng.normal(
            0.0, self.noise.dvl_water_track_mps, 3
        )

    def _depth(self, true_position: np.ndarray) -> float:
        return float(true_position[2] + self._rng.normal(0.0, self.noise.depth_m))

    def _acoustic(
        self,
        t: float,
        true_position: np.ndarray,
        technique: AcousticTechnique = SINGLE_BEACON,
        noise_state: NoiseState = NoiseState(),
        terrain_gradient: float = 0.0,
        altitude_m: float = 3.0,
    ) -> tuple[Optional[float], Optional[np.ndarray], float]:
        """One acoustic interrogation.

        Returns ``(range, position, sigma)``. Range-only techniques fill the
        first slot, position-fixing techniques the second; the caller admits
        whichever it was given.

        Multipath is applied here rather than as extra variance. When the
        receiver locks onto a surface reflection the reported range is too long
        by the image-source excess and never too short, so the error is one-sided
        and far outside the nominal distribution. A measurement-weighting method
        cannot down-weight its way out of that; only a decision to reject it
        helps, which is the difference this study can then measure.
        """
        # The acoustic outage models loss of the *positioning transponder link*:
        # a beacon that has failed, a blocked or reflected path to it, or a
        # surface vessel that has left station. None of those affect a
        # hull-mounted echo sounder looking straight down at a few metres, so
        # terrain-relative navigation survives this fault. Its own failure mode
        # is a featureless seabed, which is modelled separately and is what the
        # E16 family exists to exercise.
        #
        # Applying the outage to every technique alike made terrain matching
        # unavailable in precisely the family it was added for, and the manager
        # correctly never selected it -- an axis present in the action space and
        # dead in the only place it could have mattered.
        if (
            self.schedule.active(FaultKind.ACOUSTIC_OUTAGE, t)
            and not technique.terrain_relative
        ):
            return None, None, 0.0
        # The vessel has left. Only the technique that depends on it stops; the
        # seabed array and the echo sounder are untouched.
        if (
            self.schedule.active(FaultKind.SURFACE_ASSET_LOSS, t)
            and technique.requires_surface_asset
        ):
            return None, None, 0.0
        # One fix per interrogation cycle, not one per simulation tick. LBL
        # needs several round trips per fix, so its cycle is correspondingly
        # longer -- accuracy bought with update rate.
        period = technique.fix_period_s
        if period > 0.0:
            cycle = int(math.floor((t + 1e-9) / period))
            if cycle == self._last_acoustic_cycle:
                return None, None, 0.0
            self._last_acoustic_cycle = cycle

        position = np.asarray(true_position, dtype=float)
        response = acoustic_response(
            technique, position, noise_state, beacon=self.beacon.position,
            # Truth side: the real relief under the vehicle decides whether a
            # terrain match succeeds and how well. The manager gets its own
            # estimate of this from the altimeter and does not see this value.
            terrain_gradient=terrain_gradient,
            altitude_m=altitude_m,
        )
        if not response.available:
            return None, None, 0.0

        outlier = self._rng.random() < multipath_outlier_rate(noise_state)

        if technique.gives_position:
            reference = (
                technique.transponders[0] if technique.transponders
                else self.beacon.position
            )
            fix = position + self._rng.normal(0.0, response.sigma_m, 3)
            if outlier:
                # A reflected arrival reads long, which pushes the trilaterated
                # position away from the transponder along the line of sight.
                direction = position - np.asarray(reference, dtype=float)
                norm = float(np.linalg.norm(direction))
                if norm > 1e-9:
                    excess = multipath_excess_m(position, reference)
                    fix = fix + direction / norm * excess
            return None, fix, response.sigma_m

        delta = position - np.asarray(self.beacon.position, dtype=float)
        distance = float(np.linalg.norm(delta))
        if distance > self.beacon.maximum_range_m or distance < 1e-6:
            return None, None, 0.0
        horizontal = float(np.linalg.norm(delta[:2]))
        elevation = math.atan2(abs(delta[2]), max(horizontal, 1e-9))
        if elevation < self.beacon.minimum_elevation_rad:
            return None, None, 0.0
        measured = distance + float(self._rng.normal(0.0, response.sigma_m))
        if outlier:
            measured += multipath_excess_m(position, self.beacon.position)
        return measured, None, response.sigma_m

    def _optical(
        self,
        t: float,
        true_position: np.ndarray,
        altitude_m: float,
        water: WaterState,
        config: ChannelConfig,
    ) -> tuple[Optional[np.ndarray], ChannelResponse]:
        response = channel_response(water, altitude_m, config, rng=self._rng)
        if self.schedule.active(FaultKind.OPTICAL_BLACKOUT, t) or not response.available:
            return None, response
        measurement = (
            np.asarray(true_position, dtype=float)
            + response.bias_m
            + self._rng.normal(0.0, response.sigma_m, 3)
        )
        return measurement, response

    def _reported_quality(
        self,
        response: ChannelResponse,
        altitude_m: float,
        water: WaterState,
        config: ChannelConfig,
        blacked_out: bool,
    ) -> float:
        """The optical quality the vehicle is allowed to observe.

        With optical feedback enabled this is estimated from a rendered frame,
        by the same image-only estimator the Gazebo demonstrator runs. Without
        it, the analytic index is reported directly.

        The distinction matters more than it looks. ``channel_response`` computes
        quality from the true water state, which no vehicle can measure. Handing
        that number to the manager leaves the loop this study claims to close
        open at precisely the point of interest: every decision downstream is
        then conditioned on privileged information. Rendering a frame and reading
        the estimate back off it costs about 1.4 ms per decision and makes the
        observable one a real camera could produce.

        Whether a fix actually *arrives* remains governed by the physics above.
        Only the vehicle's belief about conditions is image-derived, which is the
        correct split: a vehicle can misjudge the water, but it cannot talk a
        photon into returning.
        """
        if self.optical_feedback is None:
            return response.quality
        if blacked_out:
            # A blacked-out sensor produces no frame, so there is nothing to
            # analyse. Reporting zero is what the estimator would return for an
            # empty image, and it is what the mode machine must see.
            return 0.0
        frame = render_patch(
            water, altitude_m, config, self._texture, rng=self._rng
        )
        return self.optical_feedback.predict(frame)

    # -- one tick -----------------------------------------------------------
    def sample(
        self,
        t: float,
        true_position: np.ndarray,
        true_velocity: np.ndarray,
        true_accel: np.ndarray,
        altitude_m: float,
        water: WaterState,
        config: ChannelConfig = CAMERA_OFFAXIS,
        true_current_mps: Sequence[float] = (0.0, 0.0, 0.0),
        technique: AcousticTechnique = SINGLE_BEACON,
        noise_state: NoiseState = NoiseState(),
        terrain_gradient: float = 0.0,
    ) -> SensorReading:
        """Produce one tick of measurements.

        The optical bias and the true position are attached for the evaluator.
        They are attributes of the *reading*, not of anything a decision
        component receives: the estimator is handed individual measurements, and
        error is always scored against true position by the evaluator.

        ``true_current_mps`` enters here for the same reason true velocity does:
        the sensor layer is where truth is turned into measurements. It reaches
        the water-track channel and nothing else.
        """
        dvl, bottom_lock = self._dvl(t, true_velocity)
        water_track = self._dvl_water_track(t, true_velocity, np.asarray(true_current_mps, dtype=float))
        optical, response = self._optical(
            t, true_position, altitude_m, water, config
        )
        blacked_out = self.schedule.active(FaultKind.OPTICAL_BLACKOUT, t)
        acoustic_range, acoustic_fix, acoustic_sigma = self._acoustic(
            t, true_position, technique, noise_state,
            terrain_gradient=terrain_gradient, altitude_m=altitude_m,
        )
        return SensorReading(
            t=t,
            accel_mps2=self._inertial(true_accel),
            dvl_velocity_mps=dvl,
            dvl_water_velocity_mps=water_track,
            depth_m=self._depth(true_position),
            optical_position_m=optical,
            optical_sigma_m=response.sigma_m,
            optical_quality=self._reported_quality(
                response, altitude_m, water, config, blacked_out
            ),
            acoustic_range_m=acoustic_range,
            acoustic_position_m=acoustic_fix,
            acoustic_sigma_m=acoustic_sigma,
            acoustic_technique=technique.name,
            dvl_bottom_lock=bottom_lock,
            true_position_m=np.asarray(true_position, dtype=float).copy(),
            optical_bias_m=response.bias_m.copy(),
            channel=config.name,
        )


def dvl_loss_schedule(start_s: float = 40.0, duration_s: float = 100.0) -> FaultSchedule:
    """E3: long bottom-lock loss. Velocity aiding gone for most of the survey."""
    return FaultSchedule(
        windows=(FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, start_s, duration_s),)
    )


def optical_loss_schedule(start_s: float = 40.0, duration_s: float = 100.0) -> FaultSchedule:
    """E5: sustained optical blackout."""
    return FaultSchedule(
        windows=(FaultWindow(FaultKind.OPTICAL_BLACKOUT, start_s, duration_s),)
    )


def compound_schedule(
    optical_start_s: float = 30.0,
    optical_duration_s: float = 110.0,
    dvl_start_s: float = 45.0,
    dvl_duration_s: float = 95.0,
    acoustic_start_s: float = 60.0,
    acoustic_duration_s: float = 45.0,
) -> FaultSchedule:
    """E7: the decisive compound case.

    Overlapping optical, velocity-aiding, and acoustic loss. Durations are long
    relative to the ~180 s survey because short outages do not stress navigation:
    the filter simply coasts and re-latches, and every policy scores the same. A
    scenario in which no method is challenged measures nothing.

    Velocity aiding is faulted in *both* DVL modes. Faulting bottom track alone
    left the vehicle still measuring its velocity through the water throughout,
    which is not a compound aiding loss: dead reckoning then completed the
    scenario with 0.22 m position RMSE and the failure matrix no longer spanned
    the comparator range. The physical case this schedule represents is a DVL
    that has failed, not one that has lost the seabed, and a failed instrument
    returns neither mode. Bottom-lock-only loss remains available separately in
    :func:`dvl_loss_schedule` and :func:`short_dvl_loss_schedule`, which is the
    degraded-but-not-lost case the capability modes distinguish.
    """
    return FaultSchedule(
        windows=(
            FaultWindow(FaultKind.OPTICAL_BLACKOUT, optical_start_s, optical_duration_s),
            FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, dvl_start_s, dvl_duration_s),
            FaultWindow(FaultKind.DVL_WATER_TRACK_LOSS, dvl_start_s, dvl_duration_s),
            FaultWindow(FaultKind.ACOUSTIC_OUTAGE, acoustic_start_s, acoustic_duration_s),
        )
    )


def short_dvl_loss_schedule(start_s: float = 60.0, duration_s: float = 12.0) -> FaultSchedule:
    """E2: short bottom-lock loss. The filter should coast and re-latch.

    Included as a negative control as much as a test: a well-behaved system must
    *not* escalate to a conservative mode for a twelve-second dropout. Excess
    mode churn here is a false-alarm result, not a robustness result.
    """
    return FaultSchedule(
        windows=(FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, start_s, duration_s),)
    )


def surface_asset_loss_schedule(
    start_s: float = 40.0, duration_s: float = 120.0,
    turbidity_start_s: float = 20.0,
) -> FaultSchedule:
    """E18: the support vessel leaves station while the water is turbid.

    The case this family exists for is the one a reader will recognise from
    practice. A vehicle working with ship support uses USBL, which is the most
    accurate technique it has. The ship is then called away -- weather, another
    task, the end of its charter -- and the vehicle is left to finish the survey
    with whatever it can obtain on its own. Turbidity rises at the same time, so
    the optical channel cannot take over.

    The point is not that the vehicle loses a sensor. It is that the *best*
    technique becomes unavailable for a reason that has nothing to do with the
    water, the seabed or the vehicle, and cannot be predicted from any of them.
    Geometry still says USBL would work; there is simply nothing at the surface
    to answer. A manager that infers availability from geometry alone will
    re-select it for the rest of the mission.
    """
    return FaultSchedule(
        windows=(
            FaultWindow(FaultKind.SURFACE_ASSET_LOSS, start_s, start_s + duration_s),
            # Velocity aiding goes at the same time. Without it the vehicle
            # still has bottom-track velocity and barely needs an absolute fix,
            # so losing the best absolute technique costs nothing and the family
            # measures nothing -- which is what the first version of it did.
            FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, start_s, start_s + duration_s),
            FaultWindow(FaultKind.DVL_WATER_TRACK_LOSS, start_s, start_s + duration_s),
        )
    )


def unprepared_area_schedule(
    turbidity_free_s: float = 0.0, horizon_s: float = 400.0,
    dvl_start_s: float = 45.0, dvl_duration_s: float = 120.0,
) -> FaultSchedule:
    """E19: an area with no acoustic infrastructure of any kind.

    Every acoustic positioning technique in the action space depends on
    something someone else has to put in the water and survey: USBL on a
    transceiver held at the surface, LBL on four transponders placed on the
    bottom, single-beacon ranging on one. A vehicle sent to work an area where
    none of that has been done has none of them, for the whole mission, and no
    fault has occurred -- this is simply the deployment.

    That is the case the autonomy argument in this paper is actually about, and
    it was missing from the failure matrix. With velocity aiding also lost and
    the water turning turbid, the only absolute fix left is the seabed itself.

    Modelled as a mission-length acoustic outage rather than as a new fault
    kind, because the vehicle cannot distinguish "no transponder was ever
    deployed here" from "the transponder is not answering". Both are silence.
    """
    return FaultSchedule(
        windows=(
            FaultWindow(FaultKind.ACOUSTIC_OUTAGE, turbidity_free_s, horizon_s),
            FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, dvl_start_s,
                        dvl_start_s + dvl_duration_s),
            FaultWindow(FaultKind.DVL_WATER_TRACK_LOSS, dvl_start_s,
                        dvl_start_s + dvl_duration_s),
        )
    )


def acoustic_duty_cycle_schedule(
    period_s: float = 40.0, off_s: float = 25.0, horizon_s: float = 400.0
) -> FaultSchedule:
    """E6: intermittent acoustic aiding.

    Fix opportunities exist but are periodic, so a manager that plans around
    them should outperform one that reacts to each outage as it arrives.
    """
    windows = []
    t = period_s - off_s
    while t < horizon_s:
        windows.append(FaultWindow(FaultKind.ACOUSTIC_OUTAGE, t, off_s))
        t += period_s
    return FaultSchedule(windows=tuple(windows))


def total_dvl_loss_schedule(
    start_s: float = 50.0, duration_s: float = 100.0
) -> FaultSchedule:
    """E9: both DVL modes lost together, so the current becomes unobservable.

    Bottom track and water track fail for different physical reasons, and E2 and
    the current-adaptation cases exercise them separately. This schedule is the
    case where the vehicle keeps neither: no velocity over ground, no velocity
    through the water, and therefore no new information about the flow.

    It is included because the alternative would flatter the method. With water
    track always available, the current estimate never goes stale and current
    compensation always works; a study that only ever showed that would be
    reporting the behaviour of a schedule rather than of a navigation system.
    Here the estimate must be *held* while its uncertainty grows, and the
    manager has to decide what to do about a compensation it can no longer
    trust.
    """
    return FaultSchedule(
        windows=(
            FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, start_s, duration_s),
            FaultWindow(FaultKind.DVL_WATER_TRACK_LOSS, start_s, duration_s),
        )
    )


def coupled_turbidity_dvl_schedule(
    start_s: float = 45.0, duration_s: float = 110.0
) -> FaultSchedule:
    """E8: velocity aiding lost while the water is too turbid for the default channel.

    Only the DVL is faulted here, but in both of its modes. What makes the
    scenario decisive is the water profile it is paired with, not the schedule:
    turbidity is raised past the point where the nominal camera configuration can
    produce a fix at survey altitude, while a fix remains obtainable from a
    *different* configuration.

    Both DVL modes are faulted for the reason given in :func:`compound_schedule`:
    the cell exists to place the vehicle in genuine need of an absolute fix, and
    a vehicle that still has water track is not in need. It measures its motion
    through the water, its velocity error stays bounded, and its position drifts
    only slowly -- so the pressure to reconfigure the camera, which is the thing
    this cell is designed to measure, disappears. Faulting bottom track alone
    left the proposed manager indistinguishable from the fixed policy and from
    the ablation that takes no action at all.

    This cell was missing from the original failure matrix, and its absence is
    why the first campaigns could not separate the comparators. The argument is
    structural and does not depend on which method wins:

    * A capability-mode manager can only contribute where a capability change is
      available to it.
    * `E4` offers the capability change -- turbidity rises, so descending or
      switching to the laser restores a fix -- but the DVL never fails, so the
      vehicle does not need one. Nothing is at stake.
    * `E7` puts the vehicle in genuine need, but removes optical aiding by hard
      blackout. A blackout removes it from *every* configuration simultaneously,
      so no reconfiguration can help and only tier-2/tier-3 actions remain.
    * Neither cell -- nor any other in the declared matrix -- presents a vehicle
      that both needs an absolute fix and could obtain one by reconfiguring.

    Without such a cell the experiment cannot measure the quantity it exists to
    measure, whatever the outcome. `E8` supplies it.
    """
    return FaultSchedule(
        windows=(
            FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, start_s, duration_s),
            FaultWindow(FaultKind.DVL_WATER_TRACK_LOSS, start_s, duration_s),
        )
    )
