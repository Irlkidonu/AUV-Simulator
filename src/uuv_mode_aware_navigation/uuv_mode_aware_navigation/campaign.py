"""Deterministic scenario runner and campaign driver.

One scenario is: a mission, a water profile, a fault schedule, and a seed. One
run is: that scenario flown by one policy. Every policy in a scenario receives an
identical measurement realisation, because the sensor suite is re-seeded from the
scenario seed rather than continuing a shared stream.

The campaign runs without a simulator, but it is not image-free. With optical
feedback enabled -- which is how the reported campaign runs -- each decision tick
renders a seabed patch through the propagation model and estimates water
condition back off it, exactly as the Gazebo demonstrator does from a real
camera frame. What is absent is Gazebo, not imagery; the Gazebo world is a
qualitative demonstration and contributes no numbers.

The distinction matters for the paper's central claim. If the manager were fed a
quality index computed from the true water state, the loop this study describes
would be open at the one point that defines it, and every decision would rest on
information no vehicle can obtain.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field, replace
from typing import Callable, Iterable, Mapping, Optional, Sequence

import numpy as np

from .acoustics import NOISE_QUIET_DB, NoiseState
from .comparators import PolicyDecision
from .environment import EnvironmentEstimate, EnvironmentFeatures
from .manager import DEFAULT_CANDIDATES, MissionCosts, VehicleConfiguration

#: The declared cost model, used here only to record what a run spent on
#: infrastructure. The manager holds its own instance for selection; this
#: one never influences a decision.
MISSION_COSTS = MissionCosts()
from .estimator import FilterConfig, NavigationFilter
from .mission import (
    Guidance,
    MissionEvaluator,
    MissionOutcome,
    SurveyMission,
    Vehicle,
    saturate_command,
)
from .modes import Observables
from .optics import ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS, WaterState
from .sensors import (
    BeaconGeometry,
    FaultSchedule,
    SensorNoise,
    SensorSuite,
)

__all__ = [
    "Scenario",
    "RunResult",
    "run_scenario",
    "run_campaign",
    "static_sweep",
    "WaterProfile",
    "CurrentProfile",
    "NoiseProfile",
]

#: Proportional gain of the station-keeping controller used while a tier-3 hold
#: is active. Chosen so the commanded velocity saturates at the configuration
#: speed for errors beyond about a metre.
STATION_KEEPING_GAIN = 0.5

#: Ascent rate for the terminal self-preservation action, in m/s.
#: A survey AUV ascends at a fraction of its forward speed; this is half the
#: nominal 0.5 m/s cruise, so surfacing from the 3 m survey altitude over a 20 m
#: water column takes about seventy seconds -- long enough that the drift
#: accumulated on the way up is a real cost the evaluator can see, rather than a
#: teleport to safety.
ASCENT_RATE_MPS = 0.25

#: Depth (m) within which the vehicle counts as surfaced and GPS is available.
#: A vehicle awash has its antenna clear; this is not a claim about a specific
#: hull, it is the depth at which the satellite channel replaces the acoustic
#: and optical ones.
SURFACE_DEPTH_M = 0.5

#: One-sigma horizontal accuracy of the satellite fix once surfaced (m).
#: Standard civilian GNSS. Deliberately worse than a good acoustic fix: this
#: action exists to bound an unbounded error and permit recovery, not to
#: navigate better than the vehicle could underwater.
GPS_SIGMA_M = 2.5

#: Decision ticks of evidence pooled for environment identification. At the
#: 0.5 s decision period this is ten seconds -- long enough to average out a
#: single dark frame, short enough to follow a turbidity ramp.
ENVIRONMENT_WINDOW = 20

#: Development and held-out seed roots (PROTOCOL.md section 8). Paper 2 uses
#: seeds at or above 20,000,000 exclusively; Paper 1 has consumed everything
#: below 13,000,000 across its frozen study and redesign versions.
DEVELOPMENT_SEED_ROOT = 20_000_000
#: Held-out roots, in the order they were reserved. Every one of them is refused
#: to ``--root``; only the last is reachable, and only through the freeze gate.
#:
#: Study 1's block (20,400,000) was executed once on 3 August 2026 and is spent.
#: It stays in this list rather than being deleted, because deleting it would
#: make it reachable again through the ordinary seed argument, and a spent
#: held-out block that can be re-entered is a development block.
HELDOUT_SEED_ROOTS = (20_400_000, 20_800_000)

#: The block Study 2 will spend.
HELDOUT_SEED_ROOT = HELDOUT_SEED_ROOTS[-1]


class CurrentProfile:
    """Ocean current as a function of time. Hidden state -- evaluator side only.

    Piecewise-linear in each world-frame component, interpolated the same way
    :class:`WaterProfile` interpolates turbidity. Like the water state, the
    vehicle is never told this: it reaches the sensor layer, which turns it into
    a water-track velocity, and the filter has to infer the rest.

    Magnitudes used by the scenarios span roughly 0.02--0.25 m/s. That range is
    chosen against the survey speed rather than against oceanography: the
    configurations under comparison fly at 0.25 and 0.50 m/s, so 0.25 m/s of
    flow is the point at which the slow configuration can be held on track only
    by crabbing hard, and beyond it cannot hold track at all. A study whose
    currents were negligible next to vehicle speed would show nothing, and one
    whose currents overwhelmed every configuration would show nothing either.
    """

    def __init__(self, points: Sequence[tuple[float, Sequence[float]]]) -> None:
        if not points:
            raise ValueError("current profile needs at least one point")
        self._t = np.array([p[0] for p in points], dtype=float)
        self._v = np.array([np.asarray(p[1], dtype=float) for p in points])

    def at(self, t: float) -> np.ndarray:
        return np.array(
            [np.interp(t, self._t, self._v[:, i]) for i in range(3)], dtype=float
        )

    @classmethod
    def constant(cls, velocity: Sequence[float]) -> "CurrentProfile":
        return cls([(0.0, velocity)])

    @classmethod
    def ramp(
        cls,
        v0: Sequence[float],
        v1: Sequence[float],
        start_s: float,
        end_s: float,
    ) -> "CurrentProfile":
        """A flow that strengthens or veers over a stated interval."""
        return cls([(0.0, v0), (start_s, v0), (end_s, v1), (end_s + 1e6, v1)])

    @classmethod
    def rotating(
        cls,
        speed_mps: float,
        period_s: float,
        horizon_s: float = 400.0,
        samples: int = 64,
    ) -> "CurrentProfile":
        """A flow of constant strength whose direction turns steadily.

        Included because a constant current is the easy case: it is a bias, and
        anything that estimates a bias removes it. A veering flow keeps the
        estimate perpetually behind the truth by roughly one correlation time,
        so it tests whether the compensation tracks rather than merely converges.
        The period is set per scenario; a semidiurnal tidal ellipse is the
        physical motivation, compressed to survey duration.
        """
        step = horizon_s / samples
        points = []
        for i in range(samples + 1):
            t = i * step
            angle = 2.0 * math.pi * t / period_s
            points.append(
                (t, (speed_mps * math.cos(angle), speed_mps * math.sin(angle), 0.0))
            )
        return cls(points)


class NoiseProfile:
    """Ambient acoustic noise over time. Hidden state -- evaluator side only.

    Spectral level in dB re 1 uPa^2/Hz, the standard unit. Like turbidity and
    current, the vehicle is never told this: it reaches the sensor layer, which
    turns it into a multipath outlier rate, and the vehicle must infer the
    condition from how its own acoustic gate behaves.

    This axis exists because the paper claims the vehicle copes with noisy
    environments. Without a profile every scenario ran at the same default level,
    so the classifier's noise axis was being fitted against a constant label -- a
    score computed from that would have looked perfect and measured nothing.
    """

    def __init__(self, points: Sequence[tuple[float, float]]) -> None:
        if not points:
            raise ValueError("noise profile needs at least one point")
        self._t = np.array([p[0] for p in points], dtype=float)
        self._level = np.array([p[1] for p in points], dtype=float)

    def at(self, t: float) -> NoiseState:
        return NoiseState(spectral_level_db=float(np.interp(t, self._t, self._level)))

    @classmethod
    def constant(cls, level_db: float) -> "NoiseProfile":
        return cls([(0.0, level_db)])

    @classmethod
    def ramp(cls, a: float, b: float, start_s: float, end_s: float) -> "NoiseProfile":
        """Noise rising or falling over an interval -- a vessel passing over."""
        return cls([(0.0, a), (start_s, a), (end_s, b), (end_s + 1e6, b)])


class WaterProfile:
    """Turbidity as a function of time. Hidden state -- evaluator side only."""

    def __init__(self, points: Sequence[tuple[float, float]]) -> None:
        if not points:
            raise ValueError("water profile needs at least one point")
        self._t = np.array([p[0] for p in points], dtype=float)
        self._c = np.array([p[1] for p in points], dtype=float)

    def at(self, t: float) -> WaterState:
        return WaterState(c=float(np.interp(t, self._t, self._c)))

    @classmethod
    def constant(cls, c: float) -> "WaterProfile":
        return cls([(0.0, c)])

    @classmethod
    def ramp(cls, c0: float, c1: float, start_s: float, end_s: float) -> "WaterProfile":
        return cls([(0.0, c0), (start_s, c0), (end_s, c1), (end_s + 1e6, c1)])


class TerrainProfile:
    """Seabed relief as a function of time along the survey. Evaluator side only.

    Carried as a scalar gradient magnitude rather than a bathymetric map, for
    the same reason turbidity is carried as ``c(t)`` rather than a particle
    field: the decision the study is about depends on how much terrain
    information is available, not on its spatial arrangement. A vehicle over a
    ridge field can fix position by terrain matching; one over a sediment plain
    cannot, whatever the plain looks like in detail.

    Stated as a limitation rather than hidden: a real terrain-matching system
    also fails on *repetitive* relief, where the correlation is ambiguous rather
    than absent, and that failure mode is not modelled here.
    """

    def __init__(self, points: Sequence[tuple[float, float]]) -> None:
        if not points:
            raise ValueError("terrain profile needs at least one point")
        self._t = np.array([p[0] for p in points], dtype=float)
        self._g = np.array([p[1] for p in points], dtype=float)

    def at(self, t: float) -> float:
        """Terrain gradient magnitude (m/m) at time ``t``."""
        return float(np.interp(t, self._t, self._g))

    @classmethod
    def constant(cls, gradient: float) -> "TerrainProfile":
        return cls([(0.0, gradient)])

    @classmethod
    def ramp(cls, g0: float, g1: float, start_s: float, end_s: float) -> "TerrainProfile":
        return cls([(0.0, g0), (start_s, g0), (end_s, g1), (end_s + 1e6, g1)])


#: Relief of the default survey area. A gently structured seabed: enough for a
#: terrain match to fix position to roughly 0.05 / 0.12 = 0.4 m, which is worse
#: than USBL and better than a single beacon, so the technique is neither
#: dominant nor useless and the manager has a real choice to make.
BASELINE_TERRAIN_GRADIENT: float = 0.12

#: A sediment plain. Below the technique's identifiability threshold, so a
#: terrain match returns nothing at all.
FEATURELESS_TERRAIN_GRADIENT: float = 0.005


@dataclass(frozen=True)
class Scenario:
    """A fully specified, reproducible experimental condition."""

    name: str
    seed: int
    water: WaterProfile
    schedule: FaultSchedule = FaultSchedule()
    mission: SurveyMission = field(default_factory=SurveyMission)
    #: Ocean current over time. The default is the weak residual flow the study
    #: used before currents were varied, retained so that scenarios which are
    #: about something else are unchanged by this axis.
    current: CurrentProfile = field(
        default_factory=lambda: CurrentProfile.constant((0.02, -0.01, 0.0))
    )
    #: Ambient acoustic noise over time. Defaults to the quiet reference so
    #: scenarios that are about something else are unchanged by this axis.
    noise: "NoiseProfile" = field(
        default_factory=lambda: NoiseProfile.constant(NOISE_QUIET_DB)
    )
    #: Seabed relief over the survey. Defaults to the gently structured seabed,
    #: so scenarios that are about something else are unchanged by this axis.
    terrain: "TerrainProfile" = field(
        default_factory=lambda: TerrainProfile.constant(BASELINE_TERRAIN_GRADIENT)
    )
    #: Whether a prior bathymetric survey of this area exists.
    #:
    #: Terrain-relative navigation matches measured depth against a map, and
    #: without the map there is nothing to match against. This is a property of
    #: the *area*, not a fault: the map either was made or was not.
    #:
    #: Modelled separately from relief because the two are independent. A ridge
    #: field nobody has surveyed carries plenty of information and is still
    #: unusable; a mapped plain is mapped and still featureless.
    #:
    #: It is the same class of dependency as a surveyed transponder array or a
    #: vessel on station, and it was previously modelled only as a cost. That
    #: understated it: a technique whose infrastructure is merely expensive is
    #: available everywhere, and one whose infrastructure is absent is not
    #: available at all.
    prior_map: bool = True
    dt: float = 0.1
    decision_period_s: float = 0.5


@dataclass
class RunResult:
    """Outcome of one policy flying one scenario."""

    scenario: str
    policy: str
    seed: int
    outcome: MissionOutcome
    mode_transitions: int = 0
    channel_switches: int = 0
    mean_altitude_m: float = 0.0
    detection_latency_s: Optional[float] = None
    false_alarms: int = 0
    #: Did the manager escalate to terminal self-preservation, and did the
    #: vehicle actually reach the surface? Reported separately on purpose: in the
    #: regime where this fires, cross-track error is not a meaningful score --
    #: the survey has been abandoned deliberately -- and what matters instead is
    #: whether the vehicle became recoverable. A run that decides to surface and
    #: never gets there has preserved nothing.
    surfacing_commanded: bool = False
    surfaced: bool = False

    def to_row(self) -> dict:
        row = {
            "scenario": self.scenario,
            "policy": self.policy,
            "seed": self.seed,
            "mode_transitions": self.mode_transitions,
            "channel_switches": self.channel_switches,
            "detection_latency_s": self.detection_latency_s,
            "false_alarms": self.false_alarms,
            "surfacing_commanded": self.surfacing_commanded,
            "surfaced": self.surfaced,
        }
        row.update(asdict(self.outcome))
        return row


def _quality_trend(window) -> float:
    """Change in observed optical quality across the identification window.

    Difference of the second-half mean and the first-half mean, rather than a
    fitted slope: it costs two means, is insensitive to a single outlying frame,
    and carries the same sign. The value is in quality units per window, so a
    trend of -0.2 means conditions have lost a fifth of full quality over the
    last ten seconds.
    """
    if len(window) < 4:
        return 0.0
    qualities = [w[0] for w in window]
    half = len(qualities) // 2
    return float(np.mean(qualities[half:]) - np.mean(qualities[:half]))


#: Altimeter samples used to estimate terrain relief. At dt = 0.1 s this is a
#: six-second window, long enough to cross a few metres of seabed at survey
#: speed and short enough that the estimate tracks a change in terrain.
TERRAIN_WINDOW = 60


#: Along-track distance spanned by the terrain window at survey speed, in
#: metres. Used to scale relief so that a gradient of g m/m produces altitude
#: variation of g * TERRAIN_WINDOW_LENGTH_M across the window, which is what
#: makes the altimeter estimate recover the gradient rather than some multiple
#: of it.
TERRAIN_WINDOW_LENGTH_M: float = 3.0


def _terrain_relief(t: float, gradient: float) -> float:
    """Height of the seabed under the vehicle relative to its local mean.

    Three incommensurate sinusoids, normalised to unit standard deviation and
    scaled so that the relief crossed in one estimation window is the gradient
    times the window length. Incommensurate because a single sinusoid is
    periodic with the window and produces a variance that depends on where in
    the cycle the window happens to fall, which showed up immediately as an
    altimeter estimate that read 0.05 m/m on terrain of 0.12.

    Deterministic in ``t`` rather than drawn per seed: terrain is a property of
    the place, not of the run, and two policies flying the same scenario must
    cross the same seabed or the comparison is not controlled.
    """
    shape = (
        math.sin(0.35 * t)
        + math.sin(0.61 * t + 1.3)
        + math.sin(0.13 * t + 2.7)
    ) / math.sqrt(1.5)
    return float(gradient) * TERRAIN_WINDOW_LENGTH_M * shape


def _terrain_gradient_estimate(altitude_history, speed_mps: float) -> float:
    """Terrain relief inferred from the vehicle's own altimeter.

    A vehicle holding a commanded depth over rising and falling seabed sees its
    measured altitude vary; over a plain it does not. Dividing the variability
    of altitude by the distance travelled while accumulating it gives a gradient
    in metres per metre, which is exactly the quantity a terrain match needs in
    order to predict its own accuracy.

    Standard deviation rather than a fitted slope, because the relevant question
    is how much terrain *information* passed under the vehicle, not which way it
    sloped: a ridge crossed at right angles carries information whether the
    vehicle went up it or down it.

    This is an estimate from an onboard instrument and it is wrong in the usual
    ways -- a short window over an atypical patch misreports the wider area, and
    altimeter noise inflates the estimate on genuinely flat ground. The floor
    imposed here is the noise level itself: variability below what the
    instrument contributes on its own is not evidence of terrain.
    """
    if len(altitude_history) < TERRAIN_WINDOW // 2:
        return 0.0
    alt = np.asarray(list(altitude_history)[-TERRAIN_WINDOW:], dtype=float)
    distance = TERRAIN_WINDOW_LENGTH_M
    variability = float(np.std(alt))
    # Altimeter noise alone produces a non-zero standard deviation. Subtracting
    # it in quadrature keeps a flat seabed reading flat instead of reading as
    # weak relief, which would make the manager select terrain matching over a
    # plain -- the one place it must not.
    noise_floor = 0.02
    corrected = max(variability * variability - noise_floor * noise_floor, 0.0) ** 0.5
    return corrected / distance


def _covariance_growth_rate(trace_history, decision_period_s: float) -> float:
    """Observed rate of growth of position uncertainty, in m^2 per second.

    This was previously a two-valued constant -- zero when aided, a fixed 0.05
    otherwise -- which made the manager's projection of the unaided branch
    identical in every scenario. A vehicle that has lost bottom lock in a strong
    current accumulates uncertainty far faster than one drifting in still water,
    and the whole point of projecting forward is to notice the difference.

    Measured as the slope across the retained window rather than the last step,
    so a single filter update does not dominate it. Negative growth is clamped
    to zero: shrinking uncertainty is reported by the aided branch, and a
    negative rate here would credit an unaided vehicle with improving.
    """
    if len(trace_history) < 2 or decision_period_s <= 0.0:
        return 0.0
    span = (len(trace_history) - 1) * decision_period_s
    if span <= 0.0:
        return 0.0
    return max((trace_history[-1] - trace_history[0]) / span, 0.0)


def run_scenario(
    scenario: Scenario,
    policy,
    filter_config: FilterConfig = FilterConfig(),
    optical_feedback=None,
    classifier=None,
) -> RunResult:
    """Fly one scenario with one policy.

    The loop enforces the information boundary explicitly:

    * ``guidance`` receives ``estimator.position`` and nothing else;
    * ``policy`` receives ``Observables``, which cannot carry water state;
    * ``evaluator`` receives true position, and its output influences nothing.
    """
    mission = scenario.mission
    start = mission.waypoints[0].copy()
    vehicle = Vehicle(start, scenario.current.at(0.0))
    estimator = NavigationFilter(filter_config, initial_position=start)
    guidance = Guidance(mission)
    evaluator = MissionEvaluator(mission)
    sensors = SensorSuite(
        schedule=scenario.schedule,
        noise=SensorNoise(),
        beacon=BeaconGeometry(),
        seed=scenario.seed,
        optical_feedback=optical_feedback,
    )

    decision = PolicyDecision(
        configuration=VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, 0.5)
    )

    t = 0.0
    since_decision = math.inf
    previous_channel = decision.configuration.optical.name
    channel_switches = 0
    mode_transitions = 0
    previous_mode = None
    first_detection: Optional[float] = None
    false_alarms = 0
    true_accel = np.zeros(3)
    hold_target: Optional[np.ndarray] = None
    #: Whether the manager escalated to terminal self-preservation, and whether
    #: the vehicle actually reached the surface. Both are reported: a manager
    #: that decides to surface and then fails to get there has not preserved
    #: anything, and the distinction is the whole point of measuring rather than
    #: asserting the behaviour.
    surfacing = False
    surfaced = False
    #: Independent stream for the satellite fix, so adding this action cannot
    #: shift any other measurement realisation and change results elsewhere.
    gps_rng = np.random.default_rng(scenario.seed + 977)

    # Rolling evidence for environment identification. A window rather than the
    # instantaneous value because conditions evolve and a single sample cannot
    # separate "the water clouded" from "the vehicle crossed a dark patch".
    window = deque(maxlen=ENVIRONMENT_WINDOW)
    acoustic_nis: deque = deque(maxlen=ENVIRONMENT_WINDOW)
    #: Position-covariance trace at each decision, for the observed growth rate.
    #: Short: the manager is projecting the next horizon, not the mission.
    trace_history: deque = deque(maxlen=5)
    altitude_history: deque = deque(maxlen=TERRAIN_WINDOW)
    #: Simulation time of the last acoustic measurement of any kind.
    #:
    #: Reported to the manager as an age. It was previously reported as 0.0 when
    #: a fix arrived on this exact tick and 60.0 otherwise, which is a flag
    #: rather than an age and is wrong by construction: fixes arrive once per
    #: interrogation cycle of two to six seconds while decisions are taken every
    #: half second, so a healthy channel reported "60 seconds since the last
    #: fix" on three ticks in four. Any rule reading it as an age fired
    #: continuously in nominal water -- measured at 74.9% of decisions in E1.
    #:
    #: Initialised to the start of the run rather than to a large sentinel, so
    #: that the age before the first fix is the time actually elapsed.
    last_acoustic_fix_t: float = 0.0
    infrastructure_samples: list = []
    acoustic_offered = 0
    acoustic_rejected = 0
    environment: Optional[EnvironmentEstimate] = None

    # The run ends when the survey is done or the clock expires -- but a vehicle
    # executing the terminal self-preservation action has abandoned the survey,
    # so ``guidance.complete`` must not end it.
    #
    # Guidance advances on the *estimate*, and in a blackout the estimate drifts;
    # it therefore walked through the remaining waypoints and declared the survey
    # finished while the real vehicle was still ascending. Every compound-scenario
    # run commanded surfacing and not one reached the surface: the ascent needs
    # 68 s from survey altitude and the run was being cut at ~257 s of a 400 s
    # budget. The vehicle decided to save itself and the simulation stopped
    # underneath it.
    while t < mission.time_limit_s and not (guidance.complete and not surfacing):
        water = scenario.water.at(t)
        # The true current is advanced here and handed only to the vehicle and
        # the sensor layer. No decision component reads it.
        vehicle.current = scenario.current.at(t)
        altitude = float(vehicle.position[2] - mission.seabed_depth_m)
        # What the altimeter reports: true altitude modulated by the relief
        # passing underneath. The vehicle never sees the terrain profile, only
        # this reading, which is the sole channel by which relief becomes
        # observable to it.
        altitude_history.append(
            altitude + _terrain_relief(t, scenario.terrain.at(t))
        )

        _tech = decision.configuration.acoustic
        infrastructure_samples.append(
            MISSION_COSTS.transponder_penalty * _tech.interrogations_per_fix
            + (MISSION_COSTS.surface_asset_penalty
               if _tech.requires_surface_asset else 0.0)
            + (MISSION_COSTS.prior_map_penalty if _tech.terrain_relative else 0.0)
        )
        reading = sensors.sample(
            t=t,
            true_position=vehicle.position,
            true_velocity=vehicle.velocity,
            true_accel=true_accel,
            altitude_m=max(altitude, 0.05),
            water=water,
            config=decision.configuration.optical,
            true_current_mps=vehicle.current,
            noise_state=scenario.noise.at(t),
            technique=decision.configuration.acoustic,
            # No map, no match: the gradient is irrelevant if there is nothing
            # to correlate the measured profile against.
            terrain_gradient=(
                scenario.terrain.at(t) if scenario.prior_map else 0.0
            ),
        )

        # --- estimation -----------------------------------------------------
        # The admission strategy is part of the selected configuration, so the
        # filter's mathematics stay shared while the *decision* about which
        # measurements to admit belongs to the policy (fairness rule R1).
        estimator.fusion = decision.configuration.fusion
        estimator.predict(reading.accel_mps2, scenario.dt)
        if reading.dvl_velocity_mps is not None:
            estimator.update_velocity(reading.dvl_velocity_mps)
        else:
            estimator.note_aiding_outage()
        if reading.dvl_water_velocity_mps is not None:
            estimator.update_water_velocity(reading.dvl_water_velocity_mps)
        if reading.depth_m is not None:
            estimator.update_depth(reading.depth_m)

        aided = False
        if decision.use_absolute_aiding:
            if reading.optical_position_m is not None:
                sigma = reading.optical_sigma_m * math.sqrt(
                    decision.optical_covariance_scale
                )
                if estimator.update_position(reading.optical_position_m, sigma).accepted:
                    aided = True
            if reading.acoustic_position_m is not None:
                # LBL and USBL deliver a position, not a range. It enters the
                # same filter through the same gate as an optical fix, so the
                # techniques are compared on equal terms.
                outcome = estimator.update_position(
                    reading.acoustic_position_m, reading.acoustic_sigma_m
                )
                acoustic_offered += 1
                if outcome.accepted:
                    aided = True
                    acoustic_nis.append(outcome.nis)
                else:
                    acoustic_rejected += 1
            if reading.acoustic_range_m is not None:
                outcome = estimator.update_acoustic_range(
                    reading.acoustic_range_m, sensors.beacon.position
                )
                # Acoustic gate outcomes are the vehicle's only observable
                # evidence about how reverberant its surroundings are. Multipath
                # outliers are large and one-sided, so a rising rejection rate
                # says "noisy environment" without any dedicated instrument.
                acoustic_offered += 1
                if outcome.accepted:
                    aided = True
                    acoustic_nis.append(outcome.nis)
                else:
                    acoustic_rejected += 1

        window.append(
            (reading.optical_quality, 1.0 if reading.optical_position_m is not None else 0.0)
        )

        # --- decision -------------------------------------------------------
        since_decision += scenario.dt
        if since_decision >= scenario.decision_period_s:
            # Privileged channel, offered only to policies that declare it. The
            # oracle (C5) is the sole taker, and the paper labels it as such
            # wherever it appears.
            truth_sink = getattr(policy, "observe_truth", None)
            if truth_sink is not None:
                truth_sink(t, scenario.water)
            if (
                reading.acoustic_range_m is not None
                or reading.acoustic_position_m is not None
            ):
                last_acoustic_fix_t = t
            trace_history.append(estimator.position_covariance_trace)
            obs = Observables(
                optical_quality=reading.optical_quality,
                optical_available=reading.optical_position_m is not None,
                dvl_bottom_lock=reading.dvl_bottom_lock,
                dvl_water_track=reading.dvl_water_velocity_mps is not None,
                dvl_age_s=0.0 if reading.dvl_bottom_lock else 5.0,
                acoustic_fix_age_s=t - last_acoustic_fix_t,
                imu_age_s=0.0,
                depth_age_s=0.0,
                position_covariance_trace=estimator.position_covariance_trace,
                covariance_growth_rate=_covariance_growth_rate(
                    trace_history, scenario.decision_period_s
                ),
                innovation_exceedance_rate=estimator.innovation_exceedance_rate,
                # The filter's estimate, never truth: the manager predicts its
                # own acoustic geometry from where it believes it is, and a
                # vehicle that has drifted mis-predicts it.
                estimated_position_m=tuple(float(v) for v in estimator.position),
                altitude_m=max(altitude, 0.05),
                current_speed_mps=float(np.linalg.norm(estimator.current)),
                current_covariance_trace=estimator.current_covariance_trace,
                optical_quality_trend=_quality_trend(window),
                # Inferred from the altimeter the vehicle already carries. The
                # scenario's true terrain profile reaches the sensor layer and
                # stops there; this is what the manager gets.
                terrain_gradient_estimate=_terrain_gradient_estimate(
                    altitude_history, float(np.linalg.norm(estimator.velocity))
                ),
            )
            if classifier is not None and window:
                qualities = np.array([w[0] for w in window], dtype=float)
                fixes = np.array([w[1] for w in window], dtype=float)
                features = EnvironmentFeatures(
                    optical_quality_mean=float(qualities.mean()),
                    optical_quality_std=float(qualities.std()),
                    optical_fix_rate=float(fixes.mean()),
                    acoustic_reject_rate=(
                        acoustic_rejected / acoustic_offered
                        if acoustic_offered else 0.0
                    ),
                    acoustic_nis_mean=(
                        float(np.mean(acoustic_nis)) if acoustic_nis else 0.0
                    ),
                    current_speed_mps=float(np.linalg.norm(estimator.current)),
                    current_sigma_mps=math.sqrt(
                        max(estimator.current_covariance_trace, 0.0) / 3.0
                    ),
                )
                environment = classifier.classify(features)
                obs = replace(
                    obs,
                    environment_turbidity=environment.turbidity,
                    environment_noise=environment.noise,
                    environment_current=environment.current,
                    environment_confidence=environment.minimum_confidence,
                )
            decision = policy.update(obs, since_decision, t)
            since_decision = 0.0

            if decision.configuration.optical.name != previous_channel:
                channel_switches += 1
                previous_channel = decision.configuration.optical.name
            if decision.mode is not None and decision.mode is not previous_mode:
                mode_transitions += 1
                previous_mode = decision.mode
                if scenario.schedule.any_active(t) and first_detection is None:
                    first_detection = t
                elif not scenario.schedule.any_active(t):
                    false_alarms += 1

        # --- guidance and motion (estimate only) ----------------------------
        if decision.configuration.mission_action.value != "surface_for_gps":
            command = guidance.command(
                estimator.position,
                decision.configuration.speed_mps,
                decision.configuration.altitude_m,
                current_estimate_mps=estimator.current,
            )
        if decision.configuration.mission_action.value == "surface_for_gps":
            # Terminal self-preservation: go up, and keep going up.
            #
            # Every other action tries to keep surveying. This one concedes the
            # survey and protects the vehicle, because the alternative to a
            # recovered AUV with a spoiled dataset is a lost AUV. Ascent is the
            # only action that restores an absolute fix by changing the problem
            # rather than the sensor: the surface has GPS however turbid or
            # noisy the water below it was.
            #
            # Horizontal command is zero. The vehicle does not know where it is
            # -- that is why it is surfacing -- so steering toward a waypoint
            # would spend the remaining position confidence on a destination it
            # cannot verify. Rising straight up keeps the horizontal error at
            # whatever it already was instead of adding to it.
            desired_ground = np.zeros(3)
            desired_ground[2] = ASCENT_RATE_MPS
            command = saturate_command(desired_ground, estimator.current)
            surfacing = True
        elif decision.configuration.mission_action.value != "continue":
            # Station-keeping, not zero thrust. A UUV told to hold for a fix
            # actively holds position against the current; commanding zero
            # velocity is not holding, it is drifting.
            #
            # The distinction is not cosmetic. Under free drift the vehicle is
            # carried by the current for as long as the hold lasts, and because
            # an unbounded hold ran to the mission time limit the displacement
            # accumulated over the whole run rather than over the outage. In the
            # coupled turbidity/DVL-loss scenario that produced a measured RMS
            # cross-track error of 4.6 m against 1.2 m for the fixed policy: the
            # tier-3 action that exists to *protect* the mission was the largest
            # single source of path error in the campaign.
            #
            # The hold target is captured from the estimate at hold onset, so
            # this remains an estimate-only control law (rule N1). A vehicle
            # whose estimate is drifting will hold the wrong point, and the
            # evaluator scores that honestly.
            if hold_target is None:
                hold_target = estimator.position.copy()
            error = hold_target - estimator.position
            # Desired velocity *over ground* is zero plus a proportional pull
            # back to the hold point, rate-limited to the configuration speed.
            desired_ground = error * STATION_KEEPING_GAIN
            speed = float(np.linalg.norm(desired_ground))
            if speed > decision.configuration.speed_mps:
                desired_ground = desired_ground / speed * decision.configuration.speed_mps
            # Holding a point over ground in a moving fluid means swimming
            # upstream at the speed of the flow indefinitely. Subtracting the
            # current estimate here is what turns "stop" into "hold", and the
            # thrust limit is what makes a hold in a strong current fail rather
            # than succeed for free.
            command = saturate_command(desired_ground, estimator.current)
        else:
            hold_target = None
        true_accel = vehicle.step(command, scenario.dt)

        # --- satellite fix once the vehicle is awash -------------------------
        #
        # This is what makes surfacing an action rather than a gesture. Without
        # it the vehicle would ascend and keep drifting blind, and the terminal
        # action would be a relabelled failure -- which is exactly the criticism
        # this study has levelled at other work, so it cannot be committed here.
        #
        # The fix is deliberately coarse. GPS at 2.5 m one-sigma is worse than a
        # healthy acoustic fix and far worse than an optical one; what it does is
        # convert an error growing without bound into a bounded one, which is the
        # difference between a recoverable vehicle and a lost one.
        #
        # Depth is read from truth because breaking the surface is a physical
        # event, not an inference: a vehicle is awash or it is not, regardless of
        # what it believes. Nothing about the *decision* to surface uses truth --
        # that came from observables alone in ``_blackout`` -- and the resulting
        # fix enters the filter as a measurement like any other.
        if vehicle.position[2] > -SURFACE_DEPTH_M:
            surfaced = True
            gps = vehicle.position + gps_rng.normal(0.0, GPS_SIGMA_M, size=3)
            gps[2] = vehicle.position[2]  # depth is known from pressure
            estimator.update_position(gps, GPS_SIGMA_M)

        # --- scoring (truth only) -------------------------------------------
        evaluator.record(
            true_position=vehicle.position,
            estimated_position=estimator.position,
            aided=aided,
            waypoints_captured=guidance.index,
        )
        t += scenario.dt

    outcome = evaluator.finish(
        elapsed_s=t, path_length_m=vehicle.path_length_m, completed=guidance.complete
    )
    latency = None
    if first_detection is not None:
        onsets = [w.start_s for w in scenario.schedule.windows]
        if onsets:
            latency = first_detection - min(onsets)

    outcome.acoustic_infrastructure_cost = (
        float(np.mean(infrastructure_samples)) if infrastructure_samples else 0.0
    )

    return RunResult(
        scenario=scenario.name,
        policy=getattr(policy, "name", policy.__class__.__name__),
        seed=scenario.seed,
        outcome=outcome,
        mode_transitions=mode_transitions,
        channel_switches=channel_switches,
        mean_altitude_m=outcome.mean_altitude_m,
        detection_latency_s=latency,
        false_alarms=false_alarms,
        surfacing_commanded=surfacing,
        surfaced=surfaced,
    )


def static_sweep(
    scenarios: Iterable[Scenario],
    candidates: Sequence = DEFAULT_CANDIDATES,
    optical_feedback=None,
) -> list[RunResult]:
    """Fly every scenario with every *static* configuration in the action space.

    This single sweep does two jobs that both bear directly on the credibility
    of the comparison.

    **It selects `C1`.** The best fixed policy is chosen as the best member of
    this sweep, over the whole development family, rather than assumed. The full
    table is published, so "did you cripple the baseline?" is answerable by
    inspection rather than by trust.

    **It bounds `C5`.** Per scenario *and per seed*, the best member of this
    sweep is the outcome a clairvoyant would obtain by picking the right static
    configuration with full hindsight, for that exact noise realisation.

    The second use is what makes the bracket sound. Note what it implies: if the
    proposed manager beats this bound, it has beaten every static configuration
    chosen with hindsight, which would mean the benefit comes from *reconfiguring
    during the run* and not from having picked well. That is a result worth
    reporting on its own terms -- but it must be reported as that, explicitly,
    and never folded silently into a recovery fraction.
    """
    from .comparators import FixedPolicy

    results: list[RunResult] = []
    for scenario in scenarios:
        for config in candidates:
            result = run_scenario(
                scenario,
                FixedPolicy(config, name=config.name),
                optical_feedback=optical_feedback,
            )
            result.policy = config.name
            results.append(result)
    return results


def run_campaign(
    scenarios: Iterable[Scenario],
    policy_factory: Callable[[Scenario], Mapping[str, object]],
) -> list[RunResult]:
    """Run every policy on every scenario.

    ``policy_factory`` is called per scenario so each run starts from a clean
    policy state, and so the oracle can be constructed with that scenario's
    schedule.
    """
    results: list[RunResult] = []
    for scenario in scenarios:
        for name, policy in policy_factory(scenario).items():
            result = run_scenario(scenario, policy)
            result.policy = name
            results.append(result)
    return results
