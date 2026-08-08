"""Survey mission, guidance, vehicle motion, and mission-level scoring.

This module is where Paper 2's central distinction becomes structural rather than
rhetorical.

**Localization** is producing a state estimate. **Navigation** is the closed loop
in which guidance and control consume *only* that estimate, together with
mission-level decisions about whether, how fast, and along what path to continue.

The invalidated earlier system in this workspace ran its waypoint controller on
``/uuv/ground_truth`` by default, so the control loop never saw estimator error
and every reported "navigation" number was localization error wearing a hat.

Here the separation is enforced by construction:

* :class:`Guidance` receives an estimate. It has no access to true state at all.
* :class:`MissionEvaluator` receives true state. It is scoring only, and runs
  after the fact; nothing it computes flows back into a decision.

A method that believes it is on-path while actually off-path is penalised, which
is exactly the failure mode a localization-only evaluation cannot see.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .optics import ALTITUDE_NOMINAL_M

__all__ = [
    "SurveyMission",
    "Guidance",
    "Vehicle",
    "MissionEvaluator",
    "MissionOutcome",
    "lawnmower",
    "saturate_command",
    "MAX_WATER_SPEED_MPS",
]

#: Maximum velocity the vehicle can achieve through the water (m/s).
#:
#: This bound is what stops current compensation from being free. Crabbing into a
#: flow costs thrust, and a vehicle asked to make 0.5 m/s over ground against a
#: 0.3 m/s head current must push 0.8 m/s through the water. Beyond this limit it
#: simply cannot, and the along-track speed falls -- or, in a strong enough beam
#: current, the track cannot be held at any heading.
#:
#: Without a limit the compensation would silently grant the vehicle unbounded
#: thrust, current strength would stop mattering, and the paper's claim to adapt
#: to ocean currents would be a claim about an unphysical vehicle. The value is
#: the BlueROV2 forward speed figure quoted by the manufacturer, matching the
#: platform modelled in the Gazebo world; it is a platform property and is not
#: tuned.
MAX_WATER_SPEED_MPS: float = 1.0

#: Lookahead distance for line-of-sight path following (m). Roughly three
#: vehicle lengths for the platform modelled here. It sets how aggressively the
#: vehicle converges onto the survey line: the commanded course makes an angle
#: ``atan(cross_track / LOOKAHEAD_M)`` with the line.
LOOKAHEAD_M: float = 5.0

#: Proportional gain on depth error, and the vertical speed it saturates at.
#: Holding a commanded height above the seabed is a separate control loop from
#: following the survey line, which is how AUV guidance is normally arranged.
#: 0.3 m/s is a realistic vertical rate for the platform modelled here, and the
#: gain saturates it beyond about 0.6 m of depth error.
DEPTH_GAIN: float = 0.5
MAX_VERTICAL_SPEED_MPS: float = 0.3


def saturate_command(
    desired_ground_velocity: np.ndarray,
    current_estimate_mps: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Convert a desired ground velocity into an achievable water-relative one.

    Returns ``desired - current``, scaled back to :data:`MAX_WATER_SPEED_MPS` if
    that exceeds what the vehicle can produce. Scaling the whole vector rather
    than clipping per axis preserves the commanded direction through the water,
    which is the behaviour of a vehicle that saturates its thrusters while
    holding heading.
    """
    desired = np.asarray(desired_ground_velocity, dtype=float)
    if current_estimate_mps is None:
        command = desired
    else:
        command = desired - np.asarray(current_estimate_mps, dtype=float)
    speed = float(np.linalg.norm(command))
    if speed > MAX_WATER_SPEED_MPS:
        command = command / speed * MAX_WATER_SPEED_MPS
    return command


def lawnmower(
    legs: int = 4,
    leg_length_m: float = 20.0,
    spacing_m: float = 6.0,
    depth_m: float = -17.0,
) -> tuple[np.ndarray, ...]:
    """A standard survey pattern: parallel legs joined by cross-track hops."""
    points: list[np.ndarray] = []
    for i in range(legs):
        y = -spacing_m * (legs - 1) / 2.0 + i * spacing_m
        x0, x1 = (-leg_length_m / 2.0, leg_length_m / 2.0)
        if i % 2:
            x0, x1 = x1, x0
        points.append(np.array([x0, y, depth_m]))
        points.append(np.array([x1, y, depth_m]))
    return tuple(points)


@dataclass(frozen=True)
class SurveyMission:
    """The mission every method is asked to fly. Identical across comparators.

    ``survey_tolerance_m`` is the coverage requirement: a waypoint counts as
    surveyed only if the vehicle **truly** passed within this distance of it. It
    is deliberately smaller than half the line spacing, so a vehicle that drifts
    onto the neighbouring leg cannot be credited with covering this one.

    This tolerance is what makes the failed-mission rate an informative metric.
    Judging completion on the guidance index instead -- as an earlier version of
    this module did -- is a tautology: guidance advances when the *estimate*
    reaches the waypoint, and guidance is what drives the estimate there, so
    every method completes every mission by construction and the metric measures
    nothing.
    """

    waypoints: tuple[np.ndarray, ...] = field(default_factory=lawnmower)
    #: Distance from a waypoint at which guidance switches to the next one.
    #:
    #: One vehicle length for the platform modelled in the Gazebo world. The
    #: earlier 1.5 m was a quarter of the 6 m line spacing and had no stated
    #: basis; combined with pure-pursuit guidance it made cross-track error a
    #: measurement of itself.
    #:
    #: It is reported rather than tuned, because line-keeping error remains
    #: sensitive to it even under the line-of-sight law: over capture radii of
    #: 0.50, 0.75 and 1.00 m the ratio of dead-reckoned to optically aided RMS
    #: cross-track error was 2.20, 1.52 and 1.23, while the two policies'
    #: terminal position errors -- 0.42 m and 0.09 m -- did not move at all.
    #: Terminal position error is therefore the metric that is robust to this
    #: choice, and cross-track error is reported beside it rather than alone.
    capture_radius_m: float = 0.5
    survey_tolerance_m: float = 2.5
    #: Distance from either end of a leg within which cross-track error is not
    #: scored, because the vehicle is turning. Set to twice the capture radius so
    #: that the whole corner-cutting arc is excluded rather than its tail only:
    #: guidance switches target at ``capture_radius_m`` from the waypoint, and
    #: the vehicle needs comparable distance again to settle onto the new line.
    turn_exclusion_m: float = 3.0
    time_limit_s: float = 400.0
    geofence_half_extent_m: float = 30.0
    altitude_band_m: tuple[float, float] = (0.5, 6.0)
    seabed_depth_m: float = -20.0

    def total_path_length_m(self) -> float:
        return float(
            sum(
                np.linalg.norm(b - a)
                for a, b in zip(self.waypoints, self.waypoints[1:])
            )
        )


class Guidance:
    """Waypoint-following guidance driven exclusively by the estimate.

    Rule N1: this class never receives true state. The only pose it can act on is
    the one the estimator believes.
    """

    def __init__(self, mission: SurveyMission) -> None:
        self.mission = mission
        self.index = 0

    @property
    def complete(self) -> bool:
        return self.index >= len(self.mission.waypoints)

    @property
    def target(self) -> Optional[np.ndarray]:
        if self.complete:
            return None
        return self.mission.waypoints[self.index]

    def command(
        self,
        estimated_position: np.ndarray,
        speed_mps: float,
        altitude_m: float,
        capture_radius_m: Optional[float] = None,
        current_estimate_mps: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Return a commanded velocity **through the water**, in world frame.

        ``altitude_m`` comes from the manager: holding a commanded height above
        the seabed is a guidance action, not a sensing one.

        Current compensation
        --------------------
        The thrusters produce motion through the water; the ground track is that
        motion plus the flow. To make the ground track point at the waypoint, the
        commanded water-relative velocity must be the desired ground velocity
        *minus* the current -- the vehicle crabs into the flow. Without this the
        track settles at a standing offset from the survey line, because pure
        waypoint pursuit corrects toward the target at the same rate the flow
        carries the vehicle off it.

        ``current_estimate_mps`` is the filter's estimate, never the true flow.
        A wrong estimate produces a wrong crab angle and a worse track, and the
        evaluator scores that honestly.

        This compensation is part of the shared guidance law and therefore
        applies identically to the proposed manager, every comparator, and every
        ablation. It is deliberately *not* a feature of the proposed method:
        making current compensation exclusive to it would be precisely the
        crippled-baseline failure this study exists to avoid, since every
        published station-keeping or transect controller compensates for set and
        drift. What the manager may do with the current is decide whether an
        action such as holding station is still affordable -- not whether to
        steer correctly.
        """
        if self.complete:
            return np.zeros(3)
        radius = (
            capture_radius_m
            if capture_radius_m is not None
            else self.mission.capture_radius_m
        )
        target = self.mission.waypoints[self.index].copy()
        target[2] = self.mission.seabed_depth_m + altitude_m

        position = np.asarray(estimated_position, dtype=float)
        delta = target - position
        # Waypoint capture is judged on the ESTIMATE, which is the point: a
        # vehicle with a bad estimate will believe it has arrived when it has
        # not, and the evaluator will score that honestly.
        captured = float(np.linalg.norm(delta[:2])) < radius

        # A proximity test alone is not sufficient, and the failure it permits is
        # unbounded rather than merely inaccurate.
        #
        # Capture is judged in estimate space against a radius of half a metre.
        # A vehicle whose position error exceeds that radius can pass the
        # waypoint without its estimate ever entering the capture circle. The
        # index then never advances, line-of-sight guidance keeps steering along
        # a leg the vehicle has already finished, and it flies that heading until
        # the mission times out. Measured on one compound-failure run before this
        # guard existed: the vehicle completed two legs, then held x = 9.9 m and
        # flew from y = 3.6 m to y = 169.7 m over the remaining five minutes,
        # 160 m beyond a survey box 18 m across, while its own position estimate
        # stayed accurate to about a metre. It knew where it was throughout.
        #
        # The remedy is the standard one: advance when the vehicle has passed the
        # waypoint along the leg, measured by projection onto the leg direction,
        # as well as when it comes near it. Progress past the far end cannot be
        # undone by a poor estimate in the way proximity can, because it depends
        # on the component along a known direction rather than on a distance to a
        # point.
        #
        # This is part of the shared guidance law and therefore applies
        # identically to the proposed manager, every comparator and every
        # ablation.
        if not captured and self.index > 0:
            previous = self.mission.waypoints[self.index - 1]
            leg = target[:2] - previous[:2]
            leg_length = float(np.linalg.norm(leg))
            if leg_length > 1e-9:
                travelled = float(np.dot(position[:2] - previous[:2], leg) / leg_length)
                captured = travelled > leg_length

        if captured:
            self.index += 1
            return self.command(
                estimated_position, speed_mps, altitude_m, radius, current_estimate_mps
            )

        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            return np.zeros(3)

        # Horizontal and vertical are commanded separately, which is how AUV
        # guidance is normally built: heading follows the survey line, depth or
        # altitude is held by its own loop.
        #
        # They must not be combined into a single line-of-sight vector. Doing so
        # made the commanded altitude completely inert: the leg runs between two
        # waypoints at the same depth, so a direction along it has no vertical
        # component, and the vehicle could never climb or descend to the height
        # the manager asked for. Measured, the whole altitude axis of the action
        # space was dead -- configurations at 1.0 m, 2.0 m and 3.0 m produced
        # bit-identical outcomes on every scenario and every channel. Altitude is
        # the exponential lever on optical range, because the light makes a
        # two-way trip of about twice the altitude, so an inert altitude removes
        # the single strongest action the manager has.
        horizontal = self._line_of_sight_heading(estimated_position, target, delta)
        desired_ground = np.zeros(3)
        desired_ground[:2] = horizontal * speed_mps
        desired_ground[2] = float(
            np.clip(DEPTH_GAIN * delta[2], -MAX_VERTICAL_SPEED_MPS,
                    MAX_VERTICAL_SPEED_MPS)
        )
        return saturate_command(desired_ground, current_estimate_mps)

    def _line_of_sight_heading(
        self,
        estimated_position: np.ndarray,
        target: np.ndarray,
        delta: np.ndarray,
    ) -> np.ndarray:
        """Lookahead line-of-sight steering onto the current survey line.

        Returns a **horizontal** unit vector. Depth is not this law's business;
        the caller holds the commanded altitude with its own loop.

        Pointing the vehicle at the next waypoint is not the same as following
        the line to it. A vehicle that enters a leg displaced sideways and then
        aims at the endpoint flies the *chord* from where it is to where the
        waypoint is, so its distance from the intended line decays linearly along
        the whole leg instead of being driven to zero. The resulting error is set
        by the entry displacement, which is set by how far it cut the previous
        corner, which is set by the capture radius.

        That is what the metric was measuring. Varying only the capture radius
        over 1.50 and 0.75 m moved RMS cross-track error from 0.64 to 0.29 m
        while dead-reckoned and optically aided flight -- whose terminal position
        errors differed by a factor of nine -- stayed within 3% of each other.
        Excluding the turns did not help, because under pure pursuit the offset
        persists along the entire leg rather than only around the corner.

        Lookahead LOS instead steers toward a point on the line a fixed distance
        ahead of the vehicle's projection onto it, which drives cross-track error
        to zero and leaves the residual reflecting estimate error and imperfect
        current compensation -- the quantities the study is about. It is the
        standard path-following law for marine vehicles and is what a real survey
        controller does. Like current compensation it is part of the shared
        guidance and applies identically to every method.
        """
        waypoints = self.mission.waypoints
        fallback = delta[:2]
        norm = float(np.linalg.norm(fallback))
        fallback = fallback / norm if norm > 1e-9 else np.array([1.0, 0.0])
        if self.index <= 0:
            return fallback
        # Horizontal geometry only, throughout.
        start = waypoints[self.index - 1][:2]
        along = target[:2] - start
        length = float(np.linalg.norm(along))
        if length < 1e-6:
            return fallback
        unit = along / length

        offset = np.asarray(estimated_position, dtype=float)[:2] - start
        # Component of the offset perpendicular to the line. Depth is handled by
        # the caller's altitude loop, not by this law.
        cross = offset - (offset @ unit) * unit
        cross_distance = float(np.linalg.norm(cross))
        if cross_distance < 1e-9:
            return unit

        # Aim at a point LOOKAHEAD_M further along the line than the vehicle's
        # projection onto it. Small lookahead converges hard and oscillates;
        # large lookahead behaves like pure pursuit. The value is a few vehicle
        # lengths, the usual choice.
        aim = unit * LOOKAHEAD_M - cross
        return aim / float(np.linalg.norm(aim))


class Vehicle:
    """True kinematic state. Visible to the evaluator and the sensors only."""

    def __init__(
        self, position: Sequence[float], current_mps: Sequence[float] = (0.0, 0.0, 0.0)
    ) -> None:
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = np.zeros(3)
        self.current = np.asarray(current_mps, dtype=float)
        self._previous_velocity = np.zeros(3)
        self.path_length_m = 0.0

    def step(self, commanded_velocity: np.ndarray, dt: float) -> np.ndarray:
        """Advance and return the true acceleration over the interval.

        ``commanded_velocity`` is water-relative: it is what the thrusters
        produce against the surrounding fluid. Ground velocity is that plus the
        current, which is why the current appears here additively and why a
        vehicle that does not compensate for it tracks off the survey line.
        """
        self._previous_velocity = self.velocity.copy()
        # First-order thruster response toward the commanded water-relative
        # velocity, advected by a current the estimator must observe rather than
        # be told about.
        self.velocity = 0.7 * self.velocity + 0.3 * (
            np.asarray(commanded_velocity, dtype=float) + self.current
        )
        step = self.velocity * dt
        self.position = self.position + step
        self.path_length_m += float(np.linalg.norm(step))
        return (self.velocity - self._previous_velocity) / max(dt, 1e-9)


@dataclass
class MissionOutcome:
    """Mission-level metrics. These are the paper's primary quantities."""

    completed: bool = False
    waypoints_captured: int = 0
    waypoints_surveyed: int = 0
    waypoints_total: int = 0
    coverage_fraction: float = 0.0
    elapsed_s: float = 0.0
    rms_cross_track_m: float = 0.0
    max_cross_track_m: float = 0.0
    safety_violations: int = 0
    path_length_m: float = 0.0
    path_overhead: float = 0.0
    mean_altitude_m: float = 0.0
    #: Mean infrastructure cost of the acoustic techniques actually flown, in
    #: the cost model's units.
    #:
    #: A mission currency in exactly the sense altitude and elapsed time are.
    #: A technique that needs a surface vessel on station, a surveyed
    #: transponder array, or a prior bathymetric map is not free merely because
    #: it costs the vehicle no swath and no time -- somebody paid for it, and a
    #: deployment without it cannot use the technique at all. Recorded so that
    #: an improvement bought with dependency is visible as bought rather than
    #: as free.
    acoustic_infrastructure_cost: float = 0.0
    swath_coverage: float = 0.0
    terminal_error_m: float = 0.0
    aiding_availability: float = 0.0
    # Diagnostics -- never claim-bearing (rule N5).
    rms_position_error_m: float = 0.0
    p95_position_error_m: float = 0.0

    @property
    def failed(self) -> bool:
        return not self.completed


class MissionEvaluator:
    """Scores a run against ground truth. Scoring only -- never a decision input."""

    def __init__(self, mission: SurveyMission) -> None:
        self.mission = mission
        self._cross_track: list[float] = []
        self._errors: list[float] = []
        self._altitudes: list[float] = []
        self._aided: list[bool] = []
        self.safety_violations = 0
        self._captured = 0
        self._surveyed = [False] * len(mission.waypoints)
        self._geofence_breached = False

    @staticmethod
    def _distance_to_segment(
        point: np.ndarray, a: np.ndarray, b: np.ndarray
    ) -> float:
        ab = b - a
        denom = float(ab @ ab)
        if denom < 1e-12:
            return float(np.linalg.norm(point - a))
        s = float(np.clip((point - a) @ ab / denom, 0.0, 1.0))
        return float(np.linalg.norm(point - (a + s * ab)))

    def _cross_track_error(self, true_position: np.ndarray, leg: int) -> float:
        """Perpendicular distance from TRUE position to the segment being flown.

        ``leg`` is the guidance index: the vehicle has been commanded from
        ``waypoints[leg - 1]`` toward ``waypoints[leg]``, so that segment -- and
        only that segment -- is the path it is supposed to be on.

        Taking the minimum over *all* segments instead, as an earlier version
        did, silently rewards the failure this paper is about. A vehicle whose
        estimate has drifted a full line spacing sits exactly on the neighbouring
        leg and scores near-zero tracking error while surveying the wrong ground.
        """
        waypoints = self.mission.waypoints
        if leg <= 0:
            return float(np.linalg.norm(true_position[:2] - waypoints[0][:2]))
        leg = min(leg, len(waypoints) - 1)
        return self._distance_to_segment(
            true_position[:2], waypoints[leg - 1][:2], waypoints[leg][:2]
        )

    def _on_line(self, true_position: np.ndarray, leg: int) -> bool:
        """Whether this sample belongs in the line-keeping statistic.

        Samples taken while the vehicle is turning are excluded, because during a
        turn it is legitimately not on either line and its distance from the
        segment measures the turn geometry rather than how well it navigates.

        This is not a cosmetic exclusion; without it the metric is close to
        meaningless. Guidance switches to the next waypoint once the estimate is
        within ``capture_radius_m``, so the vehicle cuts every corner by very
        nearly that distance, and the corner-cutting term dominates the sum.
        Measured directly: holding everything else fixed and varying only the
        capture radius over 1.50, 0.75 and 0.30 m moved RMS cross-track error to
        0.766, 0.328 and 0.080 m, and maximum cross-track error to 1.47, 0.72 and
        0.26 m -- that is, the headline path-quality metric was reporting the
        capture radius, a configuration constant identical for every method,
        rather than any property of navigation. Dead reckoning and optically
        aided flight differed by under 2% of it while their terminal position
        errors differed threefold.

        Excluding the turn is standard survey line-keeping practice, applies
        identically to every method, and leaves the along-line behaviour that the
        metric is meant to describe.
        """
        waypoints = self.mission.waypoints
        if leg <= 0 or leg >= len(waypoints):
            return False
        exclusion = self.mission.turn_exclusion_m
        start = waypoints[leg - 1][:2]
        end = waypoints[leg][:2]
        here = true_position[:2]
        return bool(
            np.linalg.norm(here - start) > exclusion
            and np.linalg.norm(here - end) > exclusion
        )

    def record(
        self,
        true_position: np.ndarray,
        estimated_position: np.ndarray,
        aided: bool,
        waypoints_captured: int,
    ) -> None:
        if self._on_line(true_position, waypoints_captured):
            self._cross_track.append(
                self._cross_track_error(true_position, waypoints_captured)
            )
        self._errors.append(
            float(np.linalg.norm(true_position - estimated_position))
        )
        altitude = float(true_position[2] - self.mission.seabed_depth_m)
        self._altitudes.append(altitude)
        self._aided.append(bool(aided))
        self._captured = waypoints_captured

        # Truth-side coverage: a waypoint is surveyed when the vehicle really
        # passes close to it, whatever the estimate believed at the time.
        tolerance = self.mission.survey_tolerance_m
        # Coverage requires being close enough to the seabed to actually image
        # it, not merely being somewhere overhead.
        #
        # This previously tested horizontal distance alone. A vehicle at any
        # depth directly above a waypoint was credited with surveying it, which
        # made the metric gameable in the most direct way possible: once the
        # terminal self-preservation action was added, a vehicle that abandoned
        # the survey, ascended seventeen metres and drifted across the area on
        # GPS scored full coverage. It reported 1.000 coverage and zero mission
        # failures while never seeing the seabed again.
        #
        # The altitude bound is the optical reach the study already declares.
        # The survey channels are useful to roughly one to two attenuation
        # lengths, and every configuration in the action space flies between
        # 1 m and 3 m for that reason; beyond twice the nominal survey altitude
        # nothing in the sensor suite can resolve the bottom, so nothing there
        # can constitute coverage.
        maximum_survey_altitude = 2.0 * ALTITUDE_NOMINAL_M
        altitude = float(true_position[2] - self.mission.seabed_depth_m)
        for i, waypoint in enumerate(self.mission.waypoints):
            if self._surveyed[i]:
                continue
            if altitude > maximum_survey_altitude:
                continue
            if float(np.linalg.norm(true_position[:2] - waypoint[:2])) <= tolerance:
                self._surveyed[i] = True

        low, high = self.mission.altitude_band_m
        if altitude < low or altitude > high:
            self.safety_violations += 1
        if (
            abs(true_position[0]) > self.mission.geofence_half_extent_m
            or abs(true_position[1]) > self.mission.geofence_half_extent_m
        ):
            self.safety_violations += 1
            self._geofence_breached = True

    def finish(
        self, elapsed_s: float, path_length_m: float, completed: bool
    ) -> MissionOutcome:
        cross = np.asarray(self._cross_track or [0.0])
        errors = np.asarray(self._errors or [0.0])
        altitudes = np.asarray(self._altitudes or [0.0])
        ideal = self.mission.total_path_length_m()
        # Swath scales with altitude: flying low sees less ground per pass.
        nominal_altitude = 3.0
        surveyed = int(sum(self._surveyed))
        total = len(self.mission.waypoints)
        # A mission is completed only if the survey was actually flown: every
        # waypoint truly visited, inside the time limit, without leaving the
        # operating area. ``completed`` as passed in is what the *vehicle*
        # believed; it is necessary but nowhere near sufficient.
        really_completed = bool(
            completed
            and surveyed == total
            and elapsed_s < self.mission.time_limit_s
            and not self._geofence_breached
        )
        return MissionOutcome(
            completed=really_completed,
            waypoints_captured=self._captured,
            waypoints_surveyed=surveyed,
            waypoints_total=total,
            coverage_fraction=float(surveyed) / total if total else 0.0,
            elapsed_s=elapsed_s,
            rms_cross_track_m=float(np.sqrt(np.mean(cross**2))),
            max_cross_track_m=float(np.max(cross)),
            safety_violations=self.safety_violations,
            path_length_m=path_length_m,
            path_overhead=float(path_length_m / ideal - 1.0) if ideal > 0 else 0.0,
            mean_altitude_m=float(np.mean(altitudes)),
            swath_coverage=float(np.mean(altitudes) / nominal_altitude),
            terminal_error_m=float(errors[-1]),
            aiding_availability=float(np.mean(self._aided)) if self._aided else 0.0,
            rms_position_error_m=float(np.sqrt(np.mean(errors**2))),
            p95_position_error_m=float(np.percentile(errors, 95)),
        )
