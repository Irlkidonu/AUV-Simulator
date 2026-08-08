"""Navigational capability modes, inference, and transition stability.

Reference implementation of ``method/MODE_MANAGER_SPEC.md``
sections 1--2.

Modes are defined by **navigational capability and the decision it forces**, not
by water condition. Labelling modes ``clear``/``medium``/``turbid`` -- as the
invalidated earlier system did -- makes the mode a restatement of a commanded
environment variable rather than an inference.

Everything here consumes observables only. There is no path from this module to
ground truth, to the commanded fault schedule, or to the water state. That is
protocol rule N2, and ``Observables`` is deliberately the only input type.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional

__all__ = [
    "Mode",
    "CONSERVATISM",
    "Observables",
    "ModeThresholds",
    "ModeDecision",
    "infer_capability",
    "ModeStateMachine",
]


class Mode(Enum):
    """Navigational capability states."""

    NOMINAL = "M0_NOMINAL"
    OPTICAL_DEGRADED = "M1_OPTICAL_DEGRADED"
    OPTICAL_LOST = "M2_OPTICAL_LOST"
    VELOCITY_AIDING_LOST = "M3_VELOCITY_AIDING_LOST"
    DR_CRITICAL = "M4_DR_CRITICAL"
    RECOVERY = "M5_RECOVERY"


#: Ordering used for the monotonicity guarantee. Losing a capability may never
#: move the decision to a strictly *less* conservative mode.
CONSERVATISM: dict[Mode, int] = {
    Mode.NOMINAL: 0,
    Mode.OPTICAL_DEGRADED: 1,
    Mode.RECOVERY: 1,
    Mode.OPTICAL_LOST: 2,
    Mode.VELOCITY_AIDING_LOST: 2,
    Mode.DR_CRITICAL: 3,
}

#: Permitted transitions (spec section 2.3). Direct ``M4 -> M0`` is forbidden:
#: recovery from a critical state must pass through confirmed re-acquisition.
PERMITTED_TRANSITIONS: dict[Mode, frozenset[Mode]] = {
    Mode.NOMINAL: frozenset(
        {Mode.NOMINAL, Mode.OPTICAL_DEGRADED, Mode.VELOCITY_AIDING_LOST,
         Mode.DR_CRITICAL}
    ),
    Mode.OPTICAL_DEGRADED: frozenset(
        {Mode.OPTICAL_DEGRADED, Mode.NOMINAL, Mode.OPTICAL_LOST,
         Mode.DR_CRITICAL, Mode.RECOVERY}
    ),
    Mode.OPTICAL_LOST: frozenset(
        {Mode.OPTICAL_LOST, Mode.OPTICAL_DEGRADED, Mode.DR_CRITICAL,
         Mode.RECOVERY}
    ),
    Mode.VELOCITY_AIDING_LOST: frozenset(
        {Mode.VELOCITY_AIDING_LOST, Mode.NOMINAL, Mode.DR_CRITICAL,
         Mode.RECOVERY}
    ),
    Mode.DR_CRITICAL: frozenset({Mode.DR_CRITICAL, Mode.RECOVERY}),
    Mode.RECOVERY: frozenset(
        {Mode.RECOVERY, Mode.NOMINAL, Mode.DR_CRITICAL, Mode.OPTICAL_LOST,
         Mode.VELOCITY_AIDING_LOST}
    ),
}


@dataclass(frozen=True)
class Observables:
    """Everything the manager is permitted to see.

    This type is the information boundary. If a quantity is not a field here,
    the manager cannot reach it. Explicitly absent, and never to be added:
    ground-truth pose, commanded fault schedule, commanded turbidity, beam
    attenuation, optical depth, scenario identifiers.
    """

    #: Image-derived optical quality in [0, 1]. Computed from image content,
    #: never from a commanded turbidity value.
    optical_quality: float
    #: Whether the active optical configuration produced a fix this tick.
    optical_available: bool
    #: Whether the DVL currently reports bottom lock.
    dvl_bottom_lock: bool
    #: Seconds since the last accepted measurement of each aiding stream.
    dvl_age_s: float
    acoustic_fix_age_s: float
    imu_age_s: float
    depth_age_s: float
    #: Filter position-covariance trace (m^2) and its growth rate (m^2/s).
    position_covariance_trace: float
    covariance_growth_rate: float
    #: Windowed rate at which normalised innovations exceed their gate.
    innovation_exceedance_rate: float = 0.0
    #: Current commanded altitude above seabed (m). The vehicle knows this.
    altitude_m: float = 3.0
    #: Whether the DVL currently returns a water-track velocity.
    #:
    #: Defaults to ``False`` because this type is fail-closed: an observable that
    #: was not supplied must never make the vehicle look more capable than it is.
    dvl_water_track: bool = False
    #: Estimated speed of the ocean current (m/s), from the filter's current
    #: state. This is an *estimate*, not the commanded flow: it is what the
    #: vehicle has inferred from the difference between bottom-track and
    #: water-track velocity, and it is wrong by however much the filter is wrong.
    current_speed_mps: float = 0.0
    #: Trace of the current-state covariance (m^2/s^2). Grows while both DVL
    #: modes are unavailable, because nothing is then observing the flow. This is
    #: the quantity that distinguishes "the current is weak" from "the current
    #: was weak when I last had a way to measure it".
    current_covariance_trace: float = 0.0
    #: Change in image-derived optical quality across the identification window
    #: (dimensionless, positive when conditions are improving).
    #:
    #: Availability prediction was otherwise memoryless, which is wrong in
    #: exactly the situation that matters. A quality of 0.4 on a falling trend
    #: means a candidate configuration will shortly stop working; the same 0.4
    #: rising means it will shortly be fine. The manager has to commit to a
    #: configuration for the decision horizon, so where conditions are *heading*
    #: is as relevant as where they are.
    optical_quality_trend: float = 0.0
    #: Estimated terrain gradient under the vehicle (m/m), from the variability
    #: of its own altimeter over a recent window divided by the distance
    #: travelled in that window.
    #:
    #: This is what makes terrain-relative navigation *selectable* rather than
    #: merely available. A terrain match fixes position to sigma_depth divided
    #: by the terrain gradient, so a vehicle that can estimate the gradient can
    #: predict whether a match would succeed before committing to it -- the same
    #: counterfactual the optical channel answers with image quality.
    #:
    #: It is an inference from the altimeter the vehicle already carries, not a
    #: commanded terrain parameter, and it is wrong by however much a short
    #: window of altitude samples is unrepresentative of the map. Over a
    #: featureless plain it reads near zero, which is both the correct estimate
    #: and the correct reason to decline the technique.
    #:
    #: Defaults to zero, which is fail-closed: a vehicle that has not measured
    #: terrain relief must not assume it has any.
    terrain_gradient_estimate: float = 0.0
    #: Identified environment, from ``environment.EnvironmentClassifier``.
    #:
    #: These are *inferences from observables*, not commanded values, and that
    #: distinction is the whole point: an earlier system in this workspace named
    #: its modes after the water condition it had been told to simulate, which
    #: made every "detection" a restatement of an input. Here the labels come
    #: from image statistics, acoustic gate behaviour, and the filter's own
    #: current state, and they are wrong a measurable fraction of the time.
    #:
    #: Defaults describe a vehicle that has not identified anything yet, and are
    #: benign rather than optimistic: an unclassified environment carries zero
    #: confidence, so a manager that gates on confidence will ignore it.
    environment_turbidity: str = "unknown"
    environment_noise: str = "unknown"
    environment_current: str = "unknown"
    environment_confidence: float = 0.0

    #: The filter's own position estimate (m), not truth.
    #:
    #: The manager needs this to ask an acoustic counterfactual: *if I
    #: interrogated that transponder array from where I believe I am, would a
    #: fix come back?* Transponder positions are surveyed and therefore known,
    #: so the geometry is computable -- but only from a position, and the
    #: manager had no field carrying one.
    #:
    #: Without it the only acoustic input was ``acoustic_fix_age_s``, which
    #: reports whether the technique *currently in use* just delivered. That is
    #: present tense, not counterfactual: when a single beacon goes quiet it
    #: says nothing about whether USBL or LBL would answer, yet it was being
    #: read as though it did, which left the manager unable to tell the three
    #: techniques apart in 87% of compound-scenario decisions.
    #:
    #: Using the estimate rather than truth is the point. A vehicle that has
    #: drifted will mis-predict its own acoustic geometry, and that error is
    #: part of what the method must survive; feeding truth here would quietly
    #: turn the manager into an oracle.
    #: Defaults to a vehicle on survey at nominal altitude -- the seabed lies at
    #: -20 m and the nominal survey altitude is 3 m -- to match ``altitude_m``.
    #: The origin would be a poor default here rather than a neutral one: it is
    #: exactly where the USBL surface transponder sits, so it describes a
    #: vehicle at zero slant range with perfect acoustic geometry, which is the
    #: one position that makes every technique look ideal.
    estimated_position_m: tuple[float, float, float] = (0.0, 0.0, -17.0)

    def is_valid(self) -> bool:
        """Reject non-finite or out-of-domain inputs. Drives fail-closed."""
        numeric = (
            self.optical_quality,
            self.dvl_age_s,
            self.acoustic_fix_age_s,
            self.imu_age_s,
            self.depth_age_s,
            self.position_covariance_trace,
            self.covariance_growth_rate,
            self.innovation_exceedance_rate,
            self.altitude_m,
            self.current_speed_mps,
            self.current_covariance_trace,
            self.environment_confidence,
            self.optical_quality_trend,
        )
        if any(not math.isfinite(v) for v in numeric):
            return False
        if not 0.0 <= self.environment_confidence <= 1.0:
            return False
        if not 0.0 <= self.optical_quality <= 1.0:
            return False
        if self.position_covariance_trace < 0.0 or self.altitude_m <= 0.0:
            return False
        if self.current_speed_mps < 0.0 or self.current_covariance_trace < 0.0:
            return False
        return True


@dataclass(frozen=True)
class ModeThresholds:
    """Tunable decision thresholds.

    Every value is selected on development seeds by the equal-budget procedure
    and frozen before held-out execution. None is set by inspecting held-out
    outcomes.
    """

    quality_good: float = 0.55
    quality_marginal: float = 0.25
    dvl_max_age_s: float = 2.0
    acoustic_max_age_s: float = 30.0
    imu_max_age_s: float = 0.5
    #: Position uncertainty (m^2) above which navigation is treated as critical.
    covariance_critical_m2: float = 4.0
    #: Horizon over which growth is projected when deciding criticality.
    projection_horizon_s: float = 20.0
    #: Innovation exceedance rate above which aiding is treated as unreliable.
    innovation_exceedance_max: float = 0.30
    #: Stability machinery.
    minimum_dwell_s: float = 3.0
    debounce_s: float = 1.0
    #: Consecutive consistent measurements required to leave RECOVERY.
    reacquisition_confirmations: int = 2


@dataclass(frozen=True)
class ModeDecision:
    """A mode decision with the evidence that produced it."""

    mode: Mode
    candidate: Mode
    reason: str
    time_in_mode_s: float
    transitioned: bool
    projected_covariance_m2: float


def infer_capability(
    obs: Observables, thresholds: ModeThresholds = ModeThresholds()
) -> tuple[Mode, str]:
    """Map observables to a capability mode, with the deciding evidence.

    Fail-closed: invalid, stale, or non-finite inputs drive the decision toward
    the more conservative mode, never toward ``NOMINAL``.

    Monotone in capability loss: each branch below is ordered from most to least
    conservative, so acquiring a fault can only move the result downward in the
    ``CONSERVATISM`` ordering.
    """
    if not obs.is_valid():
        return Mode.DR_CRITICAL, "invalid_observables"

    # Dead reckoning itself is compromised: nothing else can be trusted.
    if obs.imu_age_s > thresholds.imu_max_age_s:
        return Mode.DR_CRITICAL, "inertial_stale"

    dvl_ok = obs.dvl_bottom_lock and obs.dvl_age_s <= thresholds.dvl_max_age_s
    # Any velocity information at all, whether referenced to the seabed or to the
    # water column. The distinction matters for capability: a vehicle that has
    # lost bottom lock but retains water track still measures its motion through
    # the water and still bounds its velocity error, so it is degraded rather
    # than dead-reckoning. A vehicle with neither has only the accelerometer, and
    # its velocity error grows without any measurement to check it.
    velocity_aiding = dvl_ok or obs.dvl_water_track
    acoustic_ok = obs.acoustic_fix_age_s <= thresholds.acoustic_max_age_s
    optical_ok = (
        obs.optical_available and obs.optical_quality >= thresholds.quality_good
    )
    optical_marginal = (
        obs.optical_available and obs.optical_quality >= thresholds.quality_marginal
    )
    aiding_unreliable = (
        obs.innovation_exceedance_rate > thresholds.innovation_exceedance_max
    )

    projected = (
        obs.position_covariance_trace
        + max(obs.covariance_growth_rate, 0.0) * thresholds.projection_horizon_s
    )

    # --- most conservative first -------------------------------------------
    # Compound loss: no velocity aiding of ANY kind AND no absolute aiding.
    #
    # The velocity term is total loss rather than bottom-lock loss. Treating
    # bottom-lock loss alone as compound would declare a critical state for a
    # vehicle that is still measuring its velocity through the water, and would
    # make the total-DVL-loss scenario indistinguishable from the bottom-lock
    # one -- collapsing a genuine capability difference that the vehicle can
    # observe and should act on.
    if not velocity_aiding and not (optical_marginal or acoustic_ok):
        return Mode.DR_CRITICAL, "compound_aiding_loss"

    # Uncertainty already past threshold, or projected to pass it.
    if obs.position_covariance_trace >= thresholds.covariance_critical_m2:
        return Mode.DR_CRITICAL, "covariance_above_critical"
    if projected >= thresholds.covariance_critical_m2:
        return Mode.DR_CRITICAL, "covariance_projected_critical"

    if not dvl_ok:
        return Mode.VELOCITY_AIDING_LOST, "dvl_bottom_lock_lost"

    if not obs.optical_available:
        return Mode.OPTICAL_LOST, "optical_unavailable"

    if aiding_unreliable:
        return Mode.OPTICAL_DEGRADED, "innovation_exceedance"

    if not optical_ok:
        return Mode.OPTICAL_DEGRADED, "optical_quality_low"

    return Mode.NOMINAL, "all_aiding_healthy"


class ModeStateMachine:
    """Applies hysteresis, minimum dwell, debounce, and confirmed re-acquisition.

    Chatter was a documented failure of the earlier system: an unnormalised
    quality signal with no hysteresis produced roughly 30 mode flips per second.
    The machinery here is asymmetric by design -- becoming *more* conservative is
    easy, becoming *less* conservative is deliberately hard.
    """

    def __init__(
        self,
        thresholds: ModeThresholds = ModeThresholds(),
        initial: Mode = Mode.NOMINAL,
    ) -> None:
        self.thresholds = thresholds
        self._mode = initial
        self._time_in_mode = 0.0
        self._pending: Optional[Mode] = None
        self._pending_for = 0.0
        self._confirmations = 0
        self.transitions = 0

    @property
    def mode(self) -> Mode:
        return self._mode

    @property
    def time_in_mode_s(self) -> float:
        return self._time_in_mode

    def update(self, obs: Observables, dt: float) -> ModeDecision:
        """Advance the machine by ``dt`` seconds and return the decision."""
        if dt < 0.0:
            raise ValueError("dt must be non-negative")

        candidate, reason = infer_capability(obs, self.thresholds)
        self._time_in_mode += dt

        projected = (
            obs.position_covariance_trace
            + max(obs.covariance_growth_rate, 0.0)
            * self.thresholds.projection_horizon_s
            if obs.is_valid()
            else float("inf")
        )

        target = self._resolve_target(candidate)
        transitioned = False

        if target is self._mode:
            self._pending = None
            self._pending_for = 0.0
        else:
            more_conservative = CONSERVATISM[target] > CONSERVATISM[self._mode]
            if self._pending is target:
                self._pending_for += dt
            else:
                self._pending = target
                self._pending_for = dt

            if more_conservative:
                # Escalation: debounce only. Never blocked by dwell -- refusing
                # to escalate because a timer has not elapsed would be unsafe.
                allowed = self._pending_for >= self.thresholds.debounce_s
            else:
                # De-escalation: dwell AND debounce AND, when leaving RECOVERY,
                # repeated confirmation that aiding really has returned.
                allowed = (
                    self._time_in_mode >= self.thresholds.minimum_dwell_s
                    and self._pending_for >= self.thresholds.debounce_s
                )
                if self._mode is Mode.RECOVERY:
                    self._confirmations += 1
                    allowed = allowed and (
                        self._confirmations
                        >= self.thresholds.reacquisition_confirmations
                    )

            if allowed:
                self._mode = target
                self._time_in_mode = 0.0
                self._pending = None
                self._pending_for = 0.0
                self._confirmations = 0
                self.transitions += 1
                transitioned = True

        if candidate is not self._mode and self._mode is not Mode.RECOVERY:
            self._confirmations = 0

        return ModeDecision(
            mode=self._mode,
            candidate=candidate,
            reason=reason,
            time_in_mode_s=self._time_in_mode,
            transitioned=transitioned,
            projected_covariance_m2=projected,
        )

    def _resolve_target(self, candidate: Mode) -> Mode:
        """Route the raw candidate through the permitted-transition graph.

        Returns the next *hop* on the shortest permitted path toward the
        candidate, not the candidate itself. The graph is deliberately sparse --
        ``M0 <-> M1 <-> M2`` has no ``M0 -> M2`` edge, and leaving ``M4`` is only
        possible via ``M5`` -- so a target is frequently several steps away.

        Stepping matters in both directions. Refusing to move because the target
        was not directly reachable would strand the vehicle in ``NOMINAL`` while
        its camera was dead; jumping straight to the target would bypass the
        confirmed re-acquisition that ``M5`` exists to enforce.
        """
        if candidate is self._mode:
            return candidate

        queue: deque[tuple[Mode, Optional[Mode]]] = deque([(self._mode, None)])
        seen = {self._mode}
        while queue:
            node, first_hop = queue.popleft()
            for nxt in PERMITTED_TRANSITIONS[node]:
                if nxt in seen:
                    continue
                hop = first_hop if first_hop is not None else nxt
                if nxt is candidate:
                    return hop
                seen.add(nxt)
                queue.append((nxt, hop))
        return self._mode
