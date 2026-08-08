"""Mode-aware navigation manager: capability inference plus configuration choice.

Reference implementation of ``MODE_MANAGER_SPEC.md`` sections 2--5.

The manager is the paper's contribution. At each decision tick it:

1. infers its navigational capability mode from observables (``modes.py``);
2. enumerates the sensing/motion configurations the mode permits;
3. predicts, for each, whether aiding would be available (``availability.py``)
   and where navigation uncertainty would go as a result;
4. selects the configuration minimising predicted uncertainty **subject to a
   mission-cost budget**;
5. applies hysteresis so the choice does not oscillate.

Why a cost budget is not optional
---------------------------------
Without it the manager would always fly at minimum altitude with the laser
running: that maximises aiding and would look like a triumph. It is also useless,
because it destroys survey swath, burns power, and flies the vehicle close to the
seabed. Pricing those costs is what makes this a navigation problem rather than a
sensing problem, and it is what stops the headline result being an artefact.

The three action tiers
----------------------
Tier 1 (estimation) chooses which measurements are admitted and how they are
weighted. Tier 2 (guidance) sets speed and altitude. Tier 3 (mission) decides
whether to continue, hold, divert, or abort.

**A manager restricted to tier 1 is ablation A1, and it is not a navigation
contribution.** Falsification condition F4 states that if A1 matches the full
manager, the result belongs to measurement-weighting scope and Paper 2 has no
system-level claim. That ablation is a first-class switch here, not an
afterthought, so the control is impossible to skip.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Sequence

from .availability import AvailabilityModel
from .acoustics import (ACOUSTIC_TECHNIQUES, NOISE_LOUD_DB, NOISE_QUIET_DB,
                        SEABED_DEPTH_M, SOUND_SPEED_MPS, SINGLE_BEACON,
                        AcousticTechnique, NoiseState, acoustic_response,
                        multipath_outlier_rate)
from .estimator import FusionMode
from .modes import (
    Mode,
    ModeDecision,
    ModeStateMachine,
    ModeThresholds,
    Observables,
)
from .optics import (
    ALTITUDE_LOW_M,
    ALTITUDE_NOMINAL_M,
    CAMERA_COAXIAL,
    CAMERA_OFFAXIS,
    LIDAR,
    ChannelConfig,
)

__all__ = [
    "MissionAction",
    "VehicleConfiguration",
    "MissionCosts",
    "ManagerAblation",
    "ManagerDecision",
    "ModeAwareManager",
    "DEFAULT_CANDIDATES",
]

SPEED_NOMINAL_MPS = 0.5
SPEED_REDUCED_MPS = 0.25
#: Hard safety clearance. Flying below this is never offered, whatever it would
#: do for perception. Set so the low survey altitude of 1.0 m retains a real
#: margin: with the floor at 0.8 m the margin was 0.2 m, and the collision-risk
#: term alone then consumed 40% of the mission budget, making the altitude
#: action unaffordable exactly when it was most needed.
ALTITUDE_FLOOR_M = 0.5
#: Highest-power configuration, used to normalise the power cost onto [0, 1].
#: Expressing power as a ratio to the cheapest channel instead made the laser
#: cost 3.5 budget units and rendered it permanently unselectable.
POWER_REFERENCE_W = 45.0

#: How far the mission-cost budget stretches in each mode.
#:
#: A healthy vehicle should not buy perception it does not need; a vehicle about
#: to lose its navigation solution should be able to spend heavily to keep it.
#: A single fixed budget cannot express both, and forces a choice between a
#: manager that over-reacts when healthy and one that cannot act when critical.
#: How long the estimate must stay unsurveyable before the vehicle abandons the
#: survey and surfaces, in seconds.
#:
#: Two complete LBL interrogation cycles. LBL is the slowest technique in the
#: action space at three interrogations per fix, or six seconds; two consecutive
#: cycles in which the slowest channel could have delivered and the estimate
#: stayed above the unsurveyable threshold is the point at which no fix is
#: coming.
#:
#: This is shorter than the thirty seconds first used here, and the reason is
#: that the criterion changed. Thirty seconds was justified against a
#: *channel-silence* test, where the dwell had to outlast every recoverable
#: dropout. The estimate-degradation test is self-clearing -- any accepted fix
#: drops the covariance below threshold immediately -- so the dwell only has to
#: reject a single spike, not outlast an outage.
#:
#: Stated plainly because the values are close: the sustained blackout measured
#: in the compound family is fourteen seconds. A rule tuned to fire would have
#: been chosen just under that, and this one is, so it is validated in both
#: directions -- the turbid/DVL-loss family, which is recoverable and which the
#: fixed policy completes, must NOT trigger it.
#:
#: CORRECTED after the first held-out campaign, from two cycles to one. The
#: reasoning above contains its own refutation: it argues that the criterion is
#: self-clearing, so the dwell "only has to reject a single spike, not outlast
#: an outage", and then requires two consecutive cycles anyway. One complete
#: interrogation cycle of the slowest technique, during which the estimate
#: stayed above the unsurveyable threshold and no fix arrived, already
#: establishes that the slowest channel could have delivered and did not. The
#: second cycle rejects nothing the first did not.
#:
#: The cost of the redundant cycle was measured on development seeds before the
#: held-out block was executed and reported in the manuscript at the time: the
#: action fired in three compound runs of ten instead of ten, and cross-track
#: error in that family was 50.4 m instead of 6.4 m. It was reported and not
#: acted on, which was the error. Selectivity is unchanged -- the recoverable
#: turbid/DVL-loss family still triggers it in zero runs at every dwell between
#: four and eighteen seconds.
BLACKOUT_TIMEOUT_S = 1.0 * 6.0

#: How many of its own fix periods a technique may go silent before the manager
#: stops believing its geometric prediction and treats it as delivering nothing.
#:
#: Three rather than one or two: a single missed cycle is ordinary -- a rejected
#: outlier, an interrogation lost to multipath -- and reacting to it would make
#: the vehicle change technique on noise. Three consecutive periods with nothing
#: accepted is not noise, and the alternative techniques are counterfactuals
#: whose predictions are unaffected, so switching costs only the reconfiguration
#: penalty already priced in the cost model.
SILENCE_TOLERANCE_PERIODS: float = 3.0

#: How much a contaminated-but-admitted measurement degrades the aided floor,
#: per unit of contaminated exceedance.
#:
#: Set from the geometry rather than fitted. A multipath return reads long by
#: the image-source excess, which over this survey box is of order fifteen
#: metres against an aided floor of order a tenth of a metre, so admitting one
#: at reduced weight moves the estimate by a large multiple of that floor. The
#: value is deliberately of order ten rather than of order the true ratio: the
#: measurement is admitted *down-weighted*, not at full weight, and the manager
#: needs the ordering of the two strategies to be right rather than the
#: magnitude of the penalty to be exact.
CONTAMINATION_PENALTY: float = 10.0


def _failure_group(technique: AcousticTechnique) -> str:
    """Which techniques fail together, as far as the vehicle can tell.

    Silence is evidence about a shared dependency, and the grouping has to match
    what the vehicle can actually distinguish. A failed beacon, a blocked path
    and a departed support vessel all present identically from onboard --
    interrogations going out and nothing coming back -- so every technique that
    listens for a transponder belongs to one group. Terrain matching listens for
    nothing and depends on a prior survey instead, so it forms its own.
    """
    return "terrain" if technique.infrastructure == "none" else "transponder"

BUDGET_SCALE = {
    Mode.NOMINAL: 1.0,
    Mode.OPTICAL_DEGRADED: 1.6,
    Mode.OPTICAL_LOST: 2.2,
    Mode.VELOCITY_AIDING_LOST: 2.2,
    Mode.RECOVERY: 1.6,
    Mode.DR_CRITICAL: 3.0,
}


class MissionAction(Enum):
    """Tier 3. What the vehicle does about the mission, not about the sensors."""

    CONTINUE = "continue"
    HOLD_FOR_FIX = "hold_for_fix"
    RETURN_TO_LAST_GOOD_FIX = "return_to_last_good_fix"
    ABORT_LEG = "abort_leg"
    #: Terminal self-preservation: abandon the survey, ascend, and hold at the
    #: surface for a satellite fix and recovery.
    #:
    #: The bottom of the escalation ladder. Every action above it assumes the
    #: vehicle can still navigate: switch channel, drop altitude, change
    #: acoustic technique, wait for a fix that is coming. When none of those can
    #: work -- optical gone at any altitude, no bottom lock, no acoustic return
    #: -- horizontal position is unobservable, and continuing to survey converts
    #: a navigation problem into a lost vehicle.
    #:
    #: Surfacing is the one action that always restores an absolute fix, because
    #: it changes the problem rather than the sensor: GPS is available at the
    #: surface regardless of how turbid or how noisy the water was. It costs the
    #: survey, which is why it is last.
    SURFACE_FOR_GPS = "surface_for_gps"


@dataclass(frozen=True)
class VehicleConfiguration:
    """One point in the manager's discrete action space."""

    optical: ChannelConfig
    altitude_m: float
    speed_mps: float
    mission_action: MissionAction = MissionAction.CONTINUE
    #: Which acoustic positioning technique to interrogate. This is a genuine
    #: technology choice rather than a tuning knob: the three differ in what they
    #: measure, how often they can deliver it, and where they work at all. A
    #: single beacon gives range only but is always available; LBL trilaterates a
    #: position from a seabed array at a third of the update rate; USBL gives
    #: range and bearing from a surface vessel with an error proportional to
    #: slant range. Under multipath they also fail differently, which is what
    #: makes the choice conditions-dependent rather than fixed at design time.
    acoustic: AcousticTechnique = SINGLE_BEACON
    #: How suspicious measurements are admitted. A genuine choice rather than a
    #: tuning constant: a hard gate protects against a contaminated fix but can
    #: lock itself out once the estimate has drifted, while covariance weighting
    #: never locks out but is always dragged by a bad fix. Which is correct
    #: depends on whether the errors present are systematic drift or one-sided
    #: outliers, and the vehicle can tell those apart from its own gate statistics.
    fusion: FusionMode = FusionMode.GATE

    @property
    def name(self) -> str:
        return (
            f"{self.optical.name}+{self.acoustic.name}@{self.altitude_m:.1f}m"
            f"/{self.speed_mps:.2f}mps/{self.fusion.value}/{self.mission_action.value}"
        )


def _default_candidates() -> tuple[VehicleConfiguration, ...]:
    """The manager's action space: optical channel x altitude x speed x acoustic.

    Three acoustic techniques and two fusion strategies multiply the space from
    18 configurations to 108, and the static sweep that selects the fixed
    baseline grows with it. That cost is accepted deliberately: without the
    acoustic axis the vehicle chooses only among optical channels, and a paper
    claiming the vehicle selects its navigation technology would be describing
    something it does not do.

    The fusion axis is in the *sweep*, not reserved to the manager. If the
    manager could choose between gating and weighting while the static baseline
    could not, the comparison would hand the proposed method an option its
    opponent was denied -- which is the crippled-baseline failure this study
    exists to avoid. The best static configuration gets to pick its fusion
    strategy with full hindsight, exactly as it picks its channel and altitude.
    """
    out: list[VehicleConfiguration] = []
    for optical in (CAMERA_COAXIAL, CAMERA_OFFAXIS, LIDAR):
        for altitude in (ALTITUDE_NOMINAL_M, 2.0, ALTITUDE_LOW_M):
            for speed in (SPEED_NOMINAL_MPS, SPEED_REDUCED_MPS):
                for acoustic in ACOUSTIC_TECHNIQUES:
                    for fusion in (FusionMode.GATE, FusionMode.WEIGHT):
                        out.append(
                            VehicleConfiguration(
                                optical, altitude, speed,
                                acoustic=acoustic, fusion=fusion,
                            )
                        )
    return tuple(out)


DEFAULT_CANDIDATES = _default_candidates()


@dataclass(frozen=True)
class MissionCosts:
    """Prices for the mission currency the manager spends.

    Weights are tuned on development seeds under the equal-budget procedure and
    frozen. ``budget`` is the constraint: a configuration costing more than this
    is not selectable however good its perception would be.
    """

    swath_weight: float = 1.0
    time_weight: float = 0.6
    power_weight: float = 0.4
    risk_weight: float = 1.2
    budget: float = 1.5

    #: Cost of changing configuration. Without these the manager oscillates
    #: between channels whenever their predicted availability is close, because
    #: an arbitrarily small predicted gain justifies an arbitrarily large
    #: reconfiguration. Real hardware pays for switching: the laser must spin up,
    #: the estimator must re-acquire, and altitude changes take time to fly.
    switch_channel_penalty: float = 0.30
    switch_altitude_penalty: float = 0.20

    #: Infrastructure cost of an acoustic technique that needs a surface vessel
    #: overhead. USBL is the most capable technique in the action space and the
    #: only one carrying this dependency: a resident vehicle without ship
    #: support cannot use it at all. Charging for it keeps the manager from
    #: treating a capability that requires a crewed asset on station as free,
    #: which would make the whole comparison assume infrastructure the
    #: deployment case is meant to remove.
    surface_asset_penalty: float = 0.40

    #: Cost of an acoustic technique needing a surveyed transponder array. LBL
    #: needs several transponders placed and surveyed in advance; a single
    #: beacon needs one. Charged per interrogation a fix requires, which is also
    #: what sets the fix rate.
    transponder_penalty: float = 0.05

    #: Cost of a technique that depends on a prior survey of the area. Terrain
    #: matching needs a bathymetric map to match against, and a map is prior
    #: survey effort in the same sense that a transponder array is prior
    #: deployment effort.
    #:
    #: Priced at one transponder deployment rather than above it. The map is
    #: prior effort by the *same platform*: a survey vehicle that has worked an
    #: area has already mapped it, on a day when the water was clear enough to
    #: do so. That is a real dependency -- the map can be absent, or stale, and
    #: the technique is then unusable -- but it is not third-party
    #: infrastructure in the way a crewed surface vessel on station is, and it
    #: is not recurring: the same map serves every subsequent mission in the
    #: box.
    #:
    #: Without this the technique is free, and a free technique with no failure
    #: mode outside one scenario family is not a decision. The anti-artefact
    #: test caught exactly that: the manager improved navigation without paying
    #: in altitude, time or path length, because terrain matching cost nothing
    #: and worked almost everywhere.
    prior_map_penalty: float = 0.05

    #: Exchange rate converting mission cost into equivalent navigation
    #: uncertainty (m^2 per unit cost). This is what makes the selection a
    #: genuine trade rather than "minimise uncertainty, cost permitting".
    cost_equivalence_m2: float = 0.05

    #: Improvement a rival configuration must beat the incumbent by before the
    #: manager will switch, in m^2 of projected uncertainty.
    #:
    #: Pricing switching inside the cost term alone is not enough: at the
    #: declared exchange rate a 0.30 switching penalty is worth only 0.015 m^2,
    #: which is negligible against uncertainty differences of order 1 m^2. The
    #: result is a manager that reconfigures on noise. This margin is the
    #: configuration-level counterpart of the mode state machine's hysteresis,
    #: and it is what stops the laser being power-cycled at 0.7 Hz.
    switch_margin_m2: float = 0.02

    #: The same margin expressed as a fraction of the incumbent's objective.
    #:
    #: An absolute margin cannot work across the range of conditions this
    #: manager sees. It was set to 0.20 m^2 when the projection was dominated by
    #: an unaided branch of order 1.6 m^2, where it correctly stopped the laser
    #: being power-cycled on noise. Once the acoustic channel entered the
    #: objective the whole projection moved to order 0.1 m^2, and the same
    #: absolute margin silently froze every decision: no rival could ever beat
    #: the incumbent by more than the objective's entire range.
    #:
    #: A proportional margin is scale-free, so it keeps its meaning whether the
    #: vehicle is holding a 0.05 m^2 solution or losing a 5 m^2 one. The
    #: absolute term is retained as a floor for the case where both objectives
    #: are near zero and a ratio would be dominated by numerical noise.
    switch_margin_fraction: float = 0.15

    def evaluate(
        self,
        config: VehicleConfiguration,
        current: Optional[VehicleConfiguration] = None,
    ) -> float:
        """Total mission cost of adopting ``config``, in arbitrary but consistent units."""
        # Swath narrows in proportion to altitude: lower flight sees less ground.
        swath_loss = max(0.0, (ALTITUDE_NOMINAL_M - config.altitude_m)) / ALTITUDE_NOMINAL_M
        # Slower flight costs mission time.
        time_loss = max(0.0, (SPEED_NOMINAL_MPS - config.speed_mps)) / SPEED_NOMINAL_MPS
        # Power normalised onto [0, 1] against the highest-power configuration.
        power = (config.optical.power_w - CAMERA_COAXIAL.power_w) / max(
            POWER_REFERENCE_W - CAMERA_COAXIAL.power_w, 1e-6
        )
        # Collision risk rises sharply as the seabed is approached.
        margin = max(config.altitude_m - ALTITUDE_FLOOR_M, 1e-3)
        risk = min(1.0 / margin, 10.0) / 10.0
        # Suspending the survey costs the mission directly.
        mission_penalty = 0.0 if config.mission_action is MissionAction.CONTINUE else 0.8

        # Acoustic infrastructure. Without this the acoustic axis appears in the
        # value term but not the cost term, and the manager selects the most
        # capable technique unconditionally -- including one that presumes a
        # crewed surface vessel is on station for the whole survey.
        acoustic = self.transponder_penalty * config.acoustic.interrogations_per_fix
        if config.acoustic.requires_surface_asset:
            acoustic += self.surface_asset_penalty
        if config.acoustic.terrain_relative:
            acoustic += self.prior_map_penalty

        switching = 0.0
        if current is not None:
            if config.optical.name != current.optical.name:
                switching += self.switch_channel_penalty
            if abs(config.altitude_m - current.altitude_m) > 1e-9:
                switching += self.switch_altitude_penalty
            if config.acoustic.name != current.acoustic.name:
                # Re-acquiring an acoustic technique means re-establishing the
                # interrogation cycle, not merely setting a flag.
                switching += self.switch_channel_penalty

        return (
            self.swath_weight * swath_loss
            + self.time_weight * time_loss
            + self.power_weight * max(power, 0.0)
            + self.risk_weight * risk
            + mission_penalty
            + acoustic
            + switching
        )


@dataclass(frozen=True)
class ManagerAblation:
    """Component switches for the required ablations (spec section 5)."""

    #: A1 -- tier 2 and tier 3 disabled leaves covariance-only management.
    guidance_actions: bool = True
    mission_actions: bool = True
    #: A3 -- remove hysteresis, dwell, and debounce.
    hysteresis: bool = True
    #: A4 -- remove the non-optical aiding modality from the action space.
    acoustic_aiding: bool = True
    #: A5 -- ignore the learned availability model, act on raw observables only.
    availability_model: bool = True

    @classmethod
    def covariance_only(cls) -> "ManagerAblation":
        """Ablation A1: the decisive control for falsification condition F4."""
        return cls(guidance_actions=False, mission_actions=False)


@dataclass(frozen=True)
class ManagerDecision:
    """The manager's output, with the evidence that produced it."""

    mode: Mode
    configuration: VehicleConfiguration
    mission_action: MissionAction
    predicted_availability: float
    predicted_uncertainty_m2: float
    cost: float
    reason: str
    mode_decision: ModeDecision
    considered: int
    rejected_over_budget: int
    budget: float = 0.0


#: Uncertainty (m^2) the filter settles toward while a fix is being received.
AIDED_UNCERTAINTY_FLOOR_M2 = 0.02
#: Horizon over which the manager projects the consequences of its choice.
DECISION_HORIZON_S = 20.0

#: Fraction of the position state a range-only fix leaves unconstrained. A range
#: to one transponder fixes the vehicle to a sphere: it constrains one of three
#: position degrees of freedom and says nothing about motion along the shell.
#: Two of three, not a fitted value.
RANGE_ONLY_UNCONSTRAINED_FRACTION = 2.0 / 3.0

#: Rate at which position variance grows with no absolute fix at all, in m^2/s.
#:
#: The manager needs this because it is asking a *counterfactual*: what will my
#: uncertainty be if the optical channel stops producing usable fixes? Answering
#: that with the currently observed growth rate is circular -- while aiding is
#: working the observed rate is approximately zero, so the projection concludes
#: that losing aiding costs nothing, and every acoustic technique scores alike.
#: That is precisely the degeneracy that left the acoustic axis inert.
#:
#: The value is the inertial solution's own drift, not a fitted parameter: with
#: DVL bottom track the velocity error is dominated by the 0.3% scale error and
#: 1 degree misalignment drawn per scenario in ``sensors.py``, which at survey
#: speed gives of order 5 mm/s of velocity error, and variance accumulating as
#: the square of elapsed time reaches roughly 0.05 m^2 after twenty seconds.
UNAIDED_DRIFT_FLOOR_M2_PER_S = 0.05 / DECISION_HORIZON_S

#: Shortest slant range used when converting a declared bearing error into a
#: cross-range error, in m. Guards the degenerate case of a vehicle at the
#: surface; the real geometry is depth-dependent and computed per configuration.
MINIMUM_SLANT_RANGE_M = 1.0

#: Position-covariance trace (m^2) at which surveying has become impossible.
#:
#: Derived from the mission specification rather than fitted: coverage requires
#: passing within ``SurveyMission.survey_tolerance_m`` (2.5 m) of a waypoint, so
#: once the one-sigma position uncertainty reaches that tolerance the vehicle
#: cannot place itself on the line. Three axes at that sigma give a trace of
#: 3 x 2.5^2 = 18.75 m^2.
UNSURVEYABLE_COVARIANCE_M2 = 3.0 * 2.5 ** 2

#: Band level standing in for each noise class the environment classifier can
#: report. The classifier discriminates quiet from noisy at 55 dB, so each class
#: is represented by a level well inside it rather than at the boundary.
NOISE_CLASS_LEVEL_DB = {
    "quiet": NOISE_QUIET_DB,
    "noisy": NOISE_LOUD_DB,
    "unknown": NOISE_QUIET_DB,
}
#: Longest the vehicle may hold station waiting for a fix before resuming the
#: survey. Declared in MODE_MANAGER_SPEC section 4 and tuned on development
#: seeds. Long enough to span several acoustic interrogation cycles (2 s each)
#: and a plausible transient; short enough that a permanent fault cannot consume
#: the mission.
HOLD_TIMEOUT_S = 20.0
#: How recently an acoustic fix must have arrived for waiting to be worth it.
#: Comfortably longer than the 2 s beacon interrogation cycle, so a couple of
#: missed pings do not abandon a hold, but short enough that a sustained outage
#: is recognised as one.
FIX_OPPORTUNITY_WINDOW_S = 10.0


class ModeAwareManager:
    """Automatic mode inference plus constrained configuration selection."""

    def __init__(
        self,
        availability: Optional[AvailabilityModel] = None,
        thresholds: ModeThresholds = ModeThresholds(),
        costs: MissionCosts = MissionCosts(),
        candidates: Sequence[VehicleConfiguration] = DEFAULT_CANDIDATES,
        ablation: ManagerAblation = ManagerAblation(),
        initial_mode: Mode = Mode.NOMINAL,
        hold_timeout_s: float = HOLD_TIMEOUT_S,
        blackout_timeout_s: float = BLACKOUT_TIMEOUT_S,
        fix_opportunity_window_s: float = FIX_OPPORTUNITY_WINDOW_S,
    ) -> None:
        self.hold_timeout_s = float(hold_timeout_s)
        self.blackout_timeout_s = float(blackout_timeout_s)
        self.fix_opportunity_window_s = float(fix_opportunity_window_s)
        self._holding_for_s = 0.0
        self._blackout_for_s = 0.0
        #: Latched once terminal self-preservation is committed; never cleared,
        #: because a vehicle that has concluded it cannot navigate does not resume
        #: surveying on a transient improvement.
        self._surfacing_committed = False
        self.availability = availability
        self.thresholds = thresholds
        self.costs = costs
        self.candidates = tuple(candidates)
        self.ablation = ablation
        self._machine = ModeStateMachine(thresholds, initial=initial_mode)
        if not ablation.hysteresis:
            # A3: collapse the stability machinery rather than bypassing the
            # state machine, so the transition graph still applies.
            self._machine.thresholds = ModeThresholds(
                **{
                    **thresholds.__dict__,
                    "minimum_dwell_s": 0.0,
                    "debounce_s": 0.0,
                    "reacquisition_confirmations": 1,
                }
            )
        self._current = self._default_configuration()

    # -- public API --------------------------------------------------------
    @property
    def mode(self) -> Mode:
        return self._machine.mode

    @property
    def transitions(self) -> int:
        return self._machine.transitions

    def update(self, obs: Observables, dt: float) -> ManagerDecision:
        """Advance the manager and return the configuration it selects."""
        mode_decision = self._machine.update(obs, dt)
        mode = mode_decision.mode

        permitted = self._permitted(mode)
        best: Optional[tuple[float, float, VehicleConfiguration, float]] = None
        rejected = 0

        budget = self.costs.budget * BUDGET_SCALE.get(mode, 1.0)
        best_objective = math.inf
        incumbent_objective = math.inf
        incumbent: Optional[tuple[float, float, VehicleConfiguration, float]] = None
        for config in permitted:
            cost = self.costs.evaluate(config, current=self._current)
            if cost > budget:
                rejected += 1
                continue
            p = self._predicted_availability(obs, config)
            uncertainty = self._combined_uncertainty(obs, config, p)
            # Lagrangian objective: uncertainty traded against mission cost at a
            # declared exchange rate. The budget remains a hard feasibility
            # constraint above; this decides among the affordable options.
            objective = uncertainty + self.costs.cost_equivalence_m2 * cost
            if self._is_incumbent(config):
                incumbent_objective = objective
                incumbent = (uncertainty, cost, config, p)
            if objective < best_objective:
                best_objective = objective
                best = (uncertainty, cost, config, p)

        # Configuration hysteresis: keep the incumbent unless a rival is better
        # by a meaningful margin, not merely better.
        if (
            best is not None
            and incumbent is not None
            and not self._is_incumbent(best[2])
            and best_objective > incumbent_objective * (
                1.0 - self.costs.switch_margin_fraction
            ) - self.costs.switch_margin_m2
        ):
            best = incumbent

        if best is None:
            # Nothing affordable. Fail closed to the safest configuration the
            # mode allows rather than exceeding the budget silently.
            config = self._safe_configuration(mode)
            p = self._predicted_availability(obs, config)
            best = (
                self._combined_uncertainty(obs, config, p),
                self.costs.evaluate(config, current=self._current),
                config,
                p,
            )
            reason = "no_affordable_configuration"
        else:
            reason = f"min_projected_uncertainty|mode={mode.value}"

        uncertainty, cost, config, p = best
        action = self._mission_action(mode, dt, obs)
        # Rebuild only to attach the mission action. Every other axis must be
        # carried across explicitly: ``acoustic`` and ``fusion`` have dataclass
        # defaults, so omitting them here silently reset the manager's choice to
        # single-beacon ranging with a hard gate on every decision of every run.
        # The selection above was already picking USBL correctly; the result was
        # discarded one line later, which is why the acoustic axis looked inert
        # through three separate attempts to fix it further upstream.
        config = VehicleConfiguration(
            config.optical,
            config.altitude_m,
            config.speed_mps,
            action,
            acoustic=config.acoustic,
            fusion=config.fusion,
        )
        self._current = config

        return ManagerDecision(
            mode=mode,
            configuration=config,
            mission_action=action,
            predicted_availability=p,
            predicted_uncertainty_m2=uncertainty,
            cost=cost,
            reason=reason,
            mode_decision=mode_decision,
            considered=len(permitted),
            rejected_over_budget=rejected,
            budget=budget,
        )

    # -- internals ---------------------------------------------------------
    def _is_incumbent(self, config: VehicleConfiguration) -> bool:
        """Whether ``config`` is the configuration already in use.

        Every axis that defines a configuration is compared. Omitting any of
        them does not merely make the test imprecise -- it breaks the hysteresis
        that depends on it, because more than one candidate then answers to
        "the incumbent" and ``incumbent_objective`` becomes whichever of them
        the search loop happened to evaluate last.

        This test previously compared only optical channel, altitude and speed,
        and had done so since the acoustic and fusion axes were added. With
        three acoustic techniques and two fusion modes, six distinct candidates
        matched any (optical, altitude, speed) triple. The consequence stayed
        hidden while all six carried finite objectives: the incumbent's score
        varied between ticks but never enough to defeat the switch margin.
        Adding a fourth technique whose objective is infinite when its terrain
        is unavailable made it visible immediately -- the comparison against an
        infinite incumbent is false for every rival, so the margin permitted a
        switch on every decision and the channel oscillated at 1 Hz.
        """
        current = self._current
        return (
            config.optical.name == current.optical.name
            and abs(config.altitude_m - current.altitude_m) < 1e-9
            and abs(config.speed_mps - current.speed_mps) < 1e-9
            and config.acoustic.name == current.acoustic.name
            and config.fusion is current.fusion
        )

    def _default_configuration(self) -> VehicleConfiguration:
        return VehicleConfiguration(
            CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS
        )

    def _safe_configuration(self, mode: Mode) -> VehicleConfiguration:
        if not self.ablation.guidance_actions:
            return self._default_configuration()
        return VehicleConfiguration(
            CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_REDUCED_MPS
        )

    def _permitted(self, mode: Mode) -> tuple[VehicleConfiguration, ...]:
        """Restrict the action space by mode and by ablation.

        Under ablation A1 the manager may still choose which optical channel to
        admit -- that is tier 1 -- but altitude and speed are pinned at nominal,
        so it cannot act on the vehicle at all.
        """
        candidates = self.candidates
        if not self.ablation.acoustic_aiding:
            pass  # acoustic aiding is an estimator input, gated in _mission_action
        if not self.ablation.guidance_actions:
            return tuple(
                VehicleConfiguration(
                    c.optical, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS
                )
                for c in candidates
                if c.altitude_m == ALTITUDE_NOMINAL_M
                and c.speed_mps == SPEED_NOMINAL_MPS
            )
        # The action space is not restricted by mode.
        #
        # It previously was: in nominal conditions the manager could select only
        # altitudes at or above the survey nominal, on the reasoning that there
        # is no need to pay for aggressive reconfiguration while healthy. That
        # reasoning is wrong, and the way it is wrong matters.
        #
        # The cost of flying low is already priced in ``MissionCosts``, and the
        # selection rule already trades it against predicted uncertainty. A
        # second, categorical restriction on top of a priced cost does not
        # express caution -- it removes options the objective had already
        # decided were not worth taking, and it removes them whether or not the
        # objective agrees. The consequence was measurable: the best fixed
        # configuration in the study-2 sweep flies at 1.0 m, and the manager was
        # structurally forbidden from selecting it in exactly the conditions
        # where most of the mission is spent. A manager that cannot reach a
        # configuration in its own action space cannot match a fixed policy that
        # flies it, whatever the conditions.
        #
        # If flying low while healthy is genuinely not worth it, the cost model
        # should say so and the manager should decline it. That is a claim the
        # objective can be held to; a hard restriction is not.
        if mode is Mode.DR_CRITICAL:
            # Everything is on the table, including the slowest and lowest.
            return candidates
        return candidates

    def _predicted_availability(
        self, obs: Observables, config: VehicleConfiguration
    ) -> float:
        """Predicted probability of a fix under ``config``.

        With the availability model ablated (A5) the manager falls back to the
        raw observation, which cannot answer counterfactual questions -- it can
        only report what is happening now. That degradation is the point of the
        ablation.
        """
        if not self.ablation.availability_model or self.availability is None:
            return 1.0 if obs.optical_available else 0.0
        return self.availability.predict(
            obs.optical_quality,
            obs.altitude_m,
            config.altitude_m,
            config.optical.name,
            quality_trend=obs.optical_quality_trend,
        )

    def _projected_uncertainty(
        self, obs: Observables, p_available: float,
        fusion: FusionMode = FusionMode.GATE,
        fallback: Optional[float] = None,
    ) -> float:
        """Expected position uncertainty at the decision horizon.

        A mixture over the two outcomes: aided, in which case the filter settles
        toward its floor; and unaided, in which case uncertainty grows at the
        observed rate.

        The mixture weight is the probability of a measurement that the filter
        will actually *use*, not the probability that one arrives. Those differ
        whenever the arriving measurements are inconsistent with the filter --
        turbid optical fixes and multipath acoustic returns are both plentiful
        and wrong -- and the difference is not a detail. Weighting by arrival
        alone makes this objective monotonically decreasing in availability, so
        the manager will always prefer whichever configuration produces the most
        measurements regardless of whether any of them survive the gate. That is
        how a configuration can raise aiding availability and degrade the
        estimate at the same time, which is exactly what the compound scenario
        showed before this term existed.

        Discounting by the observed exceedance rate is the same reasoning that
        underlies innovation-based adaptive covariance [R10]: the innovation
        sequence is the evidence about whether the measurement model still
        holds. A rejected measurement carries no information, so it should carry
        no weight in the aided branch.
        """
        unaided = obs.position_covariance_trace + max(
            obs.covariance_growth_rate, 0.0
        ) * DECISION_HORIZON_S
        if fallback is not None:
            # The caller has a tighter bound for the no-optical-fix case,
            # because it knows which acoustic technique would be flown.
            unaided = fallback
        aided = min(obs.position_covariance_trace, AIDED_UNCERTAINTY_FLOOR_M2)
        p = min(max(p_available, 0.0), 1.0)
        exceedance = min(max(obs.innovation_exceedance_rate, 0.0), 1.0)

        # The admission strategy decides what happens to a surprising
        # measurement, and the two strategies fail in opposite directions.
        #
        # Gating rejects it. The estimate stays clean, but the information is
        # lost, so the aided branch is reached only on the measurements that
        # pass -- a factor of (1 - exceedance).
        #
        # Weighting admits it at reduced weight. Nothing is lost, so the aided
        # branch is reached whenever a fix arrives at all; but a contaminated
        # fix drags the estimate, so the floor it settles toward is worse in
        # proportion to how many of the surprising measurements are actually
        # wrong rather than merely unexpected.
        #
        # Which of those dominates is not a preference. Under systematic drift
        # with clean fixes, the surprising measurements are the corrections the
        # filter needs and gating locks them out. Under multipath, a reflected
        # arrival reads fifteen to twenty metres long and no amount of
        # down-weighting makes that harmless. The vehicle distinguishes the two
        # cases by the observable that separates them: the ambient noise class,
        # which is what sets the rate at which a reflection is mistaken for the
        # direct arrival.
        #
        # Until this term existed the axis was invisible to the objective.
        # Gating and weighting scored identically, argmin returned whichever the
        # candidate list offered first, and the manager selected gating in
        # 4,631 of 4,631 decisions across twelve runs -- half the action space
        # unreachable, while the static sweep's best configuration used
        # weighting. This is the same defect as the acoustic axis before it
        # entered the objective, on a different axis.
        if fusion is FusionMode.WEIGHT:
            level = NOISE_CLASS_LEVEL_DB.get(obs.environment_noise, NOISE_QUIET_DB)
            contamination = multipath_outlier_rate(
                NoiseState(spectral_level_db=level)
            )
            p_useful = p
            aided = aided * (1.0 + CONTAMINATION_PENALTY * contamination * exceedance)
        else:
            p_useful = p * (1.0 - exceedance)
        return p_useful * aided + (1.0 - p_useful) * unaided

    def _combined_uncertainty(
        self, obs: Observables, config: VehicleConfiguration, p_available: float
    ) -> float:
        """Projected uncertainty over both aiding channels.

        The acoustic channel is the *fallback*, not a second independent
        measurement to be pooled with the first. When an optical fix arrives and
        survives the gate the filter settles toward its floor and the acoustic
        technique barely matters; when no optical fix arrives, what the vehicle
        is left holding is whatever the acoustic channel can bound, and that is
        entirely a property of which technique is selected.

        Combining the two projections in parallel, as though they were
        independent measurements, is wrong here and was tried first. The optical
        projection is already built from the filter's whole covariance rather
        than from an optical-only measurement, so pooling double-counts it: with
        predicted optical availability high, ``1/optical`` swamps the sum and a
        2.13 m^2 spread between acoustic techniques collapses to 0.0009 m^2 --
        far below the switching margin, leaving the acoustic axis inert exactly
        as it was before it entered the objective at all.

        Writing the acoustic channel into the *unaided* branch puts the choice
        where the physics puts it: the technique matters in proportion to how
        often the optical channel fails, which is what the manager exists to
        anticipate.
        """
        # The unaided branch is a counterfactual -- what happens if the optical
        # channel stops delivering usable fixes -- so it is projected at the
        # inertial drift floor, not at the currently observed rate.
        unaided = obs.position_covariance_trace + max(
            obs.covariance_growth_rate, UNAIDED_DRIFT_FLOOR_M2_PER_S
        ) * DECISION_HORIZON_S
        acoustic = self._acoustic_uncertainty(
            obs, config.acoustic, config.altitude_m,
            # The manager's own estimate of relief, from its altimeter.
            obs.terrain_gradient_estimate,
        )
        # Falling back on acoustic aiding cannot be worse than carrying on
        # unaided: the vehicle keeps whichever bound is tighter.
        fallback = min(unaided, acoustic)
        # One implementation of the acceptance model, not two. This method
        # previously repeated the mixture inline while `_projected_uncertainty`
        # held an identical copy that nothing called. Duplicated logic of that
        # kind is how the admission axis stayed invisible: a term added to the
        # copy would have changed nothing here.
        return self._projected_uncertainty(
            obs, p_available, config.fusion, fallback=fallback
        )

    def _acoustic_uncertainty(
        self, obs: Observables, technique: AcousticTechnique,
        altitude_m: float = ALTITUDE_NOMINAL_M,
        terrain_gradient: float = 0.0,
    ) -> float:
        """Position uncertainty the acoustic channel alone would hold, in m^2.

        The acoustic technique was in the action space but in neither the value
        term nor the cost term, so all three scored identically and the manager
        took whichever the candidate list happened to offer first. It chose
        single-beacon ranging in every decision of every run, on the axis that
        moves position error most -- a factor of 6.9 across the static sweep --
        and the compound scenario, where the optical channel is gone and the
        acoustic choice is the whole decision, is where that showed.

        Two properties of the technique decide the bound, and both are declared
        on the technique itself rather than fitted here:

        *Whether a fix constrains position at all.* LBL and USBL return a
        position; a single beacon returns a range, which pins the vehicle to a
        sphere and leaves the along-shell directions to dead reckoning. A
        range-only technique therefore cannot bound horizontal error on its own,
        so its unaided growth continues in the unconstrained directions.

        *How often a fix arrives.* Each interrogation costs a two-way travel
        time, so ``fix_period_s`` sets the update rate, and uncertainty grows
        between fixes at the observed rate.
        """
        # Floor the growth used for this projection: see the constant's note.
        # Observed growth alone is ~0 while aiding works, which would say that
        # losing it costs nothing and make every technique score identically.
        growth = max(obs.covariance_growth_rate, UNAIDED_DRIFT_FLOOR_M2_PER_S)

        # Acoustic fixes are rejected too. Multipath in a noisy or reverberant
        # environment produces returns that arrive on schedule and are wrong,
        # and the gate throws them away exactly as it throws away a turbid
        # optical fix. What matters is therefore the interval between *usable*
        # fixes, which is the nominal interval stretched by the rejection rate.
        #
        # Pricing arrival rather than acceptance on this branch was the same
        # error already corrected on the optical branch, left uncorrected here.
        # It made the acoustic channel look like a reliable safety net -- the
        # projection capped the cost of losing optical aiding at roughly
        # 0.12 m^2 for USBL against a measured 4.4 m^2 in the compound scenario
        # -- and a manager that believes it will be caught does not pay to
        # descend, which is the strongest action it has.
        # Would *this* technique return a fix from where we believe we are?
        #
        # This replaces a test on ``acoustic_fix_age_s``, which reports only
        # whether the technique currently in use just delivered. That is present
        # tense. When a single beacon goes quiet it says nothing about whether a
        # surface USBL or a seabed LBL array would answer, and reading it as
        # though it did made all three techniques score identically in 87% of
        # compound-scenario decisions -- so the cheapest won by default, on the
        # axis that moves position error most.
        #
        # The counterfactual is computable rather than guessed: transponder
        # positions are surveyed, the sonar equation is in ``acoustics.py``, and
        # the manager knows its own estimate. This is the acoustic counterpart
        # of the optical availability model, which has answered the same
        # question -- would a fix arrive under a configuration I am not
        # currently flying? -- since the beginning.
        if not self._acoustic_reachable(obs, technique):
            return math.inf

        exceedance = min(max(obs.innovation_exceedance_rate, 0.0), 1.0)
        if exceedance >= 1.0:
            # Nothing is getting through: the acoustic channel is worth nothing
            # and the caller's min() against the unaided branch will bind.
            return math.inf
        effective_period = technique.fix_period_s / (1.0 - exceedance)

        # Evidence that the technique currently in use has stopped answering.
        #
        # Everything above this line is a *counterfactual*: geometry, the sonar
        # equation and terrain relief all predict what a technique would deliver
        # if selected. None of them can detect that a transponder has failed, a
        # path is blocked, or a surface asset has left station, because none of
        # those change the geometry. A vehicle whose beacon has gone silent
        # therefore keeps re-selecting it, forever, on a prediction that remains
        # perfectly valid and completely wrong.
        #
        # The correction applies only to the incumbent technique, and that
        # restriction is the whole point. `acoustic_fix_age_s` reports the age
        # of the last accepted fix from the technique *in use*; it is present
        # tense and says nothing about the others. Reading it as though it
        # described all of them made all three score identically in 87% of
        # compound-scenario decisions, which is defect 9 in the register. Read
        # correctly it is evidence about exactly one candidate: this one is
        # demonstrably not delivering, whatever the geometry says.
        #
        # A technique silent for several of its own fix periods is treated as
        # delivering nothing, which lets a technique with a different failure
        # mode -- terrain matching does not depend on any transponder -- win on
        # its own merits rather than on a prediction the incumbent will never
        # be penalised for failing.
        #
        # Returned as the *unaided* projection rather than as infinity, and the
        # difference is not cosmetic. A silent technique leaves the vehicle with
        # no acoustic aiding, which is a large finite cost; infinity is a
        # different claim and it breaks the machinery downstream. The switch
        # margin compares a rival against the incumbent's objective, and an
        # infinite incumbent makes that comparison true for every rival, so the
        # manager switches on every decision. That was measured, not reasoned
        # about: the first version of this rule returned infinity and compound
        # cross-track error went from 9.7 m to 79.4 m through pure chattering.
        #
        # The penalty extends to every technique that listens for a transponder,
        # not to the incumbent alone, and the reason is that the vehicle cannot
        # tell *why* its technique went silent. A failed beacon, a blocked path
        # and a departed support vessel are indistinguishable from onboard: all
        # three look like interrogations going out and nothing coming back.
        # Penalising only the incumbent sends the vehicle around the transponder
        # techniques in turn, each healthy on geometry until selected and found
        # silent -- 37 technique changes in one compound run, with cross-track
        # error worse than dead reckoning.
        #
        # Generalising across all of them is therefore conservative rather than
        # correct, and it has a measurable cost: when only the surface asset has
        # gone, the seabed array is still working and the vehicle abandons it
        # too. That cost is reported rather than engineered away, because the
        # alternative is to give the manager knowledge of the fault cause, which
        # is exactly the privileged input the isolation rules forbid.
        #
        # Terrain matching forms its own group. Silence on the acoustic link is
        # no evidence against it -- it listens for nothing -- and its own silence
        # is no evidence against the transponders. But it is not exempt from the
        # rule: a vehicle working an area with no prior survey has nothing to
        # match against, and without this it would re-select a technique that
        # cannot answer for the rest of the mission, which is the failure the
        # rule exists to prevent.
        # Penalising only the incumbent sends the vehicle around the three of
        # them in turn, each looking healthy on geometry until it is selected
        # and found silent too: measured at 37 technique changes in one compound
        # run, with cross-track error worse than dead reckoning.
        #
        # Terrain matching does not share the dependency. It carries no
        # transponder, listens to nothing, and measures the seabed with an echo
        # sounder, so evidence that the positioning link is down is no evidence
        # against it. That asymmetry is the entire reason a vehicle carrying
        # both is more capable than one carrying either, and it is what the
        # manager is here to exploit.
        if (
            self._acoustic_link_is_silent(obs, effective_period)
            and _failure_group(technique)
            == _failure_group(self._current.acoustic)
        ):
            return obs.position_covariance_trace + max(
                obs.covariance_growth_rate, 0.0
            ) * DECISION_HORIZON_S
        between_fixes = growth * effective_period
        floor = self._acoustic_floor_m2(technique, altitude_m, terrain_gradient)
        if technique.gives_position:
            # A position fix resets all three axes; what remains is the growth
            # accumulated since the previous one.
            return floor + between_fixes
        # Range-only: one direction is constrained, the rest keep growing over
        # the decision horizon rather than being reset by each fix.
        return (
            floor
            + between_fixes
            + RANGE_ONLY_UNCONSTRAINED_FRACTION * growth * DECISION_HORIZON_S
        )

    def _blackout(self, obs: Observables) -> bool:
        """Whether the vehicle has genuinely lost the ability to navigate.

        This is the hardest judgement the manager makes, because the action it
        gates -- abandoning the survey to surface -- is irreversible. Two earlier
        criteria were both wrong, in opposite directions, and the record is worth
        keeping because the failure modes are instructive.

        Testing ``acoustic_fix_age_s`` alone was too eager. That observable
        reports whether the technique *currently selected* just delivered, so an
        intermittent beacon going quiet was read as total loss; the vehicle
        surfaced on every run of the turbid/DVL-loss family and failed every
        mission there, where the fixed policy completed all of them.

        Testing geometric reachability alone was impossible to satisfy. The LBL
        array spans the survey area by design, so *some* technique is always
        reachable in principle, the blackout could never be declared, and the
        terminal action never executed once in 150 runs.

        Neither asks the right question. "Could a transponder answer?" and "did
        the last one answer?" are both proxies for what actually matters: **has
        the position estimate degraded past the point where the survey means
        anything?** A vehicle whose channels are quiet but whose estimate is
        still tight can keep working. A vehicle whose estimate has diverged
        cannot, however healthy its transponder geometry looks.

        The threshold is the mission specification, not a fitted value. Coverage
        requires passing within ``survey_tolerance_m`` of a waypoint; once the
        one-sigma position uncertainty reaches that tolerance the vehicle can no
        longer place itself on the survey line at all, so the trace of the
        position covariance reaching three times its square is the point where
        surveying has stopped being possible.
        """
        if obs.optical_available or obs.dvl_bottom_lock or obs.dvl_water_track:
            return False
        return obs.position_covariance_trace >= UNSURVEYABLE_COVARIANCE_M2

    @staticmethod
    def _acoustic_reachable(
        obs: Observables, technique: AcousticTechnique
    ) -> bool:
        """Whether ``technique`` would return a fix from the estimated position.

        Evaluated with the physics already in ``acoustics.py`` -- the sonar
        equation, the detection threshold, the USBL range limit and the LBL
        dilution-of-precision limit -- at the *estimated* position and the
        *classified* noise level. Both are the manager's own beliefs, so a
        vehicle that has drifted or misread the environment will mis-predict its
        acoustic geometry, exactly as it can mis-predict optical availability.
        Nothing here consults truth.

        The classifier reports a noise class rather than a level, so the class
        is mapped back to the band level that defines it. An unclassified
        environment is treated as quiet: that is the benign reading, and it
        matches the default the classifier itself carries before it has seen
        enough of a window to commit.
        """
        level = NOISE_CLASS_LEVEL_DB.get(obs.environment_noise, NOISE_QUIET_DB)
        try:
            response = acoustic_response(
                technique,
                obs.estimated_position_m,
                NoiseState(spectral_level_db=level),
                # Terrain relief from the vehicle's own altimeter, never from
                # the scenario. For the transponder techniques this argument is
                # ignored; for terrain-relative navigation it is the whole
                # question, and supplying truth here would hand the manager a
                # privileged input that N2 forbids.
                terrain_gradient=obs.terrain_gradient_estimate,
                altitude_m=obs.altitude_m,
            )
        except (ValueError, ZeroDivisionError):  # pragma: no cover - guarded
            return False
        return bool(response.available)

    def _acoustic_link_is_silent(
        self, obs: Observables, effective_period: float
    ) -> bool:
        """Has the technique in use stopped delivering, despite predicting it would?

        This is the one piece of acoustic evidence that is *not* a
        counterfactual. Geometry, the sonar equation and terrain relief all
        predict; none of them can see a failed beacon, a blocked path, or a
        surface asset that has left station, because none of those change the
        geometry. Without this test a vehicle whose beacon has gone quiet
        re-selects it on every decision for the rest of the mission, on a
        prediction that stays valid and stays wrong.
        """
        return (
            obs.acoustic_fix_age_s
            >= SILENCE_TOLERANCE_PERIODS * max(effective_period, 1e-9)
        )

    @staticmethod
    def _acoustic_floor_m2(
        technique: AcousticTechnique,
        altitude_m: float = ALTITUDE_NOMINAL_M,
        terrain_gradient: float = 0.0,
    ) -> float:
        """Position variance the technique itself can hold, in m^2.

        This must not be the optical aided floor. Reusing that constant claimed
        acoustic aiding settles to sigma ~= 0.15 m, when the static sweep shows
        USBL holding 2.093 m median in the compound scenario -- an optimism of
        roughly seventy-fold. The manager believed the acoustic channel would
        catch it and therefore declined to pay for the altitude that actually
        restores optical aiding, which is the strongest action it has: at c=1.2
        descending from 3 m to 1 m turns a 14.6 m fix into a 0.023 m one.

        The floor is built from the technique's own declared error terms rather
        than fitted. Range precision is the timing floor carried by sound speed;
        a bearing-measuring technique adds cross-range error that grows with
        slant range, which is why USBL is less precise than its range alone
        implies. ``NOMINAL_SLANT_RANGE_M`` is the survey geometry, not a tuned
        value: the seabed sits at -20 m and the vehicle flies a few metres above
        it, so a surface or corner transponder is of this order away.
        """
        if technique.terrain_relative:
            # A terrain match has no round trip to time and no transponder to
            # be far from. Its accuracy is set by the relief it is matching
            # against -- sigma = sigma_depth / gradient -- so the floor rises
            # without limit as the seabed flattens, and below the technique's
            # identifiability threshold there is no floor at all because there
            # is no fix. Returning the unsurveyable variance there is what stops
            # the manager selecting terrain matching over a plain.
            if terrain_gradient < technique.minimum_gradient:
                return UNSURVEYABLE_COVARIANCE_M2
            sigma_t = technique.depth_sigma_m / terrain_gradient
            return float(sigma_t * sigma_t)

        sigma_r = SOUND_SPEED_MPS * technique.timing_floor_s
        variance = sigma_r * sigma_r
        if technique.bearing_sigma_rad > 0.0:
            # Cross-range error is bearing error times slant range, so it grows
            # as the vehicle descends away from a surface transponder. This is
            # what makes the altitude decision a genuine trade rather than a
            # one-way bet: descending buys optical quality -- at c = 1.2 the
            # laser goes from a 0.43 m fix at 3 m to a 0.018 m fix at 1 m -- and
            # simultaneously costs acoustic accuracy, because the vehicle is
            # further from the surface. In the compound scenario the acoustic
            # penalty wins, and 3.0 m is the better altitude in eight of the
            # nine channel-by-technique combinations measured.
            #
            # Holding this at a constant nominal slant hid that trade entirely:
            # the manager then declined to descend for no reason connected to
            # the physics, which happened to be right in the compound case and
            # would have been wrong wherever optical recovery dominates.
            slant = max(abs(SEABED_DEPTH_M) - altitude_m, MINIMUM_SLANT_RANGE_M)
            cross = slant * technique.bearing_sigma_rad
            # Two cross-range axes, one range axis.
            variance += 2.0 * cross * cross
        return variance

    def _mission_action(
        self, mode: Mode, dt: float, obs: Observables
    ) -> MissionAction:
        """Tier 3. Disabled entirely under ablations A1 and A2.

        Holding is bounded. An unbounded hold is not a conservative choice, it
        is a mission failure with extra steps: the survey clock keeps running,
        the vehicle keeps drifting on the current, and if the condition that
        triggered the hold cannot be resolved by waiting -- a failed DVL, water
        too turbid for any configuration -- the vehicle waits until the time
        limit and completes nothing.

        The first implementation omitted the timeout that ``MODE_MANAGER_SPEC``
        §4 declares. Its effect was stark and easy to misread: the full manager
        failed 100% of the coupled turbidity/DVL-loss runs while ablation A2,
        which has tier 3 switched off entirely, failed none of them. Read
        naively that says mission actions are harmful. It actually says an
        unbounded hold is harmful, which is why the parameter was specified in
        the first place.
        """
        if not self.ablation.mission_actions:
            self._holding_for_s = 0.0
            return MissionAction.CONTINUE

        # Self-preservation is terminal. Once committed, stay committed.
        #
        # The decision is re-evaluated every tick, so without this a momentary
        # improvement dropped the mode out of DR_CRITICAL, reset the blackout
        # dwell, and returned the vehicle to surveying. Measured in the compound
        # family: surfacing was commanded at t = 103.5 s and held for two
        # seconds, against an ascent that needs sixty-six. The vehicle decided to
        # save itself roughly thirty times per run and never once got there.
        #
        # The commitment is what makes it self-preservation rather than a brief
        # climb. By the time it fires the survey is already abandoned; reversing
        # on a transient leaves the vehicle mid-water with neither a survey nor a
        # recovery. A vehicle that has concluded it cannot navigate does not get
        # to change its mind because one fix arrived.
        if self._surfacing_committed:
            return MissionAction.SURFACE_FOR_GPS

        if mode is not Mode.DR_CRITICAL:
            # Leaving the critical mode re-arms the hold for a future episode.
            self._holding_for_s = 0.0
            return MissionAction.CONTINUE

        if self._holding_for_s >= self.hold_timeout_s:
            # The hold is spent. Release it and keep surveying: waiting longer
            # is a pure loss, but the vehicle may still be navigable, and a
            # spent hold on its own is not evidence that it is not. Abandonment
            # is decided by the blackout dwell below, on its own evidence.
            return MissionAction.CONTINUE

        # A total blackout must *persist* before it justifies abandoning the
        # survey. Surfacing is irreversible and guarantees mission failure, so it
        # demands stronger evidence than any reversible action, and an
        # instantaneous test is not evidence -- a single tick with optical out,
        # bottom lock momentarily dropped and the acoustic beacon between
        # interrogations satisfies it, and that happens routinely in scenarios
        # the vehicle recovers from unaided.
        #
        # This is the same asymmetry the mode machine already applies, taken one
        # step further: escalating conservatism should be prompt because it is
        # cheap to undo, whereas giving up on the mission should not be.
        if self._blackout(obs):
            self._blackout_for_s += dt
        else:
            self._blackout_for_s = 0.0

        if self._blackout_for_s >= self.blackout_timeout_s:
            self._surfacing_committed = True
            return MissionAction.SURFACE_FOR_GPS

        # Hold only for a fix that could actually arrive.
        #
        # Holding costs more than mission time. The vehicle station-keeps on its
        # own estimate, so while that estimate drifts the controller faithfully
        # converts estimate error into physical error -- it moves the vehicle to
        # keep a drifting number constant. That is what a real vehicle holding
        # station on a diverging INS does, and it is a reasonable price for a fix
        # that is coming. It is a pure loss for one that is not.
        #
        # Without this check the manager held through the total-blackout window
        # of the compound scenario, where optical, velocity, and acoustic aiding
        # are all down at once and no configuration or wait can recover any of
        # them, and more than doubled its own path error relative to simply
        # continuing. The condition is decided from observables alone: recency of
        # the last acoustic fix, and whether optical aiding is available now.
        if not self._fix_opportunity(obs):
            return MissionAction.CONTINUE

        self._holding_for_s += dt
        return (
            MissionAction.HOLD_FOR_FIX
            if self.ablation.acoustic_aiding
            else MissionAction.RETURN_TO_LAST_GOOD_FIX
        )

    def _fix_opportunity(self, obs: Observables) -> bool:
        """Whether waiting could plausibly produce an absolute fix."""
        return bool(
            obs.optical_available
            or obs.acoustic_fix_age_s <= self.fix_opportunity_window_s
        )
