"""Comparator policies (COMPARATOR_SPEC.md).

Every policy here answers the same question -- *what sensing configuration and
motion should the vehicle adopt right now?* -- and every one is handed the same
observables, the same estimator, the same mission, and the same vehicle. They
differ only in what they choose to do.

The bracket
-----------
``C1`` (best fixed policy) and ``C5`` (oracle, handed the true fault schedule)
form a bracket. The proposed manager should land **between** them, and the paper
reports what fraction of the oracle's benefit automatic inference recovers::

    oracle_recovery = (outcome(C1) - outcome(P)) / (outcome(C1) - outcome(C5))

Publishing that fraction is the strongest available answer to "did you cripple
the baseline?", because it shows how much headroom existed and that the method
did not claim all of it. A method that appears to *beat* the oracle is treated as
evidence of a defect, investigated, and reported -- never as a result.

``C3`` is the comparator a reviewer will demand: continuous covariance adaptation
driven by optical quality, which is the measurement-weighting approach the
earlier work in this workspace already published. If the proposed manager cannot
beat ``C3`` on mission outcomes, Paper 2 has no system-level contribution.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Sequence

from .manager import (
    DECISION_HORIZON_S,
    DEFAULT_CANDIDATES,
    ManagerAblation,
    MissionAction,
    MissionCosts,
    ModeAwareManager,
    SPEED_NOMINAL_MPS,
    SPEED_REDUCED_MPS,
    VehicleConfiguration,
)
from .modes import Mode, ModeThresholds, Observables
from .optics import (
    ALTITUDE_LOW_M,
    ALTITUDE_NOMINAL_M,
    CAMERA_COAXIAL,
    CAMERA_OFFAXIS,
    CONFIGURATIONS,
    LIDAR,
    WaterState,
    channel_response,
)
from .sensors import FaultKind, FaultSchedule

__all__ = [
    "PolicyDecision",
    "Policy",
    "ProposedPolicy",
    "FixedPolicy",
    "ResidualOnlyPolicy",
    "CovarianceOnlyPolicy",
    "DeadReckoningPolicy",
    "PerfectAvailability",
    "OraclePolicy",
    "build_policies",
]

_CHANNELS_BY_NAME = {c.name: c for c in CONFIGURATIONS}


@dataclass(frozen=True)
class PolicyDecision:
    configuration: VehicleConfiguration
    mode: Optional[Mode] = None
    #: Whether the policy admits absolute (optical/acoustic) aiding at all.
    use_absolute_aiding: bool = True
    #: Multiplier applied to the optical measurement covariance. Only the
    #: covariance-adaptation comparators move this away from 1.0.
    optical_covariance_scale: float = 1.0


class Policy(Protocol):
    name: str

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        ...


# ---------------------------------------------------------------------------
# P -- the proposed manager
# ---------------------------------------------------------------------------
class ProposedPolicy:
    """The full mode-aware manager."""

    def __init__(
        self,
        availability,
        thresholds: ModeThresholds = ModeThresholds(),
        costs: MissionCosts = MissionCosts(),
        ablation: ManagerAblation = ManagerAblation(),
        name: str = "proposed",
    ) -> None:
        self.name = name
        self.manager = ModeAwareManager(
            availability, thresholds=thresholds, costs=costs, ablation=ablation
        )

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        d = self.manager.update(obs, dt)
        return PolicyDecision(configuration=d.configuration, mode=d.mode)


# ---------------------------------------------------------------------------
# C1 -- best fixed policy
# ---------------------------------------------------------------------------
class FixedPolicy:
    """One static configuration, applied in all conditions.

    This is the genuine "best you can do without condition awareness", and the
    single most important comparator in the paper: if it is weak, every number
    downstream of it is worthless.

    Its configuration is therefore **selected, not assumed**. ``scripts/
    select_fixed_policy.py`` evaluates every configuration in the manager's own
    action space over the whole development scenario family and picks the one
    with the best aggregate outcome. The default below is the outcome of that
    search, and the full sweep table is published so a reader can check that the
    baseline was not quietly handicapped -- rule R7.

    Passing a configuration explicitly is for tests and for the hindsight oracle,
    not for the reported baseline.
    """

    def __init__(
        self,
        configuration: VehicleConfiguration = VehicleConfiguration(
            CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS
        ),
        name: str = "fixed",
    ) -> None:
        self.name = name
        self.configuration = configuration

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        return PolicyDecision(configuration=self.configuration)


# ---------------------------------------------------------------------------
# C2 -- residual-only reactive policy
# ---------------------------------------------------------------------------
class ResidualOnlyPolicy:
    """Reacts to what just happened, with no mode abstraction and no vehicle actions.

    When the active channel stops producing fixes it tries the next one. It never
    changes altitude or speed, never suspends the mission, and never anticipates:
    it can only respond after aiding has already been lost.
    """

    ORDER = (CAMERA_OFFAXIS, LIDAR, CAMERA_COAXIAL)

    def __init__(self, name: str = "residual_only", patience_s: float = 2.0) -> None:
        self.name = name
        self.patience_s = patience_s
        self._index = 0
        self._starved_for = 0.0

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        if obs.optical_available:
            self._starved_for = 0.0
        else:
            self._starved_for += dt
            if self._starved_for >= self.patience_s:
                self._index = (self._index + 1) % len(self.ORDER)
                self._starved_for = 0.0
        return PolicyDecision(
            configuration=VehicleConfiguration(
                self.ORDER[self._index], ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS
            )
        )


# ---------------------------------------------------------------------------
# C3 -- continuous covariance adaptation from optical quality
# ---------------------------------------------------------------------------
class CovarianceOnlyPolicy:
    """The measurement-weighting approach: trust the camera in proportion to quality.

    Fixed configuration, fixed altitude, fixed speed. The only thing that changes
    is how much the filter trusts the optical fix. This is the approach the
    earlier work in this workspace published, and it is the comparator that
    decides whether Paper 2 is a new paper or a re-titled one.
    """

    def __init__(
        self,
        name: str = "covariance_only",
        minimum_scale: float = 1.0,
        maximum_scale: float = 400.0,
    ) -> None:
        self.name = name
        self.minimum_scale = minimum_scale
        self.maximum_scale = maximum_scale

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        # Low quality -> inflate optical covariance. Continuous, monotone, and
        # entirely confined to tier 1.
        q = min(max(obs.optical_quality, 0.0), 1.0)
        scale = self.minimum_scale + (self.maximum_scale - self.minimum_scale) * (
            1.0 - q
        ) ** 2
        return PolicyDecision(
            configuration=VehicleConfiguration(
                CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS
            ),
            optical_covariance_scale=float(scale),
        )


# ---------------------------------------------------------------------------
# C4 -- no absolute aiding
# ---------------------------------------------------------------------------
class DeadReckoningPolicy:
    """Inertial plus DVL plus depth only. The performance floor.

    Included so the contribution of absolute aiding is legible: without a floor,
    a reader cannot tell how much of any method's performance comes from having
    aiding at all rather than from managing it well.
    """

    def __init__(self, name: str = "dead_reckoning") -> None:
        self.name = name

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        return PolicyDecision(
            configuration=VehicleConfiguration(
                CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS
            ),
            use_absolute_aiding=False,
        )


# ---------------------------------------------------------------------------
# C5 -- oracle
# ---------------------------------------------------------------------------
class PerfectAvailability:
    """Clairvoyant answer to the counterfactual the availability model estimates.

    The learned model is asked: *if I flew at this altitude on this channel,
    would I get a fix?* -- and must answer from observables alone, about the
    present. This class answers the same question by evaluating the true optical
    channel against the true water profile and the true fault schedule, and it
    answers it **over the manager's own projection horizon** rather than only at
    the current instant.

    The lookahead is what makes the bracket informative. Restricted to the
    present, ground truth turns out to buy almost nothing over the learned model
    -- the development campaign put the two within 0.005 m of each other -- so
    the bracket collapses and the recovery fraction becomes a ratio of two
    numbers that are both noise. The headroom a mode-aware manager could in
    principle exploit is not *what is happening now*, which observables already
    reveal, but *what is about to happen*, which they cannot. A clairvoyant
    reconfigures before degradation arrives; the proposed manager can only react
    once evidence accumulates. That gap is the quantity ``oracle_recovery``
    exists to measure.

    The horizon is deliberately the manager's own ``DECISION_HORIZON_S``, so the
    oracle is answering exactly the question the manager's projection poses --
    not a longer one chosen to widen the bracket.

    Not implementable on a vehicle. That is the point.
    """

    def __init__(
        self,
        schedule: FaultSchedule,
        horizon_s: float = DECISION_HORIZON_S,
        samples: int = 5,
    ) -> None:
        self.schedule = schedule
        self.horizon_s = horizon_s
        self.samples = max(int(samples), 1)
        self._profile = None
        self._t = 0.0
        # channel_response is deterministic without an rng, and the manager
        # queries the same handful of (altitude, channel) pairs at every tick.
        # Without this the oracle dominates campaign runtime by an order of
        # magnitude, for identical results.
        self._cache: dict[tuple, bool] = {}

    def observe_truth(self, t: float, profile) -> None:
        self._t = t
        self._profile = profile

    def _available_at(self, t: float, altitude_m: float, config) -> bool:
        key = (round(t, 3), round(altitude_m, 4), config.name)
        hit = self._cache.get(key)
        if hit is None:
            water = self._profile.at(t)
            hit = bool(channel_response(water, altitude_m, config).available)
            self._cache[key] = hit
        return hit

    def predict(
        self,
        quality: float,
        observed_altitude_m: float,
        candidate_altitude_m: float,
        configuration: str,
        quality_trend: float = 0.0,
    ) -> float:
        # ``quality_trend`` is accepted and ignored. The trend exists so a causal
        # model can guess where conditions are heading; this class reads the
        # water profile over the horizon directly, so a hint about the future is
        # of no use to something that already has it. Accepting the argument
        # keeps the two interchangeable at the call site, which is what makes the
        # oracle a bound on the same architecture rather than a different one.
        config = _CHANNELS_BY_NAME.get(configuration)
        if config is None or self._profile is None:
            return 0.0
        step = self.horizon_s / max(self.samples - 1, 1)
        hits = 0.0
        for i in range(self.samples):
            t = self._t + i * step
            if self.schedule.active(FaultKind.OPTICAL_BLACKOUT, t):
                continue
            if self._available_at(t, candidate_altitude_m, config):
                hits += 1.0
        return hits / self.samples


class OraclePolicy:
    """The proposed manager with perfect information. The ceiling (C5).

    This comparator is deliberately **the same policy** as the proposed method:
    same action space, same cost model, same budget, same hysteresis, same mode
    machine. The single difference is that its counterfactual availability
    predictions are exact rather than inferred, because it may read the true
    water state and the true fault schedule.

    That construction is what makes ``oracle_recovery`` mean something. An
    oracle built as a separate hand-written heuristic -- as an earlier version of
    this module was -- is not an upper bound at all: it is just another policy
    that happens to be given privileged information, and it can be beaten by the
    proposed method for reasons that have nothing to do with information. It then
    silently invalidates the bracket it exists to provide.

    Labelled an ORACLE everywhere it appears, including every figure legend and
    table, because it receives information no deployable system could have
    (rule R3).
    """

    def __init__(
        self,
        schedule: FaultSchedule,
        thresholds: ModeThresholds = ModeThresholds(),
        costs: MissionCosts = MissionCosts(),
        name: str = "oracle",
    ) -> None:
        self.name = name
        self.schedule = schedule
        self.availability = PerfectAvailability(schedule)
        self.manager = ModeAwareManager(
            self.availability, thresholds=thresholds, costs=costs
        )

    def observe_truth(self, t: float, profile) -> None:
        """Privileged channel. Only policies with this method are given truth."""
        self.availability.observe_truth(t, profile)

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        d = self.manager.update(obs, dt)
        return PolicyDecision(configuration=d.configuration, mode=d.mode)


def build_policies(availability, schedule: FaultSchedule) -> dict:
    """The full comparator set, in the order the manuscript reports them."""
    return {
        "proposed": ProposedPolicy(availability),
        "fixed": FixedPolicy(),
        "residual_only": ResidualOnlyPolicy(),
        "covariance_only": CovarianceOnlyPolicy(),
        "dead_reckoning": DeadReckoningPolicy(),
        "oracle": OraclePolicy(schedule),
        "ablation_a1": ProposedPolicy(
            availability,
            ablation=ManagerAblation.covariance_only(),
            name="ablation_a1",
        ),
        "ablation_a2": ProposedPolicy(
            availability,
            ablation=ManagerAblation(mission_actions=False),
            name="ablation_a2",
        ),
    }
