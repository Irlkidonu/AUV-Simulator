"""Operational capability belief without physical fault diagnosis."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from ..modes import Mode, ModeStateMachine, Observables


CAPABILITIES=("inertial","velocity","optical","acoustic")


@dataclass(frozen=True)
class CapabilityEvidence:
    optical_quality: float
    optical_fix_observed: bool
    dvl_bottom_lock: bool
    dvl_water_track: bool
    acoustic_fix_age_s: float
    imu_age_s: float
    optical_sigma_m: float = math.inf
    optical_fix_age_s: float = math.inf
    dvl_age_s: float = math.inf
    dvl_lock_probability: float = 0.0
    acoustic_available: bool = False
    acoustic_dop: float = math.inf
    acoustic_sigma_m: float = math.inf
    acoustic_infrastructure_available: bool = False
    acoustic_validity_age_s: float = math.inf
    innovation_exceedance_rate: float = 0.0
    extended_observables: bool = False


@dataclass(frozen=True)
class CapabilityBelief:
    usable_probability: Mapping[str,float]
    point_mode: Mode
    confidence: float

    def __post_init__(self):
        if set(self.usable_probability)!=set(CAPABILITIES):
            raise ValueError("belief must contain every declared capability")
        if any(not 0<=p<=1 for p in self.usable_probability.values()):
            raise ValueError("probabilities must lie in [0,1]")


class DeterministicModeAdapter:
    """Expose the unchanged legacy state machine through the belief interface."""
    def __init__(self,state_machine=None): self.state_machine=state_machine or ModeStateMachine()
    def update(self,observation: Observables,dt: float):
        decision=self.state_machine.update(observation,dt)
        p={
          "inertial":float(observation.imu_age_s<=self.state_machine.thresholds.imu_max_age_s),
          "velocity":float((observation.dvl_bottom_lock and observation.dvl_age_s<=self.state_machine.thresholds.dvl_max_age_s) or observation.dvl_water_track),
          "optical":float(observation.optical_available and observation.optical_quality>=self.state_machine.thresholds.quality_marginal),
          "acoustic":float(observation.acoustic_fix_age_s<=self.state_machine.thresholds.acoustic_max_age_s),
        }
        return CapabilityBelief(p,decision.mode,1.0)


@dataclass
class ProbabilisticCapabilityFilter:
    """Transparent binary Bayesian filters over operational usability."""
    prior_usable: float=.95
    loss_hazard_per_s: float=.01
    recovery_hazard_per_s: float=.03
    _belief: dict|None=None

    def __post_init__(self):
        if not 0<self.prior_usable<1: raise ValueError("prior must be nondegenerate")
        self._belief={name:self.prior_usable for name in CAPABILITIES}

    @staticmethod
    def _bayes(prior,positive,sensitivity=.92,specificity=.95):
        likelihood=sensitivity if positive else 1-sensitivity
        false_likelihood=1-specificity if positive else specificity
        return prior*likelihood/max(prior*likelihood+(1-prior)*false_likelihood,1e-12)

    def update(self,evidence: CapabilityEvidence,dt: float):
        if dt<0: raise ValueError("dt cannot be negative")
        finite=lambda value: math.isfinite(value)
        velocity_score = 0.0
        if evidence.dvl_bottom_lock:
            velocity_score = max(.95, evidence.dvl_lock_probability)
        elif evidence.dvl_water_track:
            velocity_score = .75
        if finite(evidence.dvl_age_s):
            velocity_score *= math.exp(-max(evidence.dvl_age_s-2.0,0.0)/2.0)
        optical_score = 0.0
        if evidence.optical_fix_observed:
            # Infinite optional fields denote the original evidence schema,
            # where fix/quality were the only available optical observables.
            precision = (min(1.0, .10/max(evidence.optical_sigma_m,.005))
                         if finite(evidence.optical_sigma_m) else 1.0)
            freshness = math.exp(-max(evidence.optical_fix_age_s,0.0)/5.0) if finite(evidence.optical_fix_age_s) else 1.0
            # A geometrically verified P5-v4 fix is direct evidence that the
            # localization capability is usable.  Raw image quality may remain
            # low over a dark but matchable texture and must not veto that
            # stronger, task-level observation (the predictor still receives
            # image diagnostics separately).
            optical_score = precision*freshness
        acoustic_score = 0.0
        legacy_acoustic=(not evidence.extended_observables and not evidence.acoustic_available and not evidence.acoustic_infrastructure_available
                         and not finite(evidence.acoustic_sigma_m))
        if legacy_acoustic and evidence.acoustic_fix_age_s<=30:
            acoustic_score=1.0
        elif (evidence.acoustic_available and evidence.acoustic_infrastructure_available
                and finite(evidence.acoustic_sigma_m) and evidence.acoustic_sigma_m>0
                and finite(evidence.acoustic_dop) and evidence.acoustic_dop<=6.0):
            geometry = max(0.0,1.0-evidence.acoustic_dop/7.0)
            freshness = math.exp(-max(evidence.acoustic_validity_age_s,0.0)/30.0) if finite(evidence.acoustic_validity_age_s) else 0.0
            acoustic_score = geometry*freshness*min(1.0,1.0/evidence.acoustic_sigma_m)
        scores={
          "inertial":1.0 if evidence.imu_age_s<=.5 else 0.0,
          "velocity":velocity_score,
          "optical":optical_score,
          "acoustic":acoustic_score,
        }
        for name in CAPABILITIES:
            prior=self._belief[name]
            predicted=prior*(1-self.loss_hazard_per_s*dt)+(1-prior)*self.recovery_hazard_per_s*dt
            # Soft evidence retains calibrated distinctions (e.g., precise vs
            # marginal fixes) while remaining an interpretable binary Bayes filter.
            score=min(max(scores[name],0.0),1.0)
            likelihood_usable=.08+.84*score
            likelihood_unusable=.92-.84*score
            denominator=predicted*likelihood_usable+(1-predicted)*likelihood_unusable
            posterior=predicted*likelihood_usable/max(denominator,1e-12)
            if evidence.innovation_exceedance_rate>.30 and name in {"optical","acoustic"}:
                posterior*=.5
            self._belief[name]=min(max(posterior,1e-6),1-1e-6)
        p=self._belief
        if p["inertial"]<.5 or (p["velocity"]<.5 and max(p["optical"],p["acoustic"])<.5): mode=Mode.DR_CRITICAL
        elif p["velocity"]<.5: mode=Mode.VELOCITY_AIDING_LOST
        elif p["optical"]<.5: mode=Mode.OPTICAL_LOST
        elif p["optical"]<.8: mode=Mode.OPTICAL_DEGRADED
        else: mode=Mode.NOMINAL
        confidence=max(abs(value-.5)*2 for value in p.values())
        return CapabilityBelief(dict(p),mode,confidence)
