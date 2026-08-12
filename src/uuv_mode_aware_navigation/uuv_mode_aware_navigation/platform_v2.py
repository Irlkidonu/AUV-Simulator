"""Coherent platform-v2 capability, prediction, recovery and delayed-aiding loop."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .capability import (CapabilityDegradationPredictor, CapabilityEvidence,
                         CapabilityTrendEvidence, ProbabilisticCapabilityFilter)
from .delayed_estimator import DelayedPositionMeasurement, FixedLagNavigationFilter
from .localization import OpticalLocalizationSignal
from .recovery import ActiveRecoveryPlanner, RecoveryDecision, RecoveryState
from .recovery import RecoveryAction
from .selection.action_space import ActionSpaceV2, SelectionConditions


@dataclass(frozen=True)
class DVLSignal:
    bottom_lock: bool
    water_track: bool
    age_s: float
    lock_probability: float
    lock_probability_trend_per_s: float = 0.0


@dataclass(frozen=True)
class AcousticServiceEvidence:
    name: str
    responding: bool
    gives_position: bool
    dop: float
    sigma_m: float
    age_s: float = 0.0


@dataclass(frozen=True)
class AcousticSignal:
    available: bool
    validity_time_s: float
    arrival_time_s: float
    position_m: np.ndarray | None
    covariance_m2: np.ndarray | None
    dop: float
    infrastructure_available: bool
    expected_period_s: float
    dropped: bool = False
    # Technique-specific services observed through valid advertisements,
    # handshakes or replies. This is not the simulator's scenario label.
    observable_services: frozenset[str] = frozenset()
    service_evidence: tuple[AcousticServiceEvidence,...] = ()

    @property
    def sigma_m(self):
        if self.covariance_m2 is None:
            return math.inf
        eigenvalues=np.linalg.eigvalsh(np.asarray(self.covariance_m2,dtype=float))
        return math.sqrt(float(np.max(eigenvalues))) if np.all(eigenvalues>0) else math.inf


@dataclass(frozen=True)
class PlatformStepInput:
    time_s: float
    dt_s: float
    optical: OpticalLocalizationSignal
    optical_quality_trend_per_s: float
    dvl: DVLSignal
    acoustic: AcousticSignal
    imu_age_s: float
    innovation_exceedance_rate: float
    altitude_m: float
    speed_mps: float
    image_blur_m: float


@dataclass(frozen=True)
class PlatformStepOutput:
    belief: object
    forecast: object
    recovery: RecoveryDecision
    delayed_acoustic_reason: str
    acoustic_update_accepted: bool
    selected_speed_mps: float
    selected_altitude_m: float
    mission_action: str


class PlatformV2Coordinator:
    """One observable-only decision path across the existing subsystems."""

    def __init__(self, estimator=None, capability_filter=None, predictor=None, recovery=None,
                 action_space=None):
        self.estimator=estimator or FixedLagNavigationFilter(fixed_lag_s=15.0)
        self.capability_filter=capability_filter or ProbabilisticCapabilityFilter()
        self.predictor=predictor or CapabilityDegradationPredictor()
        self.recovery=recovery or ActiveRecoveryPlanner()
        self.action_space=action_space or ActionSpaceV2.default()
        self._blackout_s=0.0
        self._last_acoustic_dop=math.inf
        self._absolute_fix_age_s=0.0

    def _consume_acoustic(self, signal: AcousticSignal, now_s: float):
        if not signal.available or signal.position_m is None:
            return False,"unavailable"
        measurement=DelayedPositionMeasurement(
            signal.validity_time_s,signal.arrival_time_s,np.asarray(signal.position_m,dtype=float),
            signal.sigma_m,signal.dropped)
        outcome=self.estimator.update_delayed_acoustic_position(measurement)
        return outcome.accepted,outcome.reason

    def step(self, state: PlatformStepInput):
        if abs(self.estimator.current_time_s-state.time_s)>1e-6:
            raise ValueError("coordinator time must match estimator time")
        acoustic_accepted,acoustic_reason=self._consume_acoustic(state.acoustic,state.time_s)
        absolute_observed=bool(state.optical.available or acoustic_accepted)
        self._absolute_fix_age_s=(0.0 if absolute_observed else
                                  self._absolute_fix_age_s+state.dt_s)
        acoustic_age=max(0.0,state.time_s-state.acoustic.validity_time_s)
        evidence=CapabilityEvidence(
            optical_quality=state.optical.quality,
            optical_fix_observed=state.optical.available,
            dvl_bottom_lock=state.dvl.bottom_lock,
            dvl_water_track=state.dvl.water_track,
            acoustic_fix_age_s=acoustic_age,
            imu_age_s=state.imu_age_s,
            optical_sigma_m=state.optical.sigma_m,
            optical_fix_age_s=state.optical.age_s,
            dvl_age_s=state.dvl.age_s,
            dvl_lock_probability=state.dvl.lock_probability,
            acoustic_available=state.acoustic.available and acoustic_accepted,
            acoustic_dop=state.acoustic.dop,
            acoustic_sigma_m=state.acoustic.sigma_m,
            acoustic_infrastructure_available=state.acoustic.infrastructure_available,
            acoustic_validity_age_s=acoustic_age,
            innovation_exceedance_rate=state.innovation_exceedance_rate,
            extended_observables=True)
        belief=self.capability_filter.update(evidence,state.dt_s)
        dop_trend=(0.0 if not math.isfinite(self._last_acoustic_dop) or not math.isfinite(state.acoustic.dop)
                   else (state.acoustic.dop-self._last_acoustic_dop)/max(state.dt_s,1e-9))
        trend=CapabilityTrendEvidence(
            state.optical.quality,state.optical_quality_trend_per_s,state.optical.sigma_m,
            state.dvl.lock_probability,state.dvl.lock_probability_trend_per_s,
            acoustic_age,state.acoustic.expected_period_s,state.acoustic.dop,
            state.acoustic.infrastructure_available,dop_trend)
        forecast=self.predictor.predict(trend)
        self._last_acoustic_dop=state.acoustic.dop
        covariance_trace=float(np.trace(self.estimator.P[:3,:3]))
        # Water track observes velocity relative to the water.  Without an
        # absolute fix, uncertainty in the onboard current estimate integrates
        # into horizontal position uncertainty.  Account for that observable
        # uncertainty rather than treating water-track speed as ground speed.
        if state.dvl.water_track and not state.dvl.bottom_lock and self._absolute_fix_age_s>0:
            current_var=float(np.trace(self.estimator.P[9:12,9:12]))
            covariance_trace+=current_var*self._absolute_fix_age_s**2
        recovery_state=RecoveryState(
            optical_quality=state.optical.quality,
            optical_quality_trend_per_s=state.optical_quality_trend_per_s,
            altitude_m=state.altitude_m,speed_mps=state.speed_mps,
            image_blur_m=state.image_blur_m,acoustic_dop=state.acoustic.dop,
            acoustic_available=state.acoustic.available and state.acoustic.infrastructure_available,
            dvl_bottom_lock_probability=state.dvl.lock_probability,
            covariance_trace_m2=covariance_trace,
            expected_fix_s=max(0.0,state.acoustic.expected_period_s-acoustic_age),
            dvl_bottom_lock=state.dvl.bottom_lock,dvl_water_track=state.dvl.water_track,
            absolute_fix_age_s=self._absolute_fix_age_s)
        decision=self.recovery.decide(recovery_state)
        total_aiding=max(belief.usable_probability["optical"],
                         belief.usable_probability["acoustic"],
                         belief.usable_probability["velocity"])
        self._blackout_s=(self._blackout_s+state.dt_s if total_aiding<.2 else 0.0)
        effective_attenuation=max(0.0,-math.log(max(state.optical.quality,1e-6))
                                  /(2*max(state.altitude_m,.1)))
        infrastructure=set()
        if "single_beacon" in state.acoustic.observable_services:infrastructure.add("beacon")
        if "lbl" in state.acoustic.observable_services:infrastructure.add("lbl")
        if "usbl" in state.acoustic.observable_services:infrastructure.add("surface")
        conditions=SelectionConditions(
            optical_attenuation_m_inv=effective_attenuation,
            exposure_s=.01,texture_scale_m=.10,
            infrastructure=frozenset(infrastructure),
            total_blackout_s=self._blackout_s,
            fix_expected_s=max(0.0,state.acoustic.expected_period_s-acoustic_age),
            estimator_drift_m=math.sqrt(max(covariance_trace,0.0)))
        selected_speed=float(self.action_space.rank_speeds(conditions)[0].action.split(":")[1])
        selected_altitude=float(self.action_space.rank_altitudes(conditions)[0].action.split(":")[1])
        mission=self.action_space.reachable_mission_action(conditions)
        if decision.action is RecoveryAction.REDUCE_SPEED:selected_speed=min(selected_speed,.25)
        if decision.action is RecoveryAction.LOWER_ALTITUDE:
            selected_altitude=max(1.0,min(selected_altitude,state.altitude_m-1.0))
        if decision.action is RecoveryAction.HOLD_FOR_FIX:mission="hold_for_fix"
        if decision.action is RecoveryAction.SURFACE_FOR_GPS:mission="surface_for_gps"
        return PlatformStepOutput(belief,forecast,decision,acoustic_reason,acoustic_accepted,
                                  selected_speed,selected_altitude,mission)
