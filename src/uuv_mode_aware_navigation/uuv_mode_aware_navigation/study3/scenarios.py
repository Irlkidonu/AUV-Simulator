"""Truth-side Study 3 transition scenarios.

This module is intentionally separate from policy observations.  Scenario state
may generate measurements and evaluator labels, but no scenario field is ever
passed to :class:`Study3Policy`.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


FAMILIES=("S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
          "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
          "S3_COMPOUND_DVL_ACOUSTIC","S3_NOMINAL","S3_SUDDEN","S3_NO_RECOVERY")
PRIMARY=FAMILIES[:7]


class InfrastructureContext(Enum):
    """Evaluator-side deployment context; never supplied to a policy."""

    INFRASTRUCTURE_FREE="infrastructure_free"
    LBL_ENABLED="lbl_enabled"
    USBL_ENABLED="usbl_enabled"
    LIMITED_ACOUSTIC="limited_acoustic"
    INFRASTRUCTURE_TRANSITION="infrastructure_transition"


# This mapping is part of the physical scenario definition, not a policy input.
# It deliberately mixes supported and self-contained missions.
FAMILY_INFRASTRUCTURE={
    "S3_OPTICAL_GRADUAL":InfrastructureContext.INFRASTRUCTURE_FREE,
    "S3_DVL_GRADUAL":InfrastructureContext.LIMITED_ACOUSTIC,
    "S3_ACOUSTIC_GEOMETRY_ASYNC":InfrastructureContext.LBL_ENABLED,
    "S3_INFRASTRUCTURE_WARNING":InfrastructureContext.INFRASTRUCTURE_TRANSITION,
    "S3_RECOVERY":InfrastructureContext.LIMITED_ACOUSTIC,
    "S3_COMPOUND_OPTICAL_ACOUSTIC":InfrastructureContext.LBL_ENABLED,
    "S3_COMPOUND_DVL_ACOUSTIC":InfrastructureContext.USBL_ENABLED,
    "S3_NOMINAL":InfrastructureContext.LBL_ENABLED,
    "S3_SUDDEN":InfrastructureContext.INFRASTRUCTURE_FREE,
    "S3_NO_RECOVERY":InfrastructureContext.INFRASTRUCTURE_FREE,
}


def deployed_acoustic_services(family: str,time_s: float,horizon_s: float=120.0)->frozenset[str]:
    """Return truth-side services whose required assets are presently deployed.

    Availability after deployment is still decided by range, geometry and the
    sonar equation.  The transition family models a USBL support vessel leaving
    at 68% of the mission; silence/handshake loss, not this label, is observable.
    """
    context=FAMILY_INFRASTRUCTURE[family]
    if context is InfrastructureContext.LBL_ENABLED:return frozenset({"lbl"})
    if context is InfrastructureContext.USBL_ENABLED:return frozenset({"usbl"})
    if context is InfrastructureContext.LIMITED_ACOUSTIC:return frozenset({"single_beacon"})
    if context is InfrastructureContext.INFRASTRUCTURE_TRANSITION:
        return frozenset({"usbl"}) if time_s/horizon_s<.68 else frozenset()
    return frozenset()


@dataclass(frozen=True)
class PhysicalState:
    turbidity: float
    dvl_lock_probability: float
    dvl_water_track_probability: float
    acoustic_response_probability: float
    acoustic_noise_db: float
    infrastructure_available: bool
    vessel_offset_m: float
    degradation_active: bool
    infrastructure_context: InfrastructureContext
    deployed_acoustic_services: frozenset[str]
    current_east_mps: float=0.0
    current_north_mps: float=0.0
    dvl_noise_scale: float=1.0
    imu_drift_mps2: float=0.0
    lbl_geometry_scale: float=1.0
    service_response_probability: tuple[tuple[str,float],...]=()
    # Interactive truth-side hardware crashout. False for every registered
    # scenario/environment, so existing scientific behavior is unchanged.
    dvl_forced_unavailable: bool=False

    def response_probability(self,service:str)->float:
        return dict(self.service_response_probability).get(service,
            self.acoustic_response_probability)


def physical_state(family: str,time_s: float,horizon_s: float=120.0)->PhysicalState:
    """Deterministic physical schedule; evaluator/sensor-generator use only."""
    if family not in FAMILIES:raise ValueError(f"unknown Study 3 family {family}")
    u=min(max(time_s/horizon_s,0.0),1.0)
    ramp=min(max((u-.25)/.45,0.0),1.0)
    sudden=float(u>=.50)
    recovery=(min(max((u-.25)/.22,0.0),1.0)-min(max((u-.62)/.20,0.0),1.0))
    optical=ramp if family in {FAMILIES[0],FAMILIES[5]} else recovery if family==FAMILIES[4] else sudden if family==FAMILIES[8] else 0.0
    dvl=ramp if family in {FAMILIES[1],FAMILIES[6]} else sudden if family==FAMILIES[8] else 0.0
    acoustic=ramp if family in {FAMILIES[2],FAMILIES[3],FAMILIES[5],FAMILIES[6]} else recovery if family==FAMILIES[4] else 0.0
    if family==FAMILIES[9]: optical=dvl=acoustic=min(max((u-.25)/.15,0.0),1.0)
    services=deployed_acoustic_services(family,time_s,horizon_s)
    acoustic_dependent=family in {FAMILIES[2],FAMILIES[3],FAMILIES[6]}
    baseline_optical=.42 if acoustic_dependent else .05
    # Bottom track and water track are separate physical returns.  Water track
    # survives gradual bottom-lock loss where suspended scatterers remain, but
    # the sudden and no-recovery families explicitly remove both velocity aids.
    water_loss=(sudden if family==FAMILIES[8] else
                min(max((u-.48)/.30,0.0),1.0) if family==FAMILIES[9] else 0.0)
    # Preserve a substantial healthy-aiding phase before the registered
    # transition: response loss begins at 70% and is complete by 90%.
    acoustic_response_loss=(min(max((u-.70)/.20,0.0),1.0)
                            if family in {FAMILIES[2],FAMILIES[3],FAMILIES[5],FAMILIES[6]}
                            else acoustic)
    return PhysicalState(min(1.0,baseline_optical+.95*optical),max(.02,1-.98*dvl),
                         max(.02,.90-.88*water_loss),max(.02,1-.98*acoustic_response_loss),45+35*acoustic,
                         bool(services),5+75*acoustic,
                         bool(max(optical,dvl,acoustic)>.02),
                         FAMILY_INFRASTRUCTURE[family],services)
