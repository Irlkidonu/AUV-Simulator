"""Observability-driven recovery actions for platform-v2 development."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class RecoveryAction(Enum):
    CONTINUE="continue"
    LOWER_ALTITUDE="lower_altitude"
    REDUCE_SPEED="reduce_speed"
    CHANGE_HEADING="change_heading"
    REPOSITION_FOR_ACOUSTICS="reposition_for_acoustics"
    HOLD_FOR_FIX="hold_for_fix"
    SURFACE_FOR_GPS="surface_for_gps"


@dataclass(frozen=True)
class RecoveryState:
    optical_quality: float
    optical_quality_trend_per_s: float
    altitude_m: float
    speed_mps: float
    image_blur_m: float
    acoustic_dop: float
    acoustic_available: bool
    dvl_bottom_lock_probability: float
    covariance_trace_m2: float
    expected_fix_s: float
    dvl_bottom_lock: bool=False
    dvl_water_track: bool=False
    absolute_fix_age_s: float=0.0


@dataclass(frozen=True)
class RecoveryDecision:
    action: RecoveryAction
    reason: str
    predictive: bool


@dataclass(frozen=True)
class ActiveRecoveryPlanner:
    quality_floor: float=.25
    horizon_s: float=10.0
    maximum_dop: float=6.0
    critical_covariance_m2: float=4.0

    def decide(self,state: RecoveryState):
        projected=state.optical_quality+state.optical_quality_trend_per_s*self.horizon_s
        if state.optical_quality>=self.quality_floor and projected<self.quality_floor and state.altitude_m>1.0:
            return RecoveryDecision(RecoveryAction.LOWER_ALTITUDE,"predicted_optical_loss",True)
        if state.optical_quality<self.quality_floor:
            if state.altitude_m>1.0:
                return RecoveryDecision(RecoveryAction.LOWER_ALTITUDE,"shorten_optical_path",False)
            if state.image_blur_m>.01 and state.speed_mps>.25:
                return RecoveryDecision(RecoveryAction.REDUCE_SPEED,"reduce_motion_blur",False)
        if state.acoustic_available and state.acoustic_dop>self.maximum_dop:
            return RecoveryDecision(RecoveryAction.REPOSITION_FOR_ACOUSTICS,"improve_acoustic_geometry",False)
        if state.dvl_bottom_lock_probability<.2 and state.altitude_m>1.0:
            return RecoveryDecision(RecoveryAction.LOWER_ALTITUDE,"restore_dvl_bottom_lock",False)
        if 0<state.expected_fix_s<=10 and state.covariance_trace_m2<self.critical_covariance_m2:
            return RecoveryDecision(RecoveryAction.HOLD_FOR_FIX,"fix_imminent",False)
        if state.covariance_trace_m2>=self.critical_covariance_m2:
            return RecoveryDecision(RecoveryAction.SURFACE_FOR_GPS,"no_safe_submerged_fix",False)
        return RecoveryDecision(RecoveryAction.CONTINUE,"capability_adequate",False)
