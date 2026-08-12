"""Candidate prediction as a measurable component, separate from selection."""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from ..availability import AvailabilityModel


@dataclass(frozen=True)
class CandidatePrediction:
    candidate: str
    p_accepted: float
    predicted_sigma_m: float
    expected_information_gain: float
    time_to_fix_s: float
    switching_cost: float
    p_failure: float
    infrastructure_class: str


@dataclass(frozen=True)
class OpticalCandidatePredictor:
    model: AvailabilityModel

    def predict(self,candidate: str,quality: float,observed_altitude_m: float,
                candidate_altitude_m: float,predicted_sigma_m: float,
                prior_variance_m2: float,quality_trend: float=0.0):
        p=self.model.predict(quality,observed_altitude_m,candidate_altitude_m,candidate,quality_trend)
        measurement_variance=max(predicted_sigma_m**2,1e-12)
        information=max(0.0,prior_variance_m2-prior_variance_m2*measurement_variance/(prior_variance_m2+measurement_variance))
        return CandidatePrediction(candidate,p,predicted_sigma_m,p*information,
                                   .5,0.0,1-p,"onboard")


@dataclass(frozen=True)
class CapabilityTrendEvidence:
    optical_quality: float
    optical_quality_trend_per_s: float
    optical_sigma_m: float
    dvl_lock_probability: float
    dvl_lock_trend_per_s: float
    acoustic_fix_age_s: float
    acoustic_expected_period_s: float
    acoustic_dop: float
    acoustic_infrastructure_available: bool
    acoustic_dop_trend_per_s: float = 0.0


@dataclass(frozen=True)
class CapabilityForecast:
    horizon_s: float
    probability: dict[str, float]
    impending: frozenset[str]
    time_to_loss_s: dict[str, float]


@dataclass(frozen=True)
class CapabilityDegradationPredictor:
    """Observable trend projection; no fault schedule or environment truth."""
    horizon_s: float = 10.0
    optical_quality_floor: float = .25
    dvl_lock_floor: float = .20
    maximum_acoustic_dop: float = 6.0

    @staticmethod
    def _time_to_floor(value, trend, floor):
        return ((value-floor)/-trend if trend < 0 and value > floor else
                0.0 if value <= floor else math.inf)

    def predict(self, evidence: CapabilityTrendEvidence):
        optical_time=self._time_to_floor(evidence.optical_quality,evidence.optical_quality_trend_per_s,self.optical_quality_floor)
        dvl_time=self._time_to_floor(evidence.dvl_lock_probability,evidence.dvl_lock_trend_per_s,self.dvl_lock_floor)
        silence_margin=max(evidence.acoustic_expected_period_s*3-evidence.acoustic_fix_age_s,0.0)
        geometry_margin=self._time_to_floor(
            self.maximum_acoustic_dop-evidence.acoustic_dop,
            -evidence.acoustic_dop_trend_per_s,0.0)
        acoustic_margin=min(silence_margin,geometry_margin)
        acoustic_time=(acoustic_margin if evidence.acoustic_infrastructure_available
                       and evidence.acoustic_dop<=self.maximum_acoustic_dop else 0.0)
        times={"optical":optical_time,"velocity":dvl_time,"acoustic":acoustic_time}
        probability={
            "optical":min(max((evidence.optical_quality+evidence.optical_quality_trend_per_s*self.horizon_s-self.optical_quality_floor)/.75,0.0),1.0)*min(1.0,.1/max(evidence.optical_sigma_m,.005)),
            "velocity":min(max(evidence.dvl_lock_probability+evidence.dvl_lock_trend_per_s*self.horizon_s,0.0),1.0),
            "acoustic":float(acoustic_time>self.horizon_s)*max(0.0,1-evidence.acoustic_dop/7.0),
        }
        impending=frozenset(name for name,value in times.items() if value<=self.horizon_s)
        return CapabilityForecast(self.horizon_s,probability,impending,times)


@dataclass(frozen=True)
class OpticalEvidenceForecast:
    warning: bool
    health_score: float
    time_to_loss_s: float
    declining_diagnostics: int


@dataclass(frozen=True)
class OpticalEvidenceForecasterConfig:
    window: int = 4
    horizon_s: float = 16.0
    score_floor: float = .40
    decline_quorum: int = 2
    minimum_slope_per_s: float = .001


class OpticalEvidenceForecaster:
    """Forecast optical capability from normalized P5 evidence, not image texture.

    Each margin is normalized against an existing P5-v4 acceptance boundary.
    The median score and a Theil--Sen-style median pairwise slope prevent one
    texture-sensitive diagnostic from controlling the forecast.  Truth,
    turbidity, scenario identity and future schedules are not inputs.
    """
    def __init__(self,config=OpticalEvidenceForecasterConfig()):
        self.config=config;self._history=[];self._last_time=-math.inf

    @staticmethod
    def _margins(signal):
        kp=max(min(signal.keypoints_a,signal.keypoints_b),1)
        sigma_margin=((.1-signal.sigma_m)/.1 if math.isfinite(signal.sigma_m) else 0.)
        reprojection=((2.-signal.reprojection_px)/2. if math.isfinite(signal.reprojection_px) else 0.)
        ambiguity=((.5-signal.ambiguity_ratio)/.5 if math.isfinite(signal.ambiguity_ratio) else 0.)
        return np.clip(((signal.inliers-12)/36,(signal.inliers/kp-.05)/.75,
                        (signal.inlier_fraction-.5)/.5,reprojection,
                        ambiguity,sigma_margin),0.,1.)

    def observe(self,time_s,signal):
        # P5 evidence changes only on a new image. Repeated aged samples must not
        # masquerade as additional temporal confirmation.
        if signal.age_s<=1e-9 and time_s>self._last_time:
            self._history.append((float(time_s),self._margins(signal)))
            self._history=self._history[-self.config.window:];self._last_time=float(time_s)
        if len(self._history)<self.config.window or not signal.available:
            score=float(np.median(self._history[-1][1])) if self._history else 0.
            return OpticalEvidenceForecast(False,score,math.inf,0)
        slopes=[]
        for diagnostic in range(6):
            slopes.append(float(np.median([
                (self._history[b][1][diagnostic]-self._history[a][1][diagnostic])/
                (self._history[b][0]-self._history[a][0])
                for a in range(len(self._history)) for b in range(a+1,len(self._history))])))
        score=float(np.median(self._history[-1][1]));slope=float(np.median(slopes))
        c=self.config
        ttl=((score-c.score_floor)/-slope if slope < -c.minimum_slope_per_s and score>c.score_floor
             else 0. if score<=c.score_floor else math.inf)
        declining=sum(s < -c.minimum_slope_per_s for s in slopes)
        return OpticalEvidenceForecast(bool(ttl<=c.horizon_s and declining>=c.decline_quorum),
                                       score,float(ttl),declining)
