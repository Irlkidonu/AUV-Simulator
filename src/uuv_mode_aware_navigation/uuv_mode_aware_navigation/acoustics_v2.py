"""Platform-v2 acoustic geometry and asynchronous packet models."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .acoustics import (AcousticTechnique, NoiseState, SOUND_SPEED_MPS,
                        MAXIMUM_LBL_DOP, signal_to_noise_db, range_sigma_m,
                        DETECTION_THRESHOLD_DB, dilution_of_precision)


@dataclass(frozen=True)
class AcousticWorldGeometry:
    lbl_transponders_m: tuple[tuple[float,float,float], ...]
    single_beacon_m: tuple[float,float,float]
    vessel_position_m: tuple[float,float,float]
    vessel_velocity_mps: tuple[float,float,float] = (0.0,0.0,0.0)
    transponder_calibration_sigma_m: float = 0.02

    def vessel_at(self, validity_time_s: float) -> np.ndarray:
        return np.asarray(self.vessel_position_m)+validity_time_s*np.asarray(self.vessel_velocity_mps)


@dataclass(frozen=True)
class GeometryAwareFix:
    available: bool
    technique: str
    covariance_m2: np.ndarray | None
    slant_range_m: float
    dop: float
    snr_db: float
    reason: str


def geometry_aware_fix(technique: AcousticTechnique, vehicle_position_m: Sequence[float],
                       geometry: AcousticWorldGeometry, noise: NoiseState,
                       validity_time_s: float=0.0) -> GeometryAwareFix:
    vehicle=np.asarray(vehicle_position_m,dtype=float)
    if technique.name=="lbl":
        points=[np.asarray(p,dtype=float) for p in geometry.lbl_transponders_m]
    elif technique.name=="usbl":
        points=[geometry.vessel_at(validity_time_s)]
    else:
        points=[np.asarray(geometry.single_beacon_m,dtype=float)]
    ranges=np.asarray([np.linalg.norm(vehicle-p) for p in points])
    slant=float(np.max(ranges)); snr=signal_to_noise_db(slant,noise)
    if snr<DETECTION_THRESHOLD_DB:
        return GeometryAwareFix(False,technique.name,None,slant,math.inf,snr,"below_detection_threshold")
    sigma_r=range_sigma_m(slant,technique,noise)
    if technique.name=="lbl":
        dop=dilution_of_precision(vehicle,points)
        if not np.isfinite(dop) or dop>MAXIMUM_LBL_DOP:
            return GeometryAwareFix(False,technique.name,None,slant,dop,snr,"poor_geometry")
        # Build the horizontal range Jacobian at the actual world geometry.
        jac=np.asarray([(vehicle[:2]-p[:2])/np.linalg.norm(vehicle[:2]-p[:2]) for p in points])
        variance=sigma_r**2+geometry.transponder_calibration_sigma_m**2
        covariance=np.linalg.inv(jac.T@jac/variance)
    elif technique.name=="usbl":
        horizontal=max(float(np.linalg.norm(vehicle[:2]-points[0][:2])),1e-9)
        radial=(vehicle[:2]-points[0][:2])/horizontal
        tangent=np.array([-radial[1],radial[0]])
        sigma_cross=max(slant*technique.bearing_sigma_rad,sigma_r)
        rotation=np.column_stack((radial,tangent))
        covariance=rotation@np.diag([sigma_r**2,sigma_cross**2])@rotation.T
        dop=1.0
    else:
        radial=(vehicle[:2]-points[0][:2]); norm=max(float(np.linalg.norm(radial)),1e-9)
        radial=radial/norm
        covariance=np.outer(radial,radial)*sigma_r**2+np.eye(2)*1e6
        dop=math.inf
    return GeometryAwareFix(True,technique.name,covariance,slant,dop,snr,"available")


@dataclass(frozen=True)
class AcousticPacket:
    sequence: int
    validity_time_s: float
    arrival_time_s: float
    dropped: bool
    technique: str


@dataclass
class AcousticPacketModel:
    processing_delay_s: float=0.08
    jitter_sigma_s: float=0.02
    packet_loss_probability: float=0.0
    _sequence: int=0

    def __post_init__(self):
        if self.processing_delay_s<0 or self.jitter_sigma_s<0 or not 0<=self.packet_loss_probability<=1:
            raise ValueError("invalid acoustic timing parameters")

    def generate(self, validity_time_s: float, slant_range_m: float,
                 technique: AcousticTechnique, rng: np.random.Generator) -> AcousticPacket:
        self._sequence+=1
        round_trips=technique.interrogations_per_fix
        propagation=2.0*slant_range_m*round_trips/SOUND_SPEED_MPS
        jitter=max(0.0,float(rng.normal(0.0,self.jitter_sigma_s)))
        arrival=validity_time_s+propagation+self.processing_delay_s+jitter
        dropped=bool(rng.random()<self.packet_loss_probability)
        return AcousticPacket(self._sequence,float(validity_time_s),float(arrival),dropped,technique.name)
