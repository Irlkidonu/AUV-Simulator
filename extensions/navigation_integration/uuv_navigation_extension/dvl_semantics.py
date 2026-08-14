"""Explicit navigation/body/DVL transformations for the external bridge."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


def quaternion_xyzw_to_rotation_nav_from_body(q) -> np.ndarray:
    x, y, z, w = map(float, q)
    norm = np.linalg.norm([x,y,z,w])
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    x,y,z,w = np.asarray([x,y,z,w])/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rpy_rotation_dvl_from_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr,sr=np.cos(roll),np.sin(roll); cp,sp=np.cos(pitch),np.sin(pitch); cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


@dataclass(frozen=True)
class DvlComputation:
    velocity_body_origin: np.ndarray
    lever_velocity_body: np.ndarray
    physical_velocity_dvl: np.ndarray
    r3_equivalent_velocity_dvl: np.ndarray


def compute_dvl(velocity_nav, rotation_nav_from_body, angular_velocity_body,
                translation_body_to_dvl_body, rotation_dvl_from_body) -> DvlComputation:
    v_nav=np.asarray(velocity_nav,float); r_nb=np.asarray(rotation_nav_from_body,float)
    omega=np.asarray(angular_velocity_body,float); lever=np.asarray(translation_body_to_dvl_body,float)
    r_db=np.asarray(rotation_dvl_from_body,float)
    if any(x.shape != s for x,s in ((v_nav,(3,)),(omega,(3,)),(lever,(3,)),(r_nb,(3,3)),(r_db,(3,3)))):
        raise ValueError("invalid DVL transform dimensions")
    v_body=r_nb.T@v_nav; lever_velocity=np.cross(omega,lever)
    physical=r_db@(v_body+lever_velocity)
    # A body-origin estimator interface requires a physical displaced-head
    # measurement to be lever-arm compensated before delivery.
    equivalent=physical-r_db@lever_velocity
    return DvlComputation(v_body,lever_velocity,physical,equivalent)


class DeterministicRateGate:
    def __init__(self, rate_hz: float):
        if rate_hz <= 0: raise ValueError("rate must be positive")
        self.period=1.0/rate_hz; self.last=None
    def due(self, acquired_at: float) -> bool:
        if self.last is None or acquired_at-self.last >= self.period-1e-9:
            self.last=float(acquired_at); return True
        return False
