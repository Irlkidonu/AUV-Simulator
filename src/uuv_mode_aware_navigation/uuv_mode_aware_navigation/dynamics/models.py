"""Additive vehicle-dynamics models; legacy mission.Vehicle remains unchanged."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass(frozen=True)
class VehicleCommand:
    velocity_body_mps: np.ndarray
    yaw_rate_rps: float=0.0


@dataclass
class FirstOrderDynamics:
    position_m: np.ndarray
    current_mps: np.ndarray=field(default_factory=lambda:np.zeros(3))
    velocity_mps: np.ndarray=field(default_factory=lambda:np.zeros(3))
    response: float=0.3

    def step(self, command: VehicleCommand, dt: float):
        previous=self.velocity_mps.copy()
        self.velocity_mps=(1-self.response)*self.velocity_mps+self.response*(np.asarray(command.velocity_body_mps)+self.current_mps)
        self.position_m=np.asarray(self.position_m)+self.velocity_mps*dt
        return (self.velocity_mps-previous)/dt


@dataclass
class SixDofState:
    position_m: np.ndarray
    velocity_body_mps: np.ndarray=field(default_factory=lambda:np.zeros(3))
    attitude_rpy_rad: np.ndarray=field(default_factory=lambda:np.zeros(3))
    angular_rate_rps: np.ndarray=field(default_factory=lambda:np.zeros(3))


@dataclass
class SixDofDynamics:
    state: SixDofState
    mass_kg: float=52.0
    inertia_kgm2: np.ndarray=field(default_factory=lambda:np.array([2.8,3.1,4.2]))
    linear_drag: np.ndarray=field(default_factory=lambda:np.array([18.,24.,30.]))
    quadratic_drag: np.ndarray=field(default_factory=lambda:np.array([22.,30.,38.]))
    maximum_force_n: float=120.0
    maximum_torque_nm: float=18.0

    def step(self, command: VehicleCommand, current_world_mps: np.ndarray, dt: float):
        if dt<=0: raise ValueError("dt must be positive")
        target=np.asarray(command.velocity_body_mps,dtype=float)
        error=target-self.state.velocity_body_mps
        force=np.clip(80.0*error,-self.maximum_force_n,self.maximum_force_n)
        drag=self.linear_drag*self.state.velocity_body_mps+self.quadratic_drag*np.abs(self.state.velocity_body_mps)*self.state.velocity_body_mps
        acceleration=(force-drag)/self.mass_kg
        self.state.velocity_body_mps+=acceleration*dt
        yaw_error=command.yaw_rate_rps-self.state.angular_rate_rps[2]
        torque=np.clip(8.0*yaw_error,-self.maximum_torque_nm,self.maximum_torque_nm)
        angular_acceleration=np.zeros(3); angular_acceleration[2]=torque/self.inertia_kgm2[2]
        self.state.angular_rate_rps+=angular_acceleration*dt
        self.state.attitude_rpy_rad+=self.state.angular_rate_rps*dt
        yaw=self.state.attitude_rpy_rad[2]; c,s=np.cos(yaw),np.sin(yaw)
        rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        self.state.position_m+=(rotation@self.state.velocity_body_mps+np.asarray(current_world_mps))*dt
        return acceleration,angular_acceleration
