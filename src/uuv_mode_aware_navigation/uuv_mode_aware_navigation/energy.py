"""Separated sensing and full-platform energy accounting."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EnergyBreakdown:
    propulsion_j: float=0.0
    hotel_j: float=0.0
    compute_j: float=0.0
    optical_j: float=0.0
    acoustic_j: float=0.0
    actuation_j: float=0.0

    @property
    def sensing_j(self): return self.optical_j+self.acoustic_j
    @property
    def total_j(self): return self.propulsion_j+self.hotel_j+self.compute_j+self.optical_j+self.acoustic_j+self.actuation_j


@dataclass
class LegacyOpticalEnergyModel:
    energy_j: float=0.0
    def step(self,optical_power_w: float,dt: float):
        self.energy_j+=optical_power_w*dt
        return self.energy_j


@dataclass
class FullPlatformEnergyModel:
    hotel_power_w: float=28.0
    compute_power_w: float=14.0
    propulsion_idle_w: float=20.0
    propulsion_cubic_w_per_mps3: float=180.0
    battery_capacity_j: float=2.4e6
    breakdown: EnergyBreakdown=EnergyBreakdown()

    def step(self,speed_mps: float,optical_power_w: float,acoustic_power_w: float,
             actuation_power_w: float,dt: float):
        if dt<0 or min(speed_mps,optical_power_w,acoustic_power_w,actuation_power_w)<0:
            raise ValueError("energy inputs cannot be negative")
        prop=self.propulsion_idle_w+self.propulsion_cubic_w_per_mps3*speed_mps**3
        b=self.breakdown
        self.breakdown=EnergyBreakdown(
          b.propulsion_j+prop*dt,b.hotel_j+self.hotel_power_w*dt,
          b.compute_j+self.compute_power_w*dt,b.optical_j+optical_power_w*dt,
          b.acoustic_j+acoustic_power_w*dt,b.actuation_j+actuation_power_w*dt)
        return self.breakdown

    @property
    def state_of_charge(self):
        return max(0.0,1.0-self.breakdown.total_j/self.battery_capacity_j)
