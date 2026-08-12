"""Fixed-lag event replay for delayed platform-v2 acoustic measurements."""

from __future__ import annotations

from dataclasses import dataclass
import copy
import math
import numpy as np

from .estimator import FilterConfig, NavigationFilter, UpdateOutcome


@dataclass(frozen=True)
class DelayedPositionMeasurement:
    validity_time_s: float
    arrival_time_s: float
    position_m: np.ndarray
    sigma_m: float
    dropped: bool=False


@dataclass(frozen=True)
class DelayedUpdateOutcome:
    accepted: bool
    reason: str
    update: UpdateOutcome|None
    replayed_events: int=0


@dataclass(frozen=True)
class _Event:
    time_s: float
    priority: int
    sequence: int
    kind: str
    payload: tuple


class FixedLagNavigationFilter:
    """NavigationFilter with deterministic out-of-sequence event replay.

    The published filter is not modified. Platform-v2 records every prediction
    and update in validity-time order. A delayed acoustic fix is inserted at its
    measurement time and all later onboard events are replayed, preventing a
    stale position from being treated as if it described the arrival instant.
    """
    def __init__(self,config=FilterConfig(),initial_position=(0,0,-17),fixed_lag_s=10.0):
        if fixed_lag_s<=0: raise ValueError("fixed_lag_s must be positive")
        self.config=config; self.initial_position=tuple(initial_position); self.fixed_lag_s=fixed_lag_s
        self.filter=NavigationFilter(config,initial_position); self.current_time_s=0.0
        self._events=[]; self._sequence=0

    @property
    def position(self): return self.filter.position
    @property
    def P(self): return self.filter.P.copy()

    def _append(self,time_s,priority,kind,*payload):
        self._sequence+=1; self._events.append(_Event(float(time_s),priority,self._sequence,kind,payload))

    def predict(self,accel_measured,dt):
        if dt<0: raise ValueError("dt cannot be negative")
        self.current_time_s+=dt; self._append(self.current_time_s,0,"predict",np.asarray(accel_measured).copy(),float(dt)); self.filter.predict(accel_measured,dt)

    def update_position(self,position,sigma):
        self._append(self.current_time_s,1,"position",np.asarray(position).copy(),float(sigma),self.filter.fusion)
        return self.filter.update_position(position,sigma)

    def update_velocity(self,velocity):
        self._append(self.current_time_s,1,"velocity",np.asarray(velocity).copy(),self.filter.fusion)
        return self.filter.update_velocity(velocity)

    def update_water_velocity(self,velocity):
        self._append(self.current_time_s,1,"water_velocity",np.asarray(velocity).copy(),self.filter.fusion)
        return self.filter.update_water_velocity(velocity)

    def update_depth(self,depth):
        self._append(self.current_time_s,1,"depth",float(depth),self.filter.fusion)
        return self.filter.update_depth(depth)

    def _apply(self,event):
        if event.kind=="predict": self.filter.predict(*event.payload)
        elif event.kind=="position":
            position,sigma,fusion=event.payload;self.filter.fusion=fusion
            return self.filter.update_position(position,sigma)
        elif event.kind=="velocity":
            velocity,fusion=event.payload;self.filter.fusion=fusion
            return self.filter.update_velocity(velocity)
        elif event.kind=="water_velocity":
            velocity,fusion=event.payload;self.filter.fusion=fusion
            return self.filter.update_water_velocity(velocity)
        elif event.kind=="depth":
            depth,fusion=event.payload;self.filter.fusion=fusion
            return self.filter.update_depth(depth)
        else: raise RuntimeError(f"unknown event {event.kind}")

    def reinitialize_position(self,position,sigma):
        """Apply a surface GPS reset and restart the fixed-lag history there."""
        position=np.asarray(position,dtype=float)
        if position.shape!=(3,) or not np.all(np.isfinite(position)) or sigma<=0:
            raise ValueError("invalid position reinitialization")
        self.filter.x[:3]=position
        self.filter.P[:3,:]=0.0;self.filter.P[:,:3]=0.0
        self.filter.P[:3,:3]=np.eye(3)*float(sigma)**2
        self.initial_position=tuple(position)
        self._events=[];self._sequence=0

    def _rebuild(self,target_sequence):
        self.filter=NavigationFilter(self.config,self.initial_position); outcome=None; replayed=0
        for event in sorted(self._events,key=lambda e:(e.time_s,e.priority,e.sequence)):
            result=self._apply(event); replayed+=1
            if event.sequence==target_sequence: outcome=result
        return outcome,replayed

    def update_delayed_acoustic_position(self,measurement: DelayedPositionMeasurement):
        if measurement.dropped:return DelayedUpdateOutcome(False,"packet_dropped",None)
        if measurement.arrival_time_s>self.current_time_s+1e-9:return DelayedUpdateOutcome(False,"not_arrived",None)
        if measurement.validity_time_s>measurement.arrival_time_s:return DelayedUpdateOutcome(False,"invalid_timestamps",None)
        if self.current_time_s-measurement.validity_time_s>self.fixed_lag_s:return DelayedUpdateOutcome(False,"outside_fixed_lag",None)
        if not np.all(np.isfinite(measurement.position_m)) or not math.isfinite(measurement.sigma_m) or measurement.sigma_m<=0:
            return DelayedUpdateOutcome(False,"invalid_measurement",None)
        self._append(measurement.validity_time_s,1,"position",np.asarray(measurement.position_m).copy(),
                     float(measurement.sigma_m),self.filter.fusion); sequence=self._sequence
        outcome,replayed=self._rebuild(sequence)
        return DelayedUpdateOutcome(bool(outcome and outcome.accepted),"replayed",outcome,replayed)
