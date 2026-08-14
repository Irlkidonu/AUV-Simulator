"""Closed-loop runner: the layer that joins the others without merging them.

    guidance/reference -> controller -> body wrench -> allocator
        -> atomic actuator command -> GazeboBackend -> plant -> state feedback

Every stage above is a separate object, and this loop only sequences them. It
records each stage's output every tick, so a run can be interrogated at any
layer: what was asked, what wrench was demanded, what thrust was allocated,
whether it saturated, and what the plant did.

Mode is ``GROUND_TRUTH_CONTROL_VALIDATION`` and is written into every trace and
provenance record. State feedback is privileged. This is a physics/control
validation path and must never be reused for perception.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from .. import world_builder
from ..gazebo_backend import GazeboBackend
from .allocation import JOINT_ORDER, Allocator, Wrench
from .controller import MODE, Controller, Gains, Reference, State

__all__ = ["ClosedLoopRunner", "RunTrace"]


@dataclass
class RunTrace:
    mode: str = MODE
    rows: list = field(default_factory=list)

    def array(self, key: str) -> np.ndarray:
        return np.array([row[key] for row in self.rows], dtype=float)

    @property
    def t(self) -> np.ndarray:
        return self.array("t")

    def position(self) -> np.ndarray:
        return np.array([row["position"] for row in self.rows], dtype=float)

    def body_velocity(self) -> np.ndarray:
        return np.array([row["velocity_body"] for row in self.rows], dtype=float)

    def summary(self) -> dict:
        saturated = sum(1 for row in self.rows if row["saturated"])
        return {"samples": len(self.rows),
                "duration_s": float(self.t[-1]) if self.rows else 0.0,
                "ticks_saturated": saturated,
                "saturation_fraction": saturated / max(len(self.rows), 1)}


class ClosedLoopRunner:
    """Drive the plant through the full control stack at a fixed rate."""

    def __init__(self, backend: GazeboBackend, config: dict | None = None,
                 gains: Gains | None = None, rate_hz: float = 50.0) -> None:
        self.backend = backend
        self.config = config or world_builder.load_config(validated=True)
        self.controller = Controller(gains or Gains())
        self.allocator = Allocator(self.config)
        self.rate_hz = rate_hz

    def run(self, reference: Reference, duration_s: float,
            schedule=None) -> RunTrace:
        """Hold ``reference`` for ``duration_s`` of simulation time.

        ``schedule`` optionally maps elapsed time to a new reference, so a
        manoeuvre can change its setpoint mid-run without a second loop.
        """
        trace = RunTrace()
        self.controller.reset()
        dt = 1.0 / self.rate_hz
        start = self.backend.time_s
        last = start

        while True:
            now = self.backend.time_s
            elapsed = now - start
            if elapsed >= duration_s:
                break
            step_dt = max(now - last, 1e-4)
            last = now

            active = schedule(elapsed) if schedule else reference

            state = State(position=self.backend.position,
                          velocity_body=self.backend.velocity,
                          yaw=self.backend.yaw,
                          yaw_rate=self.backend.yaw_rate)

            wrench = self.controller.step(active, state, step_dt)
            thrusts, info = self.allocator(wrench)
            self.backend.apply_thrust(thrusts)

            trace.rows.append({
                "t": elapsed,
                "reference": asdict(active),
                "position": state.position.tolist(),
                "velocity_body": state.velocity_body.tolist(),
                "yaw": state.yaw,
                "yaw_rate": state.yaw_rate,
                "wrench": asdict(wrench),
                "thrust": dict(thrusts),
                "saturated": bool(info["saturated"]),
                "saturation_detail": info,
            })
            time.sleep(max(dt * 0.4, 0.002))

        self.backend.apply_thrust(dict.fromkeys(JOINT_ORDER, 0.0))
        return trace
