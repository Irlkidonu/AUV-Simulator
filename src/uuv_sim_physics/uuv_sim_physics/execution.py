"""Mission-level execution: the abstraction M1 should have defined.

**Architectural finding (L8).** ``DynamicsBackend`` was specified one level too
low. Its contract is ``step(commanded_velocity, dt)``, which is meaningful for
the reduced integrator and meaningless for a rigid-body plant that has no inner
loop and is actuated by thrust. Forcing Gazebo into that signature would have
required inventing a velocity controller inside the backend and calling it
"the plant", hiding the control design inside what claims to be physics.

The level that genuinely generalises is **mission intent**: go to this
waypoint, hold this depth and heading. Both modes can honour that; how they
honour it is properly plant-specific.

    Reduced mode   reference -> guidance -> ReducedBackend.step()
    Physics mode   reference -> controller -> allocator -> GazeboBackend

``DynamicsBackend`` is not withdrawn -- it is correct for what it describes, and
``ReducedBackend`` still satisfies it bit-for-bit. It is simply not the seam at
which the two execution modes meet. This module is that seam.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from . import world_builder
from .control.controller import Gains, Reference

__all__ = ["MissionOutcome", "ExecutionMode", "ReducedExecution",
           "PhysicsExecution"]


@dataclass
class MissionOutcome:
    """What both modes report, so they can be compared at all."""
    mode: str
    reached: bool
    final_error_m: float
    completion_time_s: float
    path_length_m: float
    trajectory: np.ndarray                      # (N, 3) world
    times: np.ndarray                           # (N,)
    speed: np.ndarray = field(default_factory=lambda: np.array([]))

    def summary(self) -> dict:
        return {"mode": self.mode, "reached": self.reached,
                "final_error_m": round(self.final_error_m, 4),
                "completion_time_s": round(self.completion_time_s, 3),
                "path_length_m": round(self.path_length_m, 4),
                "samples": int(len(self.times))}


class ExecutionMode(Protocol):
    """Execute a mission-level reference. Actuation is plant-specific."""

    name: str

    def goto(self, waypoint_xy, *, depth_m: float, heading_rad: float,
             tolerance_m: float = 0.15, timeout_s: float = 60.0
             ) -> MissionOutcome: ...


class ReducedExecution:
    """Deterministic lightweight mode: guidance straight into the integrator.

    ``ReducedBackend`` is used exactly as M1 froze it -- ``step(commanded
    velocity, dt)`` and nothing else. The guidance here is the *reduced mode's*
    guidance; it is not the physics controller and does not pretend to be.
    """

    name = "reduced"

    def __init__(self, start_xyz, gains: Gains | None = None,
                 dt: float = 0.02) -> None:
        from .reduced_backend import ReducedBackend
        self.backend = ReducedBackend(start_xyz)
        self.gains = gains or Gains()
        self.dt = dt

    def goto(self, waypoint_xy, *, depth_m: float, heading_rad: float,
             tolerance_m: float = 0.15, timeout_s: float = 60.0
             ) -> MissionOutcome:
        target = np.asarray(waypoint_xy, dtype=float)
        times, trajectory = [], []
        elapsed = 0.0
        reached = False

        while elapsed < timeout_s:
            position = self.backend.position
            times.append(elapsed)
            trajectory.append(position.copy())

            delta = target - position[:2]
            distance = float(np.linalg.norm(delta))
            if distance <= tolerance_m:
                reached = True
                break

            # World-frame guidance, then into the body frame the same way the
            # physics controller does it, so the two receive the same intent.
            c, s = math.cos(heading_rad), math.sin(heading_rad)
            forward = c * delta[0] + s * delta[1]
            lateral = -s * delta[0] + c * delta[1]
            surge = np.clip(self.gains.range_kp * forward,
                            -self.gains.max_speed_mps, self.gains.max_speed_mps)
            sway = np.clip(self.gains.lateral_kp * lateral,
                           -self.gains.max_speed_mps, self.gains.max_speed_mps)
            heave = np.clip(self.gains.depth_kp * (depth_m - position[2]),
                            -self.gains.max_vertical_mps,
                            self.gains.max_vertical_mps)
            command = np.array([c * surge - s * sway, s * surge + c * sway, heave])
            self.backend.step(command, self.dt)
            elapsed += self.dt

        trajectory = np.array(trajectory)
        return MissionOutcome(
            self.name, reached,
            float(np.linalg.norm(trajectory[-1][:2] - target)),
            elapsed, float(self.backend.path_length_m),
            trajectory, np.array(times))


class PhysicsExecution:
    """Physics mode: the same intent, through control, allocation and DART."""

    name = "physics"

    def __init__(self, backend, config: dict | None = None,
                 gains: Gains | None = None, rate_hz: float = 50.0) -> None:
        from .control.runner import ClosedLoopRunner
        self.backend = backend
        self.runner = ClosedLoopRunner(
            backend, config or world_builder.load_config(validated=True),
            gains or Gains(), rate_hz)

    def goto(self, waypoint_xy, *, depth_m: float, heading_rad: float,
             tolerance_m: float = 0.15, timeout_s: float = 60.0
             ) -> MissionOutcome:
        target = np.asarray(waypoint_xy, dtype=float)
        reference = Reference(waypoint_xy=tuple(target), depth_m=depth_m,
                              heading_rad=heading_rad)
        times, trajectory = [], []
        start = self.backend.time_s
        reached_at = None

        self.runner.controller.reset()
        last = start
        while True:
            now = self.backend.time_s
            elapsed = now - start
            if elapsed >= timeout_s:
                break
            step_dt = max(now - last, 1e-4)
            last = now

            from .control.controller import State
            state = State(position=self.backend.position,
                          velocity_body=self.backend.velocity,
                          yaw=self.backend.yaw, yaw_rate=self.backend.yaw_rate)
            times.append(elapsed)
            trajectory.append(state.position.copy())

            distance = float(np.linalg.norm(state.position[:2] - target))
            if distance <= tolerance_m and reached_at is None:
                reached_at = elapsed
                break

            wrench = self.runner.controller.step(reference, state, step_dt)
            thrusts, _ = self.runner.allocator(wrench)
            self.backend.apply_thrust(thrusts)
            time.sleep(0.008)

        self.backend.apply_thrust({"prop_left_joint": 0.0, "prop_right_joint": 0.0,
                                   "prop_sway_joint": 0.0, "prop_vert_joint": 0.0})
        trajectory = np.array(trajectory)
        path = float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())
        return MissionOutcome(
            self.name, reached_at is not None,
            float(np.linalg.norm(trajectory[-1][:2] - target)),
            reached_at if reached_at is not None else times[-1],
            path, trajectory, np.array(times))
