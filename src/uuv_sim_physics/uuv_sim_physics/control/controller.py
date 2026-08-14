"""Guidance and control: reference -> body wrench.

Two layers, deliberately separate:

  ``Guidance``    turns a mission reference (a pose to hold, a waypoint to reach)
                  into desired body velocities and a heading.
  ``Controller``  turns those into a body wrench.

Nothing here knows about thrusters, Gazebo or topics; it produces a
``Wrench`` and stops. That is what lets the allocation be tested against
analytic geometry and the controller against a reference, independently.

Feedback source is the plant's true state. For M3 that is intentional and the
mode is labelled ``GROUND_TRUTH_CONTROL_VALIDATION`` -- we are validating the
physics and control stack, not perception. Nothing in this module may ever be
reused as a Paper 3 perception path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .allocation import Wrench

__all__ = ["Gains", "Reference", "State", "Controller", "MODE"]

#: Explicit label. A run whose provenance carries this string used privileged
#: state and is not a perception result.
MODE = "GROUND_TRUTH_CONTROL_VALIDATION"


def wrap(angle: float) -> float:
    """Wrap to (-pi, pi]. Heading error must take the short way round."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Gains:
    """Velocity-loop proportional gains are in N per m/s; yaw in N.m per rad."""
    surge_kp: float = 120.0
    surge_ki: float = 12.0
    sway_kp: float = 140.0
    sway_ki: float = 14.0
    heave_kp: float = 150.0
    heave_ki: float = 20.0

    # Heading: PD on angle. The plant has weak yaw damping (nR = -0.8), which
    # is what makes L1 possible, so derivative action carries most of the load.
    yaw_kp: float = 26.0
    yaw_kd: float = 14.0

    # Outer loops: position error -> commanded velocity.
    depth_kp: float = 0.55
    lateral_kp: float = 0.50
    range_kp: float = 0.45

    max_speed_mps: float = 0.60
    max_vertical_mps: float = 0.30
    integral_limit: float = 40.0


@dataclass
class Reference:
    """What the vehicle is being asked to do."""
    surge_mps: float | None = None       # direct velocity command
    sway_mps: float | None = None
    depth_m: float | None = None         # world z to hold
    heading_rad: float | None = None     # world yaw to hold
    waypoint_xy: tuple[float, float] | None = None


@dataclass
class State:
    """True plant state. Privileged; see MODE."""
    position: np.ndarray
    velocity_body: np.ndarray
    yaw: float
    yaw_rate: float


@dataclass
class Controller:
    gains: Gains = field(default_factory=Gains)
    mode: str = MODE
    _integral: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def reset(self) -> None:
        self._integral = np.zeros(3)

    def step(self, reference: Reference, state: State, dt: float) -> Wrench:
        g = self.gains

        # --- outer loop: reference -> desired body velocities ---
        if reference.waypoint_xy is not None:
            delta = np.array(reference.waypoint_xy) - state.position[:2]
            c, s = math.cos(state.yaw), math.sin(state.yaw)
            # World error into body axes.
            forward = c * delta[0] + s * delta[1]
            lateral = -s * delta[0] + c * delta[1]
            desired_surge = np.clip(g.range_kp * forward,
                                    -g.max_speed_mps, g.max_speed_mps)
            desired_sway = np.clip(g.lateral_kp * lateral,
                                   -g.max_speed_mps, g.max_speed_mps)
        else:
            desired_surge = reference.surge_mps or 0.0
            desired_sway = reference.sway_mps or 0.0

        if reference.depth_m is not None:
            depth_error = reference.depth_m - state.position[2]
            desired_heave = np.clip(g.depth_kp * depth_error,
                                    -g.max_vertical_mps, g.max_vertical_mps)
        else:
            desired_heave = 0.0

        # --- inner loop: velocity error -> force, PI on each axis ---
        error = np.array([desired_surge - state.velocity_body[0],
                          desired_sway - state.velocity_body[1],
                          desired_heave - state.velocity_body[2]])
        self._integral = np.clip(self._integral + error * dt,
                                 -g.integral_limit, g.integral_limit)

        fx = g.surge_kp * error[0] + g.surge_ki * self._integral[0]
        fy = g.sway_kp * error[1] + g.sway_ki * self._integral[1]
        fz = g.heave_kp * error[2] + g.heave_ki * self._integral[2]

        # --- heading: PD ---
        if reference.heading_rad is not None:
            heading_error = wrap(reference.heading_rad - state.yaw)
            mz = g.yaw_kp * heading_error - g.yaw_kd * state.yaw_rate
        else:
            mz = 0.0

        return Wrench(fx=float(fx), fy=float(fy), fz=float(fz), mz=float(mz))
