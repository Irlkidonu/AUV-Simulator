"""The existing reduced dynamics, presented through the backend protocol.

``ReducedBackend`` holds a ``mission.Vehicle`` and forwards to it. It does not
reimplement the motion model, does not pre- or post-process the command, and
does not copy the state it exposes -- it reads straight through to the instance
it owns.

That is the whole design. Every arithmetic operation still happens inside the
frozen ``Vehicle.step``, so the wrapper cannot change a result: there is no
floating-point work here to change one with. The equivalence test proves this
rather than assuming it, over 20 seeds x 10,000 steps, comparing raw bytes.

Why composition and not a subclass
----------------------------------
Subclassing would tie the wrapper to ``Vehicle``'s internal layout, and a
future ``GazeboBackend`` has no such layout to inherit. Composition keeps both
backends answering to the protocol and to nothing else, and it leaves the
existing class untouched -- which is the architectural requirement here, not a
stylistic preference.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from uuv_mode_aware_navigation.mission import Vehicle

__all__ = ["ReducedBackend"]


class ReducedBackend:
    """Deterministic reduced-order backend: a thin shell over ``Vehicle``."""

    #: What this backend can and cannot claim to model. Read by the run logger
    #: so that a result file states its own dynamics provenance instead of
    #: relying on the reader to remember which mode produced it.
    BACKEND_NAME = "reduced"
    MODELS = ("first_order_thruster_response", "advection_by_current")
    DOES_NOT_MODEL = ("rigid_body_6dof", "buoyancy", "added_mass",
                      "hydrodynamic_damping", "thruster_allocation",
                      "contact", "attitude")

    def __init__(self, position: Sequence[float],
                 current_mps: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        self._vehicle = Vehicle(position, current_mps)

    # -- protocol state ---------------------------------------------------
    # Properties, not attributes assigned in __init__: ``Vehicle`` rebinds
    # ``position`` and ``velocity`` to new arrays on every step, so a cached
    # reference would silently freeze at the initial condition.

    @property
    def position(self) -> np.ndarray:
        return self._vehicle.position

    @property
    def velocity(self) -> np.ndarray:
        return self._vehicle.velocity

    @property
    def current(self) -> np.ndarray:
        return self._vehicle.current

    @property
    def path_length_m(self) -> float:
        return self._vehicle.path_length_m

    # -- protocol behaviour -----------------------------------------------

    def step(self, commanded_velocity: Sequence[float], dt: float) -> np.ndarray:
        return self._vehicle.step(commanded_velocity, dt)

    def reset(self, position: Sequence[float],
              current_mps: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        # A fresh instance rather than field assignment: it is the same
        # construction path the caller would take, so no state can survive a
        # reset by having been overlooked here.
        self._vehicle = Vehicle(position, current_mps)

    # -- escape hatch ------------------------------------------------------

    @property
    def vehicle(self) -> Vehicle:
        """The wrapped instance, for callers that still need it directly.

        Present so that adopting the backend is never a reason to change an
        existing call site. Nothing in this package uses it.
        """
        return self._vehicle

    def __repr__(self) -> str:
        return (f"ReducedBackend(position={self.position.tolist()}, "
                f"current={self.current.tolist()})")
