"""The dynamics-backend contract shared by every AUV Simulator execution mode.

This module is the seam between the simulator's *models* -- guidance, sensing,
optics, estimation, mode selection -- and whatever integrates the vehicle's
motion underneath them. Two backends are planned:

* ``ReducedBackend``  wraps the existing deterministic ``mission.Vehicle``;
* ``GazeboBackend``   (M3) integrates rigid-body dynamics in Gazebo/DART.

Deliberately narrow. The protocol carries only the state a caller must have in
order to advance and observe the vehicle, because every member added here is a
member a physics backend has to reproduce faithfully. Sensing, estimation and
policy are *not* part of it: they consume backend state, they do not extend it.

Import discipline
-----------------
Pure Python and NumPy. No ROS, no Gazebo, not even transitively. The reduced
path must stay usable in the headless install, which has neither, so anything
that needs them belongs in a module a caller imports explicitly.
"""

from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

import numpy as np

__all__ = ["DynamicsBackend"]


@runtime_checkable
class DynamicsBackend(Protocol):
    """Advance a vehicle through commanded water-relative velocity.

    The signature follows the existing ``mission.Vehicle`` exactly, so that the
    reduced backend can be a transparent wrapper rather than a translation
    layer. A translation layer would have to be trusted; a delegation can be
    proved bit-for-bit, which is what ``test_reduced_equivalence.py`` does.
    """

    @property
    def position(self) -> np.ndarray:
        """True position in world coordinates, metres. Shape (3,), float64."""

    @property
    def velocity(self) -> np.ndarray:
        """True ground velocity, m/s. Shape (3,), float64.

        Ground, not water-relative: it already includes the current. A vehicle
        that does not compensate for the current tracks off its line, and that
        error has to be observable in the state the backend reports.
        """

    @property
    def current(self) -> np.ndarray:
        """Water current acting on the vehicle, m/s. Shape (3,), float64."""

    @property
    def path_length_m(self) -> float:
        """Cumulative distance travelled, metres.

        Part of the contract because it is *integrated* state: a backend cannot
        reconstruct it after the fact from sampled positions without losing the
        sub-step path, so it has to be carried forward step by step.
        """

    def step(self, commanded_velocity: Sequence[float], dt: float) -> np.ndarray:
        """Advance by ``dt`` seconds; return the true acceleration over it.

        ``commanded_velocity`` is water-relative -- what the thrusters produce
        against the surrounding fluid, not a ground-frame target.

        The return value is the realised acceleration, not the commanded one.
        For an inertial sensor model that distinction is the whole point.
        """

    def reset(self, position: Sequence[float],
              current_mps: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        """Return to an initial condition, discarding all integrated state.

        After ``reset`` the backend must be indistinguishable from a freshly
        constructed one, ``path_length_m`` included. A backend that reset its
        pose but kept its odometry would silently corrupt any campaign that
        reuses one instance across runs.
        """
