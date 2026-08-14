"""The equivalence test must be able to fail. These are its teeth.

A bitwise comparison that never fires proves nothing, so each exposed member is
perturbed by one unit in the last place -- the smallest difference float64 can
represent -- and the comparison from ``test_reduced_equivalence`` is required to
catch it. If a future refactor makes the comparison blind, these fail first.

They also pin an aliasing property of the frozen ``Vehicle`` that the wrapper
must keep reproducing; see ``test_current_is_aliased_by_the_frozen_vehicle``.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uuv_mode_aware_navigation.mission import Vehicle
from uuv_sim_physics import ReducedBackend

from test_reduced_equivalence import SEEDS, _bits, _conditions, _state

MEMBERS = ("position", "velocity", "path_length_m", "current")


def _perturb(vehicle: Vehicle, member: str) -> None:
    """Advance one component of ``member`` by exactly one ULP."""
    up = lambda x: math.nextafter(x, math.inf)   # noqa: E731
    if member == "path_length_m":
        vehicle.path_length_m = up(vehicle.path_length_m)
    else:
        array = getattr(vehicle, member)
        array[0] = up(array[0])


@pytest.mark.parametrize("member", MEMBERS)
def test_a_one_ulp_difference_is_detected(member: str) -> None:
    start, current, dt, commands = _conditions(SEEDS[0])

    # Independent current arrays: the frozen constructor aliases the one it is
    # given, so sharing it would perturb both sides and hide the difference.
    reference = Vehicle(start, current.copy())
    mutated = ReducedBackend(start, current.copy())

    for command in commands[:200]:
        reference.step(command, dt)
        mutated.step(command, dt)

    _perturb(mutated.vehicle, member)

    detected = _state(mutated) != _state(reference)
    for command in commands[200:260]:
        expected = reference.step(command, dt)
        actual = mutated.step(command, dt)
        detected = detected or _bits(actual) != _bits(expected) \
            or _state(mutated) != _state(reference)

    assert detected, f"a one-ULP change in {member} went undetected"


def test_the_unmutated_wrapper_survives_the_same_comparison() -> None:
    """The counterpart: the checks above are strict, not merely noisy."""
    start, current, dt, commands = _conditions(SEEDS[0])
    reference = Vehicle(start, current.copy())
    backend = ReducedBackend(start, current.copy())

    for command in commands[:2_000]:
        assert _bits(backend.step(command, dt)) == _bits(reference.step(command, dt))
        assert _state(backend) == _state(reference)


def test_current_is_aliased_by_the_frozen_vehicle() -> None:
    """Documents an existing property; the wrapper must inherit, not repair it.

    ``Vehicle.__init__`` copies ``position`` but stores ``current`` by
    reference, so two vehicles built from one array share it. That is a sharp
    edge in code we may not modify. The wrapper must reproduce it exactly --
    copying defensively would make ``ReducedBackend`` behave *differently* from
    the model it claims to be transparent to, which is the one thing it must
    not do.

    If this test ever fails, ``Vehicle`` changed. That is a signal to re-derive
    the equivalence contract, not to adjust this file.
    """
    current = np.array([0.05, -0.02, 0.0])
    reference = Vehicle((0.0, 0.0, -5.0), current)
    backend = ReducedBackend((0.0, 0.0, -5.0), current)

    assert reference.current is current, "Vehicle no longer aliases current"
    assert backend.current is current, "wrapper stopped inheriting the aliasing"
    assert backend.position is not reference.position, "position must be copied"
