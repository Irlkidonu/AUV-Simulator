"""``ReducedBackend`` must be indistinguishable from a bare ``mission.Vehicle``.

Bit-for-bit, not to a tolerance. The wrapper performs no arithmetic of its own,
so any divergence would mean it had changed the model rather than forwarded to
it -- and a tolerance would hide exactly the class of defect this test exists to
catch. Comparisons are on ``ndarray.tobytes()`` and ``struct.pack`` so that
signed zero and the last mantissa bit both count.

Scope: 20 seeds x 10,000 steps, every exposed state member checked at every
step, not just at the end. An error that cancels out by the final position is
still an error.
"""

from __future__ import annotations

import struct

import numpy as np
import pytest

from uuv_mode_aware_navigation.mission import Vehicle
from uuv_sim_physics import DynamicsBackend, ReducedBackend

SEEDS = tuple(range(41_000_000, 41_000_020))   # 20 seeds
STEPS = 10_000


def _bits(value) -> bytes:
    """Raw bytes of a float or array -- the strictest available comparison."""
    if isinstance(value, np.ndarray):
        assert value.dtype == np.float64, f"unexpected dtype {value.dtype}"
        return value.tobytes()
    return struct.pack("<d", float(value))


def _state(obj) -> tuple[bytes, ...]:
    return (_bits(obj.position), _bits(obj.velocity),
            _bits(obj.current), _bits(obj.path_length_m))


def _conditions(seed: int):
    """An initial condition and a command stream, both seed-determined.

    The command stream is deliberately awkward: sign changes, a step, a hold at
    zero and occasional large excursions. A wrapper defect that only shows up
    on reversal or on a zero command would survive a smooth ramp.
    """
    rng = np.random.default_rng(seed)
    start = rng.uniform(-40.0, 40.0, 3)
    current = rng.uniform(-0.4, 0.4, 3)
    dt = float(rng.uniform(0.05, 2.5))
    commands = rng.normal(0.0, 0.6, (STEPS, 3))
    commands[STEPS // 4:STEPS // 4 + 50] = 0.0            # hold at zero
    commands[STEPS // 2:] *= -1.0                          # reversal
    commands[::997] *= 25.0                                # excursions
    return start, current, dt, commands


@pytest.mark.parametrize("seed", SEEDS)
def test_backend_is_bitwise_identical_to_bare_vehicle(seed: int) -> None:
    start, current, dt, commands = _conditions(seed)

    reference = Vehicle(start, current)
    backend = ReducedBackend(start, current)

    assert _state(backend) == _state(reference), "initial state differs"

    for index, command in enumerate(commands):
        expected = reference.step(command, dt)
        actual = backend.step(command, dt)

        assert _bits(actual) == _bits(expected), (
            f"seed {seed} step {index}: acceleration differs")
        assert _state(backend) == _state(reference), (
            f"seed {seed} step {index}: state differs")


def test_reset_restores_a_freshly_constructed_instance() -> None:
    start, current, dt, commands = _conditions(SEEDS[0])

    backend = ReducedBackend(start, current)
    for command in commands[:500]:
        backend.step(command, dt)
    assert backend.path_length_m > 0.0

    later_start = np.array([1.5, -2.5, -7.0])
    later_current = np.array([0.05, -0.02, 0.0])
    backend.reset(later_start, later_current)

    assert _state(backend) == _state(Vehicle(later_start, later_current))
    assert backend.path_length_m == 0.0, "odometry survived a reset"


def test_reset_leaves_no_trajectory_memory() -> None:
    """A reset instance must re-run a trajectory exactly as a new one does."""
    start, current, dt, commands = _conditions(SEEDS[1])

    fresh = ReducedBackend(start, current)
    reused = ReducedBackend(start, current)
    for command in commands[:2_000]:
        reused.step(command, dt)
    reused.reset(start, current)

    for index, command in enumerate(commands[:2_000]):
        assert _bits(reused.step(command, dt)) == _bits(fresh.step(command, dt)), (
            f"step {index} differs after reset")
        assert _state(reused) == _state(fresh)


def test_exposed_state_reads_through_rather_than_caching() -> None:
    """``Vehicle`` rebinds its arrays each step; the wrapper must not cache."""
    start, current, dt, commands = _conditions(SEEDS[2])
    backend = ReducedBackend(start, current)

    before = backend.position
    backend.step(commands[0], dt)
    after = backend.position

    assert after is not before, "position looks cached, not read through"
    assert _bits(after) == _bits(backend.vehicle.position)


def test_conforms_to_the_protocol() -> None:
    backend = ReducedBackend((0.0, 0.0, -5.0))
    assert isinstance(backend, DynamicsBackend)
    for member in ("position", "velocity", "current", "path_length_m",
                   "step", "reset"):
        assert hasattr(backend, member), member


def test_declares_what_it_does_not_model() -> None:
    """The reduced backend must not be mistakable for the physics one."""
    assert ReducedBackend.BACKEND_NAME == "reduced"
    for absent in ("rigid_body_6dof", "buoyancy", "added_mass",
                   "hydrodynamic_damping", "contact"):
        assert absent in ReducedBackend.DOES_NOT_MODEL
