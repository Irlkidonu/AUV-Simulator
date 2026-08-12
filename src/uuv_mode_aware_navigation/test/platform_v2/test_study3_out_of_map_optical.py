"""Leaving the georeferenced texture must cost optical aiding, not the mission.

A vehicle that drifts off the surveyed patch physically has no georeferenced
imagery. That is loss of a modality -- exactly the condition the six-mode
selector exists to handle -- and not a programming error. Before the fix the
renderer raised out of ``run_one`` and destroyed the run.

These tests pin both halves of the contract: the out-of-map path yields an
unavailable optical observation and a completed mission, and every in-map render
behaves exactly as it did before.
"""
from __future__ import annotations

import math
from dataclasses import asdict

import numpy as np
import pytest

from uuv_mode_aware_navigation.rendering import (
    CameraPose, FootprintOutsideWorld, GeoreferencedRenderer, WorldTexture)
from uuv_mode_aware_navigation.optics import LIDAR, WaterState
from uuv_mode_aware_navigation.study3 import PolicyKind
from uuv_mode_aware_navigation.study3.interactive import (
    interactive_environment_config, run_interactive_session)

#: Matches the world ``run_one`` builds: 2048 px at 0.04 m/px, so +/- 40.94 m.
SIZE_PX, METRES_PER_PIXEL = 2048, .04
HALF_EXTENT = .5 * (SIZE_PX - 1) * METRES_PER_PIXEL


@pytest.fixture(scope="module")
def world():
    return WorldTexture.generate(SIZE_PX, METRES_PER_PIXEL, seed=22_000_101)


def test_out_of_map_error_is_still_a_value_error(world):
    """Backward compatibility: existing ``except ValueError`` handlers must work."""
    assert issubclass(FootprintOutsideWorld, ValueError)
    with pytest.raises(ValueError):
        world.sample(np.array([HALF_EXTENT + 5.0]), np.array([0.0]))


def test_out_of_map_error_is_distinguishable(world):
    """It must be catchable *specifically*, so real errors are not swallowed."""
    with pytest.raises(FootprintOutsideWorld):
        world.sample(np.array([HALF_EXTENT + 5.0]), np.array([0.0]))


def test_covers_agrees_with_sample(world):
    """``covers`` is the non-raising form of the same predicate."""
    for x, y in ((0., 0.), (10., -10.), (HALF_EXTENT - .5, 0.),
                 (HALF_EXTENT + .5, 0.), (0., -HALF_EXTENT - 3.), (200., 200.)):
        inside = world.covers(np.array([x]), np.array([y]))
        if inside:
            world.sample(np.array([x]), np.array([y]))
        else:
            with pytest.raises(FootprintOutsideWorld):
                world.sample(np.array([x]), np.array([y]))


def test_a_non_positive_altitude_is_not_an_out_of_map_condition(world):
    """A bad altitude is a programming error and must not be reclassified."""
    renderer = GeoreferencedRenderer(world, add_sensor_noise=False)
    with pytest.raises(ValueError) as raised:
        renderer.clear_scene(CameraPose(0., 0., 0., 0.))
    assert not isinstance(raised.value, FootprintOutsideWorld)


def test_renderer_raises_the_specific_type_off_the_patch(world):
    renderer = GeoreferencedRenderer(world, add_sensor_noise=False)
    pose = CameraPose(HALF_EXTENT + 20.0, 0., 5., 0.)
    with pytest.raises(FootprintOutsideWorld):
        renderer.render(pose, WaterState.from_turbidity(.1), LIDAR)


def _drifting_session(policy="reactive", current=.45, horizon_s=300., dt_s=2.):
    """Interactive session with a current strong enough to leave the patch."""
    record = {
        "schema": "study3_interactive_disturbance_v1", "policy_recorded": policy,
        "root": 34_900_000, "index": 0, "seed": 34_900_000,
        "dt_s": dt_s, "horizon_s": horizon_s,
        "base_config": asdict(interactive_environment_config()),
        "service_catalogue": ["lbl", "usbl"],
        "events": [{"effective_step": 2, "simulation_time_s": 2 * dt_s,
                    "sequence": 0, "control": "current_east_mps", "value": current}],
    }
    samples, truth = [], []
    _, completion = run_interactive_session(
        policy_kind=PolicyKind(policy), horizon_s=horizon_s, dt_s=dt_s, pace=False,
        replay_record=record, on_telemetry=samples.append,
        on_environment=lambda e: setattr(e, "on_physical_state", truth.append))
    return completion, samples, truth


def test_drifting_off_the_patch_completes_the_mission():
    """The defect: this used to abort with FootprintOutsideWorld."""
    completion, samples, truth = _drifting_session()
    assert completion["status"] == "complete", completion.get("error")
    assert samples, "no telemetry produced"
    excursion = max(math.hypot(*s["position_xy"]) for s in truth)
    assert excursion > HALF_EXTENT, (
        f"test did not leave the patch (max |r| = {excursion:.1f} m)")


def test_optical_aiding_becomes_unavailable_off_the_patch():
    """Off the patch there is no map imagery, so optical must not be available."""
    _, samples, truth = _drifting_session()
    position = {round(s["time_s"], 3): math.hypot(*s["position_xy"]) for s in truth}
    far = [s for s in samples
           if position.get(round(s["time_s"], 3), 0.) > HALF_EXTENT + 10.0]
    assert far, "no telemetry well outside the patch"
    assert not any(s["optical_available"] for s in far), (
        "optical reported available with no georeferenced imagery beneath it")
    assert all(s["optical_quality"] == 0.0 for s in far), (
        "an absent image must not carry a non-zero quality")


def test_the_vehicle_keeps_navigating_on_other_modalities():
    """Losing the map is a modality loss; the selector should still run."""
    _, samples, _ = _drifting_session()
    modes = {s["navigation_mode"] for s in samples}
    assert modes - {"terminal_degraded"}, "no non-terminal mode was ever selected"
    assert all(s["mission_action"] != "surface_for_gps" for s in samples[:5])


def test_in_map_runs_are_bit_identical_to_the_pre_fix_implementation():
    """Regression guard: a fully in-map session must reproduce a fixed digest.

    DEPLOYMENT_FIXED is used because it never consults the mode selector's
    output, so this digest pins the *rendering* path alone and stays valid
    across the later selector correction. The value was captured from a
    worktree at the pre-fix commit.
    """
    record = {
        "schema": "study3_interactive_disturbance_v1", "policy_recorded": "deployment_fixed",
        "root": 34_900_100, "index": 0, "seed": 34_900_100,
        "dt_s": 2.0, "horizon_s": 120.0,
        "base_config": asdict(interactive_environment_config()),
        "service_catalogue": ["lbl", "usbl"],
        "events": [{"effective_step": 10, "simulation_time_s": 20.0, "sequence": 0,
                    "control": "turbidity", "value": .55}],
    }
    _, completion = run_interactive_session(
        policy_kind=PolicyKind.DEPLOYMENT_FIXED, horizon_s=120.0, dt_s=2.0, pace=False,
        replay_record=record)
    assert completion["status"] == "complete"
    assert completion["result"]["trace_digest"] == PRE_FIX_IN_MAP_DIGEST


#: Captured from a worktree at 3691f9c9, before both the out-of-map fix and the
#: selector correction. DEPLOYMENT_FIXED does not consult the selector, so this
#: pins the rendering path across both changes.
PRE_FIX_IN_MAP_DIGEST = "3406f1d490f196d98ed001b7d7da7efae9291465038fda563a58a8693e6b11ad"
