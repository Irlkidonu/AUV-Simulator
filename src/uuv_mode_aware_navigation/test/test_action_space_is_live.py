"""Every axis of the action space must actually do something.

These tests exist because a full suite of 138 tests passed while a third of the
manager's action space was inert. Commanded altitude had no effect whatsoever:
line-of-sight guidance built its direction between two waypoints at equal depth,
so the vertical component of the commanded velocity was identically zero and the
vehicle could never climb or descend. Configurations at 1.0 m, 2.0 m and 3.0 m
produced bit-identical outcomes on every scenario and every channel.

Nothing caught it, because the tests checked which configuration was *selected*
and never whether it was *carried out*. A manager can only be evaluated on
actions the vehicle can actually take, so each axis gets a test that fails if the
axis goes dead.
"""

import numpy as np
import pytest

from uuv_mode_aware_navigation.campaign import (
    DEVELOPMENT_SEED_ROOT,
    CurrentProfile,
    Scenario,
    WaterProfile,
    run_scenario,
)
from uuv_mode_aware_navigation.comparators import FixedPolicy
from uuv_mode_aware_navigation.manager import VehicleConfiguration
from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS, LIDAR
from uuv_mode_aware_navigation.sensors import FaultSchedule


def _scenario():
    return Scenario(
        "live", DEVELOPMENT_SEED_ROOT + 1000, WaterProfile.constant(0.20),
        FaultSchedule(), current=CurrentProfile.constant((0.02, -0.01, 0.0)),
    )


@pytest.mark.parametrize("altitude", [1.0, 2.0, 3.0])
def test_commanded_altitude_is_actually_flown(altitude):
    """The vehicle must reach the height it was told to hold.

    Altitude is the exponential lever in this study: light makes a two-way trip
    of roughly twice the altitude, so ``2h`` sits inside the exponent of the
    attenuation. An altitude command that does not move the vehicle removes the
    strongest action the manager has, and does so silently.
    """
    config = VehicleConfiguration(CAMERA_OFFAXIS, altitude, 0.5)
    outcome = run_scenario(_scenario(), FixedPolicy(config)).outcome
    assert abs(outcome.mean_altitude_m - altitude) < 0.25, (
        f"commanded {altitude} m, flew {outcome.mean_altitude_m:.2f} m"
    )


def test_different_altitudes_give_different_outcomes():
    """The direct regression: three altitudes must not be interchangeable."""
    results = [
        run_scenario(
            _scenario(),
            FixedPolicy(VehicleConfiguration(CAMERA_OFFAXIS, altitude, 0.5)),
        ).outcome
        for altitude in (1.0, 2.0, 3.0)
    ]
    altitudes = [round(o.mean_altitude_m, 3) for o in results]
    assert len(set(altitudes)) == 3, f"altitude axis is inert: {altitudes}"


def test_different_speeds_give_different_outcomes():
    elapsed = [
        run_scenario(
            _scenario(),
            FixedPolicy(VehicleConfiguration(CAMERA_OFFAXIS, 3.0, speed)),
        ).outcome.elapsed_s
        for speed in (0.25, 0.50)
    ]
    assert elapsed[0] > elapsed[1] * 1.5, f"speed axis is inert: {elapsed}"


def test_different_optical_channels_give_different_outcomes():
    """Turbid water, where the channels genuinely differ."""
    scenario = Scenario(
        "channels", DEVELOPMENT_SEED_ROOT + 1001, WaterProfile.constant(1.20),
        FaultSchedule(),
    )
    outcomes = {
        channel.name: run_scenario(
            scenario, FixedPolicy(VehicleConfiguration(channel, 2.0, 0.5))
        ).outcome.aiding_availability
        for channel in (CAMERA_OFFAXIS, LIDAR)
    }
    assert len(set(round(v, 6) for v in outcomes.values())) > 1, (
        f"optical channel axis is inert: {outcomes}"
    )


def test_vertical_command_is_nonzero_when_altitude_differs_from_current():
    """The specific mechanism that failed, tested directly at the guidance law."""
    from uuv_mode_aware_navigation.mission import Guidance, SurveyMission

    mission = SurveyMission()
    guidance = Guidance(mission)
    guidance.index = 1
    # Sitting 3 m above the commanded height.
    position = mission.waypoints[0].copy()
    position[2] += 3.0
    command = guidance.command(position, 0.5, 3.0)
    assert abs(command[2]) > 0.01, (
        f"guidance issued no vertical command while 3 m off altitude: {command}"
    )
    assert command[2] < 0.0, "commanded upward while needing to descend"


def test_fusion_axis_is_live_and_differs_under_an_outlier():
    """Gate and weight must not be interchangeable.

    A one-sided multipath outlier is the case that separates them: a hard gate
    refuses it entirely, while covariance weighting admits it at reduced weight
    and is dragged. If both strategies produced the same estimate the axis would
    be decorative, and the paper's claim that the vehicle selects a fusion
    strategy would be describing something with no effect.
    """
    import numpy as np

    from uuv_mode_aware_navigation.estimator import FusionMode, NavigationFilter

    moved = {}
    for mode in (FusionMode.GATE, FusionMode.WEIGHT):
        filt = NavigationFilter()
        filt.fusion = mode
        filt.predict(np.array([0.0, 0.0, 9.81]), 0.1)
        before = filt.position.copy()
        # A 20 m surface-bounce style error: large, and in one direction only.
        filt.update_position(before + np.array([20.0, 0.0, 0.0]), 0.10)
        moved[mode] = float(np.linalg.norm(filt.position - before))

    assert moved[FusionMode.GATE] < 0.01, (
        f"the gate admitted a 20 m outlier: moved {moved[FusionMode.GATE]:.3f} m"
    )
    assert moved[FusionMode.WEIGHT] > 1.0, (
        f"weighting rejected outright, so it is not weighting: "
        f"moved {moved[FusionMode.WEIGHT]:.3f} m"
    )


def test_every_action_axis_appears_in_the_candidate_set():
    """All four axes must vary across the declared action space."""
    from uuv_mode_aware_navigation.manager import DEFAULT_CANDIDATES

    assert len({c.optical.name for c in DEFAULT_CANDIDATES}) == 3
    assert len({c.altitude_m for c in DEFAULT_CANDIDATES}) == 3
    assert len({c.speed_mps for c in DEFAULT_CANDIDATES}) == 2
    # Four: single beacon, LBL, USBL and terrain-relative. USBL is selectable
    # because a support vessel is present in most families; E18 removes it by
    # fault rather than by omission, which is how a deployment dependency
    # should be modelled.
    assert len({c.acoustic.name for c in DEFAULT_CANDIDATES}) == 4
    assert {"usbl", "terrain_relative"} <= {c.acoustic.name for c in DEFAULT_CANDIDATES}
    # The three dependency classes must all be represented, or a silent
    # transponder leaves the vehicle with nothing structurally different to
    # switch to, which is the whole point of carrying more than one technique.
    assert {c.acoustic.infrastructure for c in DEFAULT_CANDIDATES} == {
        "surface", "seabed_array", "none",
    }
    assert len({c.fusion for c in DEFAULT_CANDIDATES}) == 2
    # 3 optical x 3 altitudes x 2 speeds x 4 acoustic x 2 fusion.
    assert len(DEFAULT_CANDIDATES) == 144
    # Names must be unique, or the sweep would silently collapse configurations.
    assert len({c.name for c in DEFAULT_CANDIDATES}) == len(DEFAULT_CANDIDATES)
