"""The rendered scene must agree with the mission it is supposed to depict.

The demonstrator contributes no statistics, so nothing here can change a
reported number. What it can change is whether the figures and the video show
the system the paper describes. Two disagreements have already cost time:

  * the SDF spawned the vehicle at the world origin while the mission started
    at its first waypoint, and the demonstrator reported the 13.45 m offset as
    position error for the entire run; and
  * the survey area extended beyond the lit volume, so estimated optical
    quality read zero in places for a reason that had nothing to do with water.

Both are scene/mission agreements that a test can hold, and neither is visible
from reading either file alone. The SDF is parsed rather than pattern-matched
so that reformatting it cannot quietly disable the check.
"""

from __future__ import annotations

import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
WORLD = PACKAGE_ROOT / "worlds" / "mode_aware_survey.sdf"

sys.path.insert(0, str(PACKAGE_ROOT))

from uuv_mode_aware_navigation.mission import SurveyMission  # noqa: E402


def _root() -> ET.Element:
    return ET.parse(WORLD).getroot()


def _pose(element: ET.Element) -> list[float]:
    node = element.find("pose")
    assert node is not None and node.text, "element has no pose"
    return [float(v) for v in node.text.split()]


def _model(name: str) -> ET.Element:
    for model in _root().iter("model"):
        if model.get("name") == name:
            return model
    raise AssertionError(f"no model named {name!r} in {WORLD.name}")


def test_the_world_parses():
    """A malformed SDF fails at launch with an unhelpful message; fail here."""
    assert _root().tag == "sdf"


def test_vehicle_spawns_at_the_missions_first_waypoint():
    """The scene and the mission must not disagree about where the run starts."""
    expected = [float(v) for v in SurveyMission().waypoints[0]]
    actual = _pose(_model("bluerov2"))[:3]
    assert actual == pytest.approx(expected, abs=1e-6), (
        f"SDF spawns the vehicle at {actual} but the mission starts at "
        f"{expected}; the demonstrator would report the difference as position "
        f"error for the whole run"
    )


def test_launch_file_passes_the_spawn_position_from_the_mission():
    """The literal must not reappear in the launch file.

    The node carries a default so it can be run standalone, but the launch path
    has to source the value from the mission, or the three definitions can drift
    apart again without any single file looking wrong.
    """
    text = (PACKAGE_ROOT / "launch" / "demo.launch.py").read_text()
    assert "SurveyMission" in text
    assert "spawn_position_m" in text


def _static_lights() -> list[ET.Element]:
    """Lights belonging to the world rather than to the vehicle.

    Vehicle-mounted lamps travel with the camera and always illuminate whatever
    it is looking at, so they say nothing about coverage of the survey area.
    Only the fixed lights do.
    """
    world = _root().find("world")
    assert world is not None
    return [light for light in world.findall("light")]


def test_the_survey_area_is_inside_the_lit_volume():
    """Every waypoint must be lit by a fixed light, not only by the vehicle lamp.

    The check is coverage of the seabed under each waypoint by a point light's
    declared attenuation range, or by any directional light -- a directional
    light has no position and lights everything it faces. A waypoint outside
    every range renders black, and the optical-feedback node then reports zero
    quality for a scene reason rather than a water reason, which is precisely
    the confusion this test exists to prevent.
    """
    mission = SurveyMission()
    seabed_z = mission.seabed_depth_m

    directional = [
        light for light in _static_lights() if light.get("type") == "directional"
    ]
    point_lights = [
        (_pose(light)[:3], float(light.findtext("attenuation/range", "0")))
        for light in _static_lights() if light.get("type") in ("point", "spot")
    ]
    assert directional or point_lights, "the world has no fixed lighting at all"

    unlit = []
    for waypoint in mission.waypoints:
        target = (float(waypoint[0]), float(waypoint[1]), float(seabed_z))
        if directional:
            continue
        if not any(
            math.dist(pos, target) <= reach for pos, reach in point_lights
        ):
            unlit.append(target)
    assert not unlit, f"seabed under these waypoints is outside every light: {unlit}"


# The clearest water the demonstrator is launched with. Anything murkier
# attenuates more, so checking the scene's haze against this value checks it
# against the whole range.
CLEAREST_DEMO_TURBIDITY_PER_M = 0.2


def test_scene_fog_is_a_minor_term_beside_the_modelled_water():
    """Scene haze must not compete with the water model the paper is about.

    Gazebo's exp2 fog removes exp(-(density*range)^2); the water-column node
    then applies the two-way path exp(-2*c*range) on top. The scene fog exists
    to make the render read as underwater, not to attenuate -- if its optical
    depth were comparable to the water's, changing ``turbidity_c`` would move
    the estimated quality far less than the model says it should, and the
    demonstrator would understate the very lever it is there to show.

    The comparison is against the *clearest* water the demonstrator uses, which
    is the case where the scene's fixed haze has the best chance of dominating.
    """
    density = float(_root().findtext("world/scene/fog/density", "0"))
    altitude = SurveyMission().altitude_band_m[1]

    fog_optical_depth = (density * altitude) ** 2
    water_optical_depth = 2.0 * CLEAREST_DEMO_TURBIDITY_PER_M * altitude
    assert water_optical_depth > 0

    share = fog_optical_depth / water_optical_depth
    assert share < 0.25, (
        f"at {altitude} m the scene's fog contributes optical depth "
        f"{fog_optical_depth:.3f} against the water's {water_optical_depth:.3f} "
        f"({share:.0%}); the render would be haze-limited rather than "
        f"water-limited"
    )


def _weeds() -> list[ET.Element]:
    return [m for m in _root().iter("model")
            if (m.get("name") or "").startswith("weed_")]


def test_every_weed_is_planted_on_the_seabed():
    """The plants stand on the floor the altimeter reads, not near it.

    They were hoisted between 0.6 and 1.6 m above it, which had been
    compensating for meshes that hang below their own origin. Both halves of
    that are now fixed, and they have to stay fixed together: correcting one
    alone leaves the plants either buried or floating.
    """
    from uuv_mode_aware_navigation.seabed import depth_at

    assert _weeds(), "no weed models in the world"
    for weed in _weeds():
        x, y, z = _pose(weed)[:3]
        assert z == pytest.approx(depth_at(x, y), abs=0.01), (
            f"{weed.get('name')} sits at z={z:.2f} but the seabed under it is "
            f"at {depth_at(x, y):.2f}"
        )


def test_no_weed_dips_below_the_seabed_at_full_lean():
    """Nothing goes under the bedrock, however hard the current blows.

    A weed leans by rotating about its link origin, so the visual pose has to
    put the holdfast there and then lift the clump by the depth its lowest leaf
    would otherwise sweep to at the lean ceiling. Without the lift the plant
    swings through the sand, and the stronger the current the deeper it goes.
    """
    # Read the ceiling out of the source instead of importing the module that
    # declares it. fish_school imports rclpy, and every other test in this file
    # runs without ROS on the path; making one test the exception would mean the
    # suite quietly stopped being runnable on a machine that only wants to check
    # the geometry.
    source = (PACKAGE_ROOT / "uuv_mode_aware_navigation" / "nodes"
              / "fish_school.py").read_text()
    declared = re.search(r"^\s*MAX_LEAN\s*=\s*([0-9.]+)", source, re.M)
    assert declared, "Weed.MAX_LEAN is no longer declared as a literal"
    max_lean = float(declared.group(1))

    for weed in _weeds():
        visual = weed.find(".//visual")
        assert visual is not None, f"{weed.get('name')} has no visual"
        pose = _pose(visual)
        assert pose[2] > 0.0, (
            f"{weed.get('name')} has no lift on its visual, so it will sweep "
            f"below the seabed as soon as it leans"
        )
        # The lift only has to cover the arc the clump actually sweeps.
        assert pose[2] < 0.5, (
            f"{weed.get('name')} is lifted {pose[2]:.2f} m, which would leave "
            f"it visibly hovering above the sand"
        )
    assert max_lean == pytest.approx(0.7), (
        "the lifts above were computed at a 0.7 rad lean ceiling; changing it "
        "means recomputing them from the meshes"
    )


def test_the_plant_that_was_lying_down_is_stood_up():
    """plant_1 arrives flat and needs roll; the other two arrive upright.

    An earlier attempt baked a -90 degree X rotation into plant_1 to make it
    match the others. It rotated the wrong way and laid it on the sand, and
    since eight of the twenty-four weeds use that mesh, exactly a third of them
    were flat.
    """
    seen = {"plant_1": 0, "plant_2": 0, "plant_3": 0}
    for weed in _weeds():
        uri = weed.find(".//uri")
        assert uri is not None and uri.text
        mesh = next(k for k in seen if k in uri.text)
        seen[mesh] += 1
        roll = _pose(weed.find(".//visual"))[3]
        expected = math.pi / 2 if mesh == "plant_1" else 0.0
        assert roll == pytest.approx(expected, abs=1e-3), (
            f"{weed.get('name')} uses {mesh} with roll {roll:.3f}, expected "
            f"{expected:.3f}"
        )
    assert all(v > 0 for v in seen.values()), f"not all meshes used: {seen}"
