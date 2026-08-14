"""The physics package must reproduce the reference model, not reinterpret it.

Each value in ``config/`` is compared against the value parsed out of the frozen
reference world. Exact equality, no tolerances: these are transcriptions, and a
transcription is either right or wrong.

This is the test that gives M2.5 its meaning. If the parameters had drifted
during copying, every validation result afterwards would be measuring a model
nobody had specified.

The reference is read read-only and its hash is checked first, so a change on
that side is reported as a changed reference rather than as a copy error.
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from uuv_sim_physics import world_builder

#: The frozen reference world lives in a DIFFERENT repository
#: (``uuv_adaptive_nav``), deliberately: this package must not vendor a copy of
#: the thing it claims to have transcribed faithfully. So on the author's
#: workspace these tests verify the transcription, and on a clean clone -- where
#: that repository is absent -- they skip with a reason rather than hard-fail.
#: The transcription is still pinned there by the recorded SHA-256.
REFERENCE = Path(
    os.environ.get("UUV_REFERENCE_WORLD",
                   "/home/chris/uuv_ws/src/uuv_adaptive_nav/worlds/"
                   "underwater_docking_physics.sdf"))

pytestmark = pytest.mark.skipif(
    not REFERENCE.is_file(),
    reason=f"frozen reference world not present at {REFERENCE}; set "
           "UUV_REFERENCE_WORLD to run the transcription-fidelity checks")


@pytest.fixture(scope="module")
def config() -> dict:
    return world_builder.load_config()


@pytest.fixture(scope="module")
def reference_root() -> ET.Element:
    return ET.parse(REFERENCE).getroot()


@pytest.fixture(scope="module")
def reference_vehicle(reference_root) -> ET.Element:
    world = reference_root.find("world")
    for model in world.findall("model"):
        if model.get("name") == "bluerov2_phys":
            return model
    raise AssertionError("bluerov2_phys not found in the reference world")


def test_reference_is_unmodified(config) -> None:
    """The recorded hash must still match, or the comparison means nothing."""
    import hashlib
    actual = hashlib.sha256(REFERENCE.read_bytes()).hexdigest()
    assert actual == config["physics"]["reference"]["sha256"], (
        "the frozen reference changed; re-derive the copy before trusting it")


# --- solver ------------------------------------------------------------------

def test_solver_matches(config, reference_root) -> None:
    physics = reference_root.find("world/physics")
    engine = config["physics"]["engine"]
    assert physics.get("type") == engine["type"]
    assert physics.get("name") == engine["profile_name"]
    assert float(physics.find("max_step_size").text) == engine["max_step_size_s"]
    assert float(physics.find("real_time_factor").text) == engine["real_time_factor"]


def test_fluid_density_matches(config, reference_root) -> None:
    for plugin in reference_root.findall("world/plugin"):
        if "buoyancy" in plugin.get("filename", ""):
            density = float(plugin.find("uniform_fluid_density").text)
            assert density == config["physics"]["fluid"]["density_kgm3"]
            return
    raise AssertionError("buoyancy plugin not found in the reference")


# --- mass properties ---------------------------------------------------------

def test_mass_inertia_and_com_match(config, reference_vehicle) -> None:
    inertial = reference_vehicle.find("link[@name='base_link']/inertial")
    base = config["vehicle_bluerov2_phys"]["base_link"]

    assert float(inertial.find("mass").text) == base["mass_kg"]

    com = [float(v) for v in inertial.find("pose").text.split()]
    assert com[:3] == list(base["center_of_mass_m"])
    assert com[3:] == [0.0, 0.0, 0.0], "reference CoM pose carries a rotation"

    inertia = inertial.find("inertia")
    for key, value in base["inertia_kgm2"].items():
        assert float(inertia.find(key).text) == value, key


def test_collision_box_matches(config, reference_vehicle) -> None:
    collision = reference_vehicle.find("link[@name='base_link']/collision")
    base = config["vehicle_bluerov2_phys"]["base_link"]
    assert collision.get("name") == base["collision"]["name"]
    size = [float(v) for v in collision.find("geometry/box/size").text.split()]
    assert size == list(base["collision"]["size_m"])


def test_derived_buoyancy_is_arithmetically_consistent(config) -> None:
    """The recorded derivation must follow from the recorded inputs."""
    vehicle = config["vehicle_bluerov2_phys"]
    box = vehicle["base_link"]["collision"]["size_m"]
    density = config["physics"]["fluid"]["density_kgm3"]
    derived = vehicle["derived_buoyancy"]

    volume = box[0] * box[1] * box[2]
    assert volume == pytest.approx(derived["displaced_volume_m3"], abs=1e-9)
    assert volume * density == pytest.approx(derived["displaced_mass_kg"], abs=1e-3)

    bg = derived["center_of_buoyancy_m"][2] - vehicle["base_link"]["center_of_mass_m"][2]
    assert bg == pytest.approx(derived["bg_separation_m"], abs=1e-9)


# --- hydrodynamics -----------------------------------------------------------

def test_every_hydrodynamic_coefficient_matches(config, reference_vehicle) -> None:
    for plugin in reference_vehicle.findall("plugin"):
        if "hydrodynamics" not in plugin.get("filename", ""):
            continue
        hydro = config["vehicle_bluerov2_phys"]["hydrodynamics"]
        assert plugin.find("link_name").text == hydro["link_name"]

        recorded = {**hydro["added_mass"], **hydro["linear_damping"],
                    **hydro["quadratic_damping"]}
        assert len(recorded) == 18, "expected 18 coefficients"
        for key, value in recorded.items():
            element = plugin.find(key)
            assert element is not None, f"{key} absent from the reference"
            assert float(element.text) == value, key
        return
    raise AssertionError("hydrodynamics plugin not found")


def test_zero_damping_terms_are_reproduced_not_repaired(config) -> None:
    """The inherited zeros must survive the copy; M2 reproduces, M2.5 judges."""
    linear = config["vehicle_bluerov2_phys"]["hydrodynamics"]["linear_damping"]
    assert linear["yV"] == 0.0, "sway linear damping was silently corrected"
    assert linear["kP"] == 0.0, "roll linear damping was silently corrected"
    discrepancies = config["vehicle_bluerov2_phys"]["known_discrepancies"]
    assert "zero_linear_sway_and_roll_damping" in discrepancies


# --- actuation ---------------------------------------------------------------

def test_thruster_geometry_and_limits_match(config, reference_vehicle) -> None:
    thrusters = config["vehicle_bluerov2_phys"]["thrusters"]

    for unit in thrusters["units"]:
        link = reference_vehicle.find(f"link[@name='{unit['name']}']")
        assert link is not None, unit["name"]
        pose = [float(v) for v in link.find("pose").text.split()]
        assert pose[:3] == list(unit["position_m"]), unit["name"]

        joint = reference_vehicle.find(f"joint[@name='{unit['joint']}']")
        assert joint is not None, unit["joint"]
        assert joint.get("type") == "revolute"
        axis = [float(v) for v in joint.find("axis/xyz").text.split()]
        assert axis == [float(a) for a in unit["axis"]], unit["joint"]

    common = thrusters["common"]
    plugins = [p for p in reference_vehicle.findall("plugin")
               if "thruster" in p.get("filename", "")]
    assert len(plugins) == len(thrusters["units"]) == 4
    for plugin in plugins:
        assert float(plugin.find("thrust_coefficient").text) == common["thrust_coefficient"]
        assert float(plugin.find("propeller_diameter").text) == common["propeller_diameter_m"]
        assert float(plugin.find("max_thrust_cmd").text) == common["max_thrust_cmd_N"]
        assert float(plugin.find("min_thrust_cmd").text) == common["min_thrust_cmd_N"]
        assert plugin.find("namespace").text == common["namespace"]


def test_derived_authority_follows_from_the_coefficients(config) -> None:
    """Terminal velocities are solved from the copied drag, not asserted."""
    import math
    vehicle = config["vehicle_bluerov2_phys"]
    linear = vehicle["hydrodynamics"]["linear_damping"]
    quad = vehicle["hydrodynamics"]["quadratic_damping"]
    derived = vehicle["derived_authority"]
    limit = vehicle["thrusters"]["common"]["max_thrust_cmd_N"]

    def terminal(thrust, lin, sq):
        # |sq| v^2 + |lin| v - thrust = 0
        a, b = abs(sq), abs(lin)
        return (-b + math.sqrt(b * b + 4 * a * thrust)) / (2 * a)

    assert terminal(2 * limit, linear["xU"], quad["xUabsU"]) == \
        pytest.approx(derived["surge_terminal_mps"], abs=5e-3)
    assert terminal(limit, linear["yV"], quad["yVabsV"]) == \
        pytest.approx(derived["sway_terminal_mps"], abs=5e-3)
    assert terminal(limit, linear["zW"], quad["zWabsW"]) == \
        pytest.approx(derived["heave_terminal_mps"], abs=5e-3)

    # Roll free-decay period from the righting moment and the rolling inertia.
    mass = vehicle["base_link"]["mass_kg"]
    bg = vehicle["derived_buoyancy"]["bg_separation_m"]
    ixx = vehicle["base_link"]["inertia_kgm2"]["ixx"]
    added = abs(vehicle["hydrodynamics"]["added_mass"]["kDotP"])
    period = 2 * math.pi / math.sqrt(mass * 9.8 * bg / (ixx + added))
    assert period == pytest.approx(derived["roll_natural_period_s"], abs=5e-3)

    ratio = mass / (mass + abs(vehicle["hydrodynamics"]["added_mass"]["xDotU"]))
    assert ratio == pytest.approx(derived["added_mass_surge_ratio"], abs=5e-3)


# --- dock --------------------------------------------------------------------

def test_dock_pose_and_led_constellation_match(config, reference_root) -> None:
    dock = config["dock_station"]
    for model in reference_root.findall("world/model"):
        if model.get("name") != "docking_station":
            continue
        pose = [float(v) for v in model.find("pose").text.split()]
        assert pose == list(dock["pose"])

        leds = model.find("link[@name='led_markers']")
        reference_poses = sorted(
            tuple(round(float(v), 6) for v in visual.find("pose").text.split()[:3])
            for visual in leds.findall("visual"))
        recorded = sorted(tuple(round(float(v), 6) for v in led["pose"][:3])
                          for led in dock["links"]["led_markers"]["visuals"])
        assert recorded == reference_poses
        return
    raise AssertionError("docking_station not found")


def test_dock_absence_of_collision_is_recorded_not_silently_added(config,
                                                                 reference_root) -> None:
    """The dock is intangible in the reference; the copy must say so."""
    for model in reference_root.findall("world/model"):
        if model.get("name") == "docking_station":
            assert model.findall(".//collision") == [], (
                "the reference dock gained collision geometry")
    assert config["dock_station"]["collision"]["present"] is False
    assert "dock_has_no_collision" in config["dock_station"]["known_discrepancies"]

    generated = world_builder.WORLD_PATH.read_text()
    dock_block = re.search(r'<model name="docking_station">.*?</model>',
                           generated, re.S)
    assert dock_block is not None
    assert "<collision" not in dock_block.group(0), (
        "collision was added to the dock during M2; that is an M2.5 decision")
