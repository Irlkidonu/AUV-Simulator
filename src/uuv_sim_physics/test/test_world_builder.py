"""The checked-in world must be exactly what the configuration generates.

The SDF is a build product kept in the tree so it can be reviewed and hashed.
That only works if it cannot be edited independently of the YAML it claims to
come from -- otherwise the "discoverable parameters" become documentation that
quietly disagrees with the world actually being simulated.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from uuv_sim_physics import world_builder


def test_checked_in_world_matches_regeneration() -> None:
    assert world_builder.WORLD_PATH.read_text() == world_builder.build_world_sdf(), (
        "worlds/auv_sim_physics_base.sdf differs from what config/ generates. "
        "Edit the YAML and re-run `python3 -m uuv_sim_physics.world_builder`.")


def test_generation_is_deterministic() -> None:
    assert world_builder.build_world_sdf() == world_builder.build_world_sdf()


def test_world_is_well_formed_xml_with_one_world() -> None:
    root = ET.fromstring(world_builder.WORLD_PATH.read_text())
    assert root.tag == "sdf" and root.get("version") == "1.9"
    worlds = root.findall("world")
    assert len(worlds) == 1
    assert worlds[0].get("name") == world_builder.WORLD_NAME


def test_expected_models_present_and_cosmetic_ones_absent() -> None:
    root = ET.fromstring(world_builder.WORLD_PATH.read_text())
    names = {model.get("name") for model in root.findall("world/model")}

    assert {"seabed", "docking_station", "bluerov2_phys"} <= names
    assert {f"rock_{s}" for s in "abcde"} <= names
    # Scope reductions declared in physics.yaml -- asserted so the reduction is
    # deliberate rather than accidental.
    assert "dock_grabber" not in names
    assert "marine_snow" not in names


def test_solver_and_gravity_are_explicit() -> None:
    root = ET.fromstring(world_builder.WORLD_PATH.read_text())
    physics = root.find("world/physics")
    assert physics.get("type") == "dart"
    assert float(physics.find("max_step_size").text) == 0.001
    assert float(physics.find("real_time_factor").text) == 1.0
    gravity = [float(v) for v in root.find("world/gravity").text.split()]
    assert gravity == [0.0, 0.0, -9.8]


def test_required_systems_are_declared() -> None:
    root = ET.fromstring(world_builder.WORLD_PATH.read_text())
    declared = {p.get("filename") for p in root.findall("world/plugin")}
    assert {"gz-sim-physics-system", "gz-sim-user-commands-system",
            "gz-sim-scene-broadcaster-system", "gz-sim-sensors-system",
            "gz-sim-buoyancy-system"} <= declared

    vehicle = next(m for m in root.findall("world/model")
                   if m.get("name") == "bluerov2_phys")
    vehicle_plugins = [p.get("filename") for p in vehicle.findall("plugin")]
    assert vehicle_plugins.count("gz-sim-thruster-system") == 4
    assert vehicle_plugins.count("gz-sim-hydrodynamics-system") == 1


def test_contact_geometry_exists_where_expected() -> None:
    """Seabed, hull and rocks are tangible. The dock, per the reference, is not."""
    root = ET.fromstring(world_builder.WORLD_PATH.read_text())
    with_collision = {model.get("name") for model in root.findall("world/model")
                      if model.findall(".//collision")}
    assert "seabed" in with_collision
    assert "bluerov2_phys" in with_collision
    assert {f"rock_{s}" for s in "abcde"} <= with_collision
    assert "docking_station" not in with_collision      # inherited; M2.5 decides


def test_config_digest_changes_when_config_changes(tmp_path) -> None:
    before = world_builder.config_digest()
    target = world_builder.CONFIG_DIR / "physics.yaml"
    original = target.read_bytes()
    try:
        target.write_bytes(original + b"\n# probe\n")
        assert world_builder.config_digest() != before
    finally:
        target.write_bytes(original)
    assert world_builder.config_digest() == before
