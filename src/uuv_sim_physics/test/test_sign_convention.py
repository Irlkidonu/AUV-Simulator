"""The frozen thruster sign convention. No controller should rediscover this.

Both halves are asserted: the allocation map is checked statically (fast, always
runs), and the resulting motion is checked against the simulator (slow, marked).

The static half is the one that matters day to day -- it pins the yaw
differential, which was wrong on the first validated run and produced a vehicle
that turned the wrong way under a positive yaw command.
"""

from __future__ import annotations

import math

import pytest

from uuv_sim_physics import world_builder
from uuv_sim_physics.validation import protocol

# --- the convention, stated once ------------------------------------------
#
#   body +X forward, +Y port, +Z up; yaw positive about +Z (bow to port).
#
#   thruster      position (m)        positive thrust acts along
#   prop_left     (-0.20, +0.16, 0)   body +X
#   prop_right    (-0.20, -0.16, 0)   body +X
#   prop_sway     ( 0.00,  0.00, -0.02) body +Y
#   prop_vert     ( 0.00,  0.00, +0.10) body +Z
#
#   surge F -> both surge units +F/2            -> body +X force
#   yaw   M -> left -M/2, right +M/2            -> body +Z moment (bow to port)
#   sway  F -> prop_sway +F                     -> body +Y force
#   heave F -> prop_vert +F                     -> body +Z force
#
# Yaw is the subtle one: prop_left is on the PORT side, so driving it alone
# pushes the port side forward and swings the bow to STARBOARD, i.e. negative
# yaw. Measured: prop_left alone gives r = -2.11 rad/s.

EXPECTED_POSITIONS = {
    "prop_left": [-0.20, 0.16, 0.0],
    "prop_right": [-0.20, -0.16, 0.0],
    "prop_sway": [0.0, 0.0, -0.02],
    "prop_vert": [0.0, 0.0, 0.10],
}
EXPECTED_AXES = {
    "prop_left": [1, 0, 0], "prop_right": [1, 0, 0],
    "prop_sway": [0, 1, 0], "prop_vert": [0, 0, 1],
}


def test_thruster_positions_and_axes_are_frozen() -> None:
    units = {u["name"]: u for u
             in world_builder.load_config()["vehicle_bluerov2_phys"]["thrusters"]["units"]}
    for name, position in EXPECTED_POSITIONS.items():
        assert list(units[name]["position_m"]) == position, name
        assert list(units[name]["axis"]) == EXPECTED_AXES[name], name


def test_surge_drives_both_units_equally() -> None:
    vector = protocol.thrust(surge=100.0)
    assert vector["prop_left_joint"] == vector["prop_right_joint"] == 50.0
    assert vector["prop_sway_joint"] == vector["prop_vert_joint"] == 0.0


def test_positive_yaw_pushes_the_starboard_unit_harder() -> None:
    """The regression this file exists for."""
    vector = protocol.thrust(yaw=20.0)
    assert vector["prop_right_joint"] > vector["prop_left_joint"], (
        "positive yaw must drive the STARBOARD unit harder; prop_left is at "
        "+Y (port) and driving it alone yields r = -2.11 rad/s")
    assert vector["prop_right_joint"] == pytest.approx(10.0)
    assert vector["prop_left_joint"] == pytest.approx(-10.0)


def test_yaw_is_a_pure_couple() -> None:
    vector = protocol.thrust(yaw=20.0)
    assert vector["prop_left_joint"] + vector["prop_right_joint"] == pytest.approx(0.0)


def test_surge_and_yaw_superpose() -> None:
    vector = protocol.thrust(surge=100.0, yaw=20.0)
    assert vector["prop_left_joint"] == pytest.approx(40.0)
    assert vector["prop_right_joint"] == pytest.approx(60.0)


@pytest.mark.parametrize("axis,key", [(0, "surge"), (1, "sway"), (2, "heave")])
def test_translational_commands_are_sign_symmetric(axis: int, key: str) -> None:
    positive = protocol.thrust(**{key: 40.0})
    negative = protocol.thrust(**{key: -40.0})
    for joint in positive:
        assert positive[joint] == pytest.approx(-negative[joint]), joint


@pytest.mark.slow
def test_measured_response_follows_the_declared_sign_in_all_six_dof() -> None:
    """P13, as a permanent regression rather than a one-off validation."""
    from uuv_sim_physics.validation import harness

    world = world_builder.VALIDATED_WORLD_PATH
    cases = {"+surge": (protocol.thrust(surge=80.0), 0, +1),
             "-surge": (protocol.thrust(surge=-80.0), 0, -1),
             "+sway": (protocol.thrust(sway=40.0), 1, +1),
             "-sway": (protocol.thrust(sway=-40.0), 1, -1),
             "+heave": (protocol.thrust(heave=40.0), 2, +1),
             "-heave": (protocol.thrust(heave=-40.0), 2, -1)}
    wrong = []
    for label, (command, axis, expected) in cases.items():
        tel = harness.run([harness.Command(0.0, command)], duration_s=5.0,
                          settle_s=3.0, world=world)
        value = float(tel.window(1.5, 5.0).body_velocity()[:, axis].mean())
        if math.copysign(1.0, value) != expected:
            wrong.append(f"{label} -> {value:+.4f}")

    for label, sign in (("+yaw", +1), ("-yaw", -1)):
        tel = harness.run([harness.Command(0.0, protocol.thrust(yaw=sign * 80.0))],
                          duration_s=5.0, settle_s=3.0, world=world)
        rate = float(tel.window(1.5, 5.0).body_rates()[:, 2].mean())
        if math.copysign(1.0, rate) != sign:
            wrong.append(f"{label} -> {rate:+.4f}")

    assert not wrong, f"sign convention violated: {wrong}"
