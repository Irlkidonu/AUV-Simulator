"""Release-gate check #9, as a permanent test.

The rule is not "do not subscribe to /uuv/ground_truth". It is that no component
presenting an observation as sensor-derived may build it from privileged
simulator state, by any route. So the test is an allowlist over interfaces, and
it is checked for teeth: a synthetic violator must be caught, or the test is
decoration.
"""

from __future__ import annotations

import textwrap

import pytest

from uuv_sim_physics import privileged


def test_no_unapproved_module_touches_privileged_state() -> None:
    violations = privileged.audit()
    assert not violations, "\n".join(str(v) for v in violations)


def test_allowlist_is_well_formed() -> None:
    """Real modules, real justifications, and no observation producers."""
    assert not privileged.allowlist_is_well_formed()


def test_observation_producers_are_never_allowlisted() -> None:
    overlap = set(privileged.ALLOWED_CONSUMERS) & privileged.OBSERVATION_PRODUCERS
    assert not overlap, f"observation producers on the allowlist: {overlap}"


@pytest.mark.parametrize("snippet,label", [
    ('node.subscribe(Pose_V, "/world/w/dynamic_pose/info", cb)', "pose stream"),
    ('self.sub = create_subscription(PoseStamped, "/uuv/ground_truth", cb, 10)',
     "legacy ground-truth topic"),
    ('from gz.msgs10.pose_v_pb2 import Pose_V', "true-pose message import"),
    ('truth = backend.true_position()', "explicitly true state"),
    ('client.call(set_pose_request)', "set_pose teleport"),
])
def test_a_synthetic_violator_is_caught(tmp_path, snippet: str, label: str) -> None:
    """The audit must fail on a module that reaches for privileged state."""
    offender = tmp_path / "sensors"
    offender.mkdir()
    (offender / "optical.py").write_text(textwrap.dedent(f"""
        \"\"\"A perception node that cheats.\"\"\"
        def observe(backend, node, cb=None, create_subscription=None,
                    PoseStamped=None, Pose_V=None, client=None,
                    set_pose_request=None):
            {snippet}
            return None
        """))
    findings = privileged.audit(tmp_path)
    assert findings, f"audit missed a {label} violation"
    assert any(f.module == "sensors/optical.py" for f in findings)


def test_prose_mentioning_privileged_state_is_not_a_violation(tmp_path) -> None:
    """Docstrings and comments may name these interfaces in order to forbid them."""
    (tmp_path / "harmless.py").write_text(textwrap.dedent('''
        """This node must never read /uuv/ground_truth or dynamic_pose/info."""
        # ground_truth is deliberately not consumed here
        def observe(image):
            return image * 0.5
        '''))
    assert not privileged.audit(tmp_path)


def test_every_sensor_stream_originates_in_gazebo() -> None:
    """Each observation topic must be produced by a Gazebo sensor element."""
    import xml.etree.ElementTree as ET

    from uuv_sim_physics import world_builder

    root = ET.fromstring(world_builder.VALIDATED_WORLD_PATH.read_text())
    vehicle = next(m for m in root.findall("world/model")
                   if m.get("name") == "bluerov2_phys")
    sensors = {s.get("name"): s.get("type")
               for s in vehicle.findall("link/sensor")}

    assert sensors.get("camera") == "camera"
    assert sensors.get("fls") == "gpu_lidar"
    assert sensors.get("imu") == "imu"
    assert sensors.get("dvl") == "custom"

    # Each must declare its own topic; an observation with no sensor behind it
    # would have to come from somewhere else.
    for name in ("camera", "fls", "imu", "dvl"):
        element = vehicle.find(f"link/sensor[@name='{name}']/topic")
        assert element is not None and element.text.strip(), name
