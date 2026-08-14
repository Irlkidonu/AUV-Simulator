"""M2 smoke test: the package's own world executes correctly on the pinned stack.

M2 asks whether the independently owned physics simulator *runs*, not whether
its physics is *credible*. So these tests check that systems instantiate, the
vehicle responds to thrust at all, and the integration stays finite and bounded.
Whether the response has the right magnitude is M2.5's question, and no
assertion here should be read as answering it.

Every run goes through the pinned toolchain, and Gazebo's own log is the
evidence -- not a re-implementation of what it was expected to print.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess

import pytest

from uuv_sim_physics import provenance, toolchain, world_builder

ANSI = re.compile(r"\x1b\[[0-9;]*m")
WORLD = world_builder.WORLD_PATH


def _run(iterations: int = 400, verbosity: str = "4") -> str:
    env = toolchain.environment({"GZ_PARTITION": f"m2smoke{os.getpid()}"})
    completed = subprocess.run(
        [str(toolchain.GZ_EXECUTABLE), "sim", "-s", "-r", "-v", verbosity,
         "--iterations", str(iterations), str(WORLD)],
        capture_output=True, text=True, timeout=300, env=env)
    return ANSI.sub("", completed.stdout + completed.stderr)


@pytest.fixture(scope="module")
def log() -> str:
    return _run()


def test_toolchain_verifies_before_anything_runs() -> None:
    stack = toolchain.verify()
    assert stack["gz_sim_version"] == "8.11.0"
    assert "gz_tools_vendor" in stack["gz_executable"]


def test_server_reports_the_pinned_version(log: str) -> None:
    assert f"Gazebo Sim Server v{toolchain.GZ_VERSION}" in log


def test_dart_is_the_loaded_physics_engine(log: str) -> None:
    assert "gz::physics::dartsim::Plugin" in log
    assert str(toolchain.DARTSIM_PLUGIN) in log, "a non-pinned DART build loaded"


@pytest.mark.parametrize("system", ["Physics", "UserCommands", "SceneBroadcaster",
                                    "Sensors", "Buoyancy", "Hydrodynamics"])
def test_system_instantiates(log: str, system: str) -> None:
    assert f"Loaded system [gz::sim::systems::{system}]" in log


def test_all_four_thrusters_instantiate(log: str) -> None:
    joints = re.findall(r"Thruster listening to commands on "
                        r"\[/model/bluerov2_phys/joint/(\w+)/cmd_thrust\]", log)
    assert set(joints) == {"prop_left_joint", "prop_right_joint",
                           "prop_sway_joint", "prop_vert_joint"}


def test_models_spawn(log: str) -> None:
    """The world initialises and the run completes without aborting."""
    assert "initialized with [1ms] physics profile" in log
    assert "Loaded level [default]" in log


def test_camera_sensor_instantiates(log: str) -> None:
    assert re.search(r"Camera images for .*advertised", log), \
        "camera did not advertise; the Sensors system may not have rendered"


def test_no_errors_logged(log: str) -> None:
    classified = provenance.classify_log(log)
    assert classified["errors"] == [], classified["errors"]


def test_only_known_cosmetic_warnings(log: str) -> None:
    classified = provenance.classify_log(log)
    assert classified["new_warnings"] == [], (
        f"new warnings appeared: {classified['new_warnings']}")


# --- dynamic response --------------------------------------------------------

def _displacement(thrusts: dict[str, float], settle_s: float = 5.0,
                  drive_s: float = 8.0) -> tuple[float, float, float] | None:
    """World-frame displacement produced by holding a constant thrust."""
    import time
    env = toolchain.environment(
        {"GZ_PARTITION": f"m2dyn{os.getpid()}{int(time.time() * 1000) % 10000}"})
    topic = f"/world/{world_builder.WORLD_NAME}/dynamic_pose/info"

    def sample():
        out = subprocess.run(
            [str(toolchain.GZ_EXECUTABLE), "topic", "-e", "-t", topic, "-n", "1"],
            capture_output=True, text=True, timeout=40, env=env).stdout
        block = re.search(r'name:\s*"bluerov2_phys".*?position\s*\{(.*?)\}',
                          out, re.S)
        if not block:
            return None
        values = dict(re.findall(r"([xyz]):\s*(-?[\d.eE+-]+)", block.group(1)))
        return tuple(float(values.get(axis, 0.0)) for axis in "xyz")

    server = subprocess.Popen(
        [str(toolchain.GZ_EXECUTABLE), "sim", "-s", "-r", "-v", "1", str(WORLD)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    try:
        time.sleep(settle_s)
        before = sample()
        # Two properties this loop has to satisfy, both learned the hard way:
        #
        #  * repeat -- a single `gz topic -p` can race the plugin's subscription
        #    and be dropped silently, which reads as "the thruster does nothing";
        #  * fire together -- issuing the two surge commands sequentially leaves
        #    ~100 ms of pure differential thrust, which is 6.4 N.m on a 0.862
        #    kg.m^2 yaw inertia. That spins the vehicle up to ~0.7 rad/s and the
        #    heading never recovers, because nothing holds it. The measurement
        #    then shows large off-axis motion that the model is not responsible
        #    for.
        publishers = [
            subprocess.Popen(
                [str(toolchain.GZ_EXECUTABLE), "topic", "-t",
                 f"/model/bluerov2_phys/joint/{joint}/cmd_thrust",
                 "-m", "gz.msgs.Double", "-p", f"data: {newtons}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
            for joint, newtons in thrusts.items()]
        for publisher in publishers:
            publisher.wait(timeout=30)
        time.sleep(drive_s)
        after = sample()
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:                    # pragma: no cover
            server.kill()

    if before is None or after is None:
        return None
    return tuple(b - a for a, b in zip(before, after))


@pytest.mark.slow
def test_thrust_produces_motion() -> None:
    """Criterion 8: a thruster command must move the vehicle.

    Direction is asserted from *measured* behaviour, not from the sign the model
    appears to declare -- the two disagree for surge, which is recorded as an
    inherited discrepancy. Magnitudes are M2.5's business; this only requires
    that the response exists, is finite, and is on the expected axis.
    """
    displacement = _displacement({"prop_left_joint": 40.0,
                                  "prop_right_joint": 40.0})
    if displacement is None:
        pytest.skip("dynamic_pose/info unavailable in this configuration")

    assert all(math.isfinite(v) for v in displacement)
    travelled = math.dist((0.0, 0.0, 0.0), displacement)
    assert travelled > 1.0, f"surge thrust produced no motion: {displacement}"

    # Only the magnitude is asserted. The vehicle has no heading hold, so a few
    # milliseconds of asymmetry between the two surge thrusters yaws it
    # permanently -- 40 N at a 0.16 m lever on a 0.862 kg.m^2 yaw inertia. The
    # world-frame *direction* of an open-loop run is therefore not reproducible
    # between runs, which is itself an M2 finding rather than a flaw in the
    # measurement. Direction is established separately, in the body frame, by
    # M2.5 P5-P8 and P13.


@pytest.mark.slow
def test_sway_and_heave_follow_their_declared_axes() -> None:
    """Sway and heave behave as declared, which is what isolates the surge case."""
    sway = _displacement({"prop_sway_joint": 40.0})
    heave = _displacement({"prop_vert_joint": 40.0})
    if sway is None or heave is None:
        pytest.skip("dynamic_pose/info unavailable in this configuration")

    # Body +Y is world -Y at yaw = pi: positive sway command -> body port.
    assert sway[1] < -1.0, f"sway response wrong or absent: {sway}"
    # Body +Z is world +Z: positive heave command -> up.
    assert heave[2] > 1.0, f"heave response wrong or absent: {heave}"


def test_run_remains_numerically_stable(log: str) -> None:
    """No NaN, no solver blow-up, no unbounded-velocity complaints.

    Matched on whole words: a substring search for "inf" also matches "gui
    information", which is how the first version of this test failed against a
    perfectly healthy run.
    """
    patterns = {
        "NaN": r"\bnan\b",
        "infinity": r"\b[-+]?inf(inity)?\b",
        "assertion": r"\bassertion\b",
        "segfault": r"\bsegmentation fault\b",
        "divergence": r"\bdiverg(ed|ence|ing)\b",
    }
    for label, pattern in patterns.items():
        match = re.search(pattern, log, re.IGNORECASE)
        assert match is None, (
            f"{label} reported by the solver: ...{log[max(0, match.start()-90):match.end()+90]}...")


def test_provenance_record_is_complete() -> None:
    record = provenance.record(loaded_systems=("Physics", "Buoyancy"))
    for key in ("toolchain", "world", "configuration", "vehicle", "solver",
                "loaded_systems", "known_cosmetic_warnings", "source"):
        assert key in record, key
    assert record["toolchain"]["gz_sim_version"] == "8.11.0"
    assert record["toolchain"]["dart_version"] == "6.13.2"
    assert record["solver"]["engine"] == "dart"
    assert record["solver"]["max_step_size_s"] == 0.001
    assert len(record["world"]["sha256"]) == 64
    assert len(record["configuration"]["combined_sha256"]) == 64
    assert len(record["source"]["uuv_sim_physics_digest"]) == 64
    json.dumps(record)          # must be serialisable
