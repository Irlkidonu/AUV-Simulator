"""Privileged-state allowlist: release-gate check #9.

The invariant:

    No component presenting an observation as sensor-derived may construct that
    observation using privileged simulator state.

"Privileged" means anything the simulator knows that a physical sensor could not
measure: true pose, true velocity, true attitude, the dock's true relative
position. A perception path that reaches for any of it is no longer perceiving,
it is reading the answer.

Banning one topic name is not enough. The same privileged state is reachable
through ``/world/<name>/dynamic_pose/info``, through ``GazeboBackend.position``,
and through any object holding a backend reference. So this module works the
other way round: it enumerates the interfaces that *are* privileged, enumerates
the modules *allowed* to touch them and why, and fails on anything else.

Adding a module to ``ALLOWED_CONSUMERS`` is a deliberate act that must carry a
reason. Adding one without a reason fails its own test.
"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path

__all__ = ["PRIVILEGED_INTERFACES", "ALLOWED_CONSUMERS", "Finding", "audit"]

PACKAGE_ROOT = Path(__file__).resolve().parent

#: Every route to state a physical sensor could not measure.
PRIVILEGED_INTERFACES: dict[str, str] = {
    r"/uuv/ground_truth": "legacy ground-truth pose topic",
    r"dynamic_pose/info": "Gazebo true-pose stream for every model",
    r"\bground_truth": "any ground-truth-named interface",
    r"\bpose/info\b": "Gazebo pose info stream",
    r"gz\.msgs\d*\.pose_v_pb2": "direct subscription to the true-pose message",
    r"\bset_pose": "teleporting a model bypasses the plant",
    r"\btrue_pose|\btrue_position|\btrue_state": "explicitly true state",
}

#: Backend attributes that return privileged state to a Python caller.
PRIVILEGED_ATTRIBUTES = frozenset({
    "position", "quaternion", "yaw", "yaw_rate", "velocity", "velocity_world",
})

#: Modules permitted to consume privileged state, each with its justification.
#: Anything not listed here must not touch the interfaces above.
ALLOWED_CONSUMERS: dict[str, str] = {
    "gazebo_backend.py":
        "IS the state-feedback provider; it reads the plant so others need not",
    "execution.py":
        "mission-level execution and the reduced/physics comparison; control "
        "validation only, never presented as an observation",
    "control/controller.py":
        "GROUND_TRUTH_CONTROL_VALIDATION -- state feedback for the control "
        "stack, explicitly labelled and never a perception path",
    "control/runner.py":
        "closed-loop sequencing under GROUND_TRUTH_CONTROL_VALIDATION",
    "control/maneuvers.py":
        "T1-T7 canonical control validation; measures the plant under closed "
        "loop, produces no observation",
    "validation/harness.py":
        "measurement harness -- the instrument that validates the plant",
    "validation/protocol.py":
        "P1-P16 physics validation",
    "validation/timestep_study.py":
        "P15 timestep convergence study; measures plant response across solver "
        "settings",
    "validation/figures.py":
        "renders validation evidence from harness telemetry",
    "validation/sensor_harness.py":
        "records Gazebo sensor streams and the true-pose track so sensor signs "
        "and frames can be checked against known motion; produces no observation",
    "validation/m3_closeout.py":
        "M3 closeout comparison and figures",
    "privileged.py":
        "this module names the interfaces in order to forbid them",
}

#: Modules that produce or process observations. These must NEVER appear in
#: ALLOWED_CONSUMERS; listed explicitly so a future edit that tries has to
#: delete a line that says not to.
OBSERVATION_PRODUCERS = frozenset({
    "sensors/__init__.py", "sensors/optical.py", "sensors/water_column.py",
    "sensors/fls.py", "sensors/camera.py", "sensors/imu.py", "sensors/dvl.py",
    "sensors/bridge.py", "perception.py",
})


@dataclass(frozen=True)
class Finding:
    module: str
    line: int
    interface: str
    reason: str
    text: str

    def __str__(self) -> str:
        return (f"{self.module}:{self.line} touches privileged state "
                f"({self.reason}): {self.text.strip()[:90]}")


def _source_files(root: Path | None = None):
    base = Path(root) if root else PACKAGE_ROOT
    for path in sorted(base.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        yield path, path.relative_to(base).as_posix()


def _strip_comments_and_docstrings(source: str) -> dict[int, str]:
    """Return code lines only.

    A module may *name* a privileged interface in prose -- this file does it
    constantly -- without consuming it. Only executable code counts.
    """
    keep: dict[int, str] = {}
    try:
        tree = ast.parse(source)
    except SyntaxError:                                      # pragma: no cover
        return {i + 1: line for i, line in enumerate(source.splitlines())}

    docstring_lines: set[int] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node, clean=False)
            if doc and node.body and isinstance(node.body[0], ast.Expr):
                first = node.body[0]
                docstring_lines.update(
                    range(first.lineno, (first.end_lineno or first.lineno) + 1))

    for index, line in enumerate(source.splitlines(), start=1):
        if index in docstring_lines:
            continue
        stripped = line.split("#", 1)[0]
        if stripped.strip():
            keep[index] = stripped
    return keep


def audit(root: Path | None = None) -> list[Finding]:
    """Every privileged-state use by a module not on the allowlist."""
    findings: list[Finding] = []
    for path, relative in _source_files(root):
        if relative in ALLOWED_CONSUMERS:
            continue
        code = _strip_comments_and_docstrings(path.read_text())
        for number, text in code.items():
            for pattern, reason in PRIVILEGED_INTERFACES.items():
                if re.search(pattern, text):
                    findings.append(Finding(relative, number, pattern,
                                            reason, text))
    return findings


def allowlist_is_well_formed() -> list[str]:
    """The allowlist must be honest: real modules, real reasons, no producers."""
    problems = []
    for module, reason in ALLOWED_CONSUMERS.items():
        if not (PACKAGE_ROOT / module).is_file():
            problems.append(f"allowlisted module does not exist: {module}")
        if len(reason.strip()) < 25:
            problems.append(f"allowlist entry lacks a justification: {module}")
        if module in OBSERVATION_PRODUCERS:
            problems.append(
                f"OBSERVATION PRODUCER on the privileged allowlist: {module}")
    return problems


if __name__ == "__main__":
    issues = allowlist_is_well_formed()
    violations = audit()
    for item in issues:
        print(f"ALLOWLIST: {item}")
    for finding in violations:
        print(f"VIOLATION: {finding}")
    print(f"\nallowlisted consumers : {len(ALLOWED_CONSUMERS)}")
    print(f"privileged interfaces : {len(PRIVILEGED_INTERFACES)}")
    print(f"violations            : {len(violations)}")
    print("GROUND-TRUTH ISOLATION:",
          "PASS" if not violations and not issues else "FAIL")
    raise SystemExit(1 if (violations or issues) else 0)
