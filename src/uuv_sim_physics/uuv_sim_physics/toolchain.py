#!/usr/bin/env python3
"""PC-2: pin exactly one Gazebo stack, and refuse to run on any other.

Three Gazebo installations are present on this machine and ``gz`` resolves to a
different one depending on whether ROS has been sourced:

    ROS sourced      -> 8.11.0   (ROS-vendored Harmonic)
    ROS not sourced  -> 10.5.0, 8.14.0

An implicit PATH lookup is therefore not a reproducible way to start a physics
run. This module resolves the pinned stack by absolute path, forces the plugin
search configuration that selects it, and **verifies** the result before any
caller is allowed to proceed. It fails closed: a mismatch raises rather than
falling back, because a silently-substituted physics engine would invalidate
results without producing an error anywhere else.

Selected stack: ROS-vendored Gazebo Harmonic, gz-sim 8.11.0 -- the build that
``ros_gz_sim`` 1.0.22 is linked against. See ``PC2_REPORT.md`` for the evidence.

Read-only. Nothing here writes to the repository.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

# --- the pin ------------------------------------------------------------

GZ_VERSION = "8.11.0"
GZ_EXECUTABLE = Path("/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz")
ROS_DISTRO = "jazzy"
ROS_PREFIX = Path("/opt/ros/jazzy")
VENDOR_ROOT = ROS_PREFIX / "opt"

#: gz-tools picks its ``sim`` implementation from the first matching config on
#: GZ_CONFIG_PATH. Restricting it to the vendored prefixes is what makes 8.11.0
#: win over the apt 8.14.0 and 10.5.0 configs in /usr/share/gz.
GZ_CONFIG_DIRS = tuple(
    VENDOR_ROOT / name / "share" / "gz" for name in (
        "gz_sim_vendor", "sdformat_vendor", "gz_gui_vendor",
        "gz_transport_vendor", "gz_rendering_vendor", "gz_plugin_vendor",
        "gz_fuel_tools_vendor", "gz_msgs_vendor", "gz_common_vendor",
    )
)

DARTSIM_PLUGIN = (VENDOR_ROOT / "gz_physics_vendor" / "lib" / "gz-physics-7"
                  / "engine-plugins" / "libgz-physics-dartsim-plugin.so")
SYSTEM_PLUGIN_DIR = VENDOR_ROOT / "gz_sim_vendor" / "lib" / "gz-sim-8" / "plugins"
DART_VERSION = "6.13.2"          # vendored; the system also carries 6.16.6

#: Every system the reused docking SDFs instantiate.
REQUIRED_SYSTEMS = (
    "gz-sim-physics-system", "gz-sim-user-commands-system",
    "gz-sim-scene-broadcaster-system", "gz-sim-sensors-system",
    "gz-sim-imu-system", "gz-sim-dvl-system", "gz-sim-buoyancy-system",
    "gz-sim-hydrodynamics-system", "gz-sim-thruster-system",
    "gz-sim-particle-emitter-system",
    "gz-sim-joint-position-controller-system",
)


class ToolchainError(RuntimeError):
    """The pinned Gazebo stack is absent or a different one resolved."""


def environment(extra: dict | None = None) -> dict:
    """A process environment that can only resolve the pinned stack."""
    env = dict(os.environ)
    env["GZ_CONFIG_PATH"] = os.pathsep.join(str(p) for p in GZ_CONFIG_DIRS)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(SYSTEM_PLUGIN_DIR)
    env.setdefault("GZ_SIM_RESOURCE_PATH", "")
    if extra:
        env.update(extra)
    return env


def verify() -> dict:
    """Resolve and check the stack. Raises ``ToolchainError`` on any mismatch."""
    problems: list[str] = []

    if not GZ_EXECUTABLE.is_file():
        problems.append(f"pinned executable missing: {GZ_EXECUTABLE}")
    if not DARTSIM_PLUGIN.is_file():
        problems.append(f"pinned DART plugin missing: {DARTSIM_PLUGIN}")
    if not SYSTEM_PLUGIN_DIR.is_dir():
        problems.append(f"plugin directory missing: {SYSTEM_PLUGIN_DIR}")

    resolved = None
    if GZ_EXECUTABLE.is_file():
        completed = subprocess.run([str(GZ_EXECUTABLE), "sim", "--versions"],
                                   capture_output=True, text=True, timeout=60,
                                   env=environment())
        versions = [line.strip() for line in completed.stdout.splitlines()
                    if line.strip()]
        if versions != [GZ_VERSION]:
            problems.append(
                f"resolved gz-sim versions {versions}, expected exactly "
                f"['{GZ_VERSION}']")
        resolved = versions[0] if len(versions) == 1 else versions

    missing = [name for name in REQUIRED_SYSTEMS
               if not (SYSTEM_PLUGIN_DIR / f"lib{name}.so").is_file()]
    if missing:
        problems.append(f"required systems absent: {missing}")

    # An implicit `gz` on PATH is not necessarily the pinned one. That is a
    # warning, not a failure: this module never invokes it.
    on_path = shutil.which("gz")
    path_matches = on_path == str(GZ_EXECUTABLE)

    if problems:
        raise ToolchainError("; ".join(problems))

    return {
        "gz_sim_version": resolved,
        "gz_executable": str(GZ_EXECUTABLE),
        "gz_on_path": on_path,
        "gz_on_path_is_pinned": path_matches,
        "ros_distro": ROS_DISTRO,
        "dart_version": DART_VERSION,
        "physics_engine_plugin": str(DARTSIM_PLUGIN),
        "system_plugin_dir": str(SYSTEM_PLUGIN_DIR),
        "required_systems_present": len(REQUIRED_SYSTEMS),
    }


def _dpkg(package: str) -> str:
    try:
        out = subprocess.run(["dpkg-query", "-W", "-f=${Version}", package],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "(not installed)"
    except Exception:                                    # noqa: BLE001
        return "(unknown)"


def sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance(world: str | Path | None = None,
               vehicle_config: str | Path | None = None) -> dict:
    """The record every physics run must carry, per the M2 contract."""
    record = verify()
    record["ros_gz_versions"] = {
        name: _dpkg(f"ros-{ROS_DISTRO}-{name.replace('_', '-')}")
        for name in ("ros_gz_sim", "ros_gz_bridge", "ros_gz_interfaces")
    }
    record["vendor_versions"] = {
        name: _dpkg(f"ros-{ROS_DISTRO}-{name.replace('_', '-')}")
        for name in ("gz_sim_vendor", "gz_physics_vendor", "gz_dartsim_vendor")
    }
    record["world_sdf"] = ({"path": str(world), "sha256": sha256(world)}
                           if world else None)
    record["vehicle_config"] = ({"path": str(vehicle_config),
                                 "sha256": sha256(vehicle_config)}
                                if vehicle_config else None)
    record["uuv_sim_physics_git_sha"] = _git_sha()
    return record


def _git_sha() -> str:
    try:
        here = Path(__file__).resolve()
        out = subprocess.run(["git", "-C", str(here.parent), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "(untracked)"
    except Exception:                                    # noqa: BLE001
        return "(unknown)"


if __name__ == "__main__":
    try:
        print(json.dumps(provenance(), indent=2, sort_keys=True))
    except ToolchainError as error:
        print(f"TOOLCHAIN FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
