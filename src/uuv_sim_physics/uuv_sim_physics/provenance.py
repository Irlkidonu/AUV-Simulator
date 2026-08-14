"""The provenance record every physics run must carry.

A reduced-order run is reproducible from its seed. A physics run is not: it
depends on the solver build, the plugin versions, the exact world and the
configuration that generated it. So a physics result is only interpretable
alongside a record of what produced it, and that record has to be produced
automatically -- one written by hand is one that will eventually be wrong.

``record()`` fails closed through ``toolchain.verify()``: if the pinned Gazebo
stack is not the one that would run, no provenance is issued and no run should
start.
"""

from __future__ import annotations

import hashlib
import json
import platform
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from . import toolchain, world_builder

__all__ = ["record", "write", "KNOWN_COSMETIC_WARNINGS", "classify_log"]

#: Warnings the pinned stack emits on every run. Recorded, never suppressed:
#: filtering them would also hide a new warning that happened to match. Anything
#: not matching these is reported as new.
KNOWN_COSMETIC_WARNINGS = (
    "ParticleEmitter SetColorRange is currently disabled",
    "Ogre2Camera::SetVisibilityMask: Mask bits",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _source_digest() -> str:
    """Hash of the package's own Python and configuration sources."""
    root = world_builder.PACKAGE_ROOT
    digest = hashlib.sha256()
    files = sorted(list((root / "uuv_sim_physics").rglob("*.py"))
                   + list((root / "config").rglob("*.yaml")))
    for path in files:
        if "__pycache__" in path.parts:
            continue
        digest.update(path.relative_to(root).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _git_sha(path: Path) -> str:
    try:
        out = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "(untracked)"
    except Exception:                                        # noqa: BLE001
        return "(unknown)"


def record(world: Path | None = None, loaded_systems: tuple[str, ...] = (),
           extra: dict | None = None) -> dict:
    """Assemble the run record. Raises if the pinned toolchain does not verify."""
    world_path = Path(world) if world else world_builder.WORLD_PATH
    config = world_builder.load_config()
    engine = config["physics"]["engine"]

    stack = toolchain.verify()          # fails closed
    entry = {
        "captured_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "host": socket.gethostname(),
        "platform": {"os": platform.platform(), "python": platform.python_version()},

        "toolchain": {
            "gz_sim_version": stack["gz_sim_version"],
            "gz_executable": stack["gz_executable"],
            "physics_engine_plugin": stack["physics_engine_plugin"],
            "dart_version": stack["dart_version"],
            "ros_distro": stack["ros_distro"],
            "ros_gz_versions": toolchain.provenance()["ros_gz_versions"],
            "vendor_versions": toolchain.provenance()["vendor_versions"],
        },

        "world": {
            "name": world_builder.WORLD_NAME,
            "path": str(world_path),
            "sha256": _sha256(world_path),
        },
        "configuration": {
            "files": {name: _sha256(world_builder.CONFIG_DIR / f"{name}.yaml")
                      for name in ("physics", "vehicle_bluerov2_phys",
                                   "dock_station")},
            "combined_sha256": world_builder.config_digest(),
        },
        "vehicle": {
            "name": config["vehicle_bluerov2_phys"]["name"],
            "mass_kg": config["vehicle_bluerov2_phys"]["base_link"]["mass_kg"],
            "sha256": _sha256(world_builder.CONFIG_DIR / "vehicle_bluerov2_phys.yaml"),
        },

        "solver": {
            "engine": engine["type"],
            "max_step_size_s": engine["max_step_size_s"],
            "real_time_factor_target": engine["real_time_factor"],
        },

        "loaded_systems": list(loaded_systems),
        "known_cosmetic_warnings": list(KNOWN_COSMETIC_WARNINGS),

        "source": {
            "uuv_sim_physics_digest": _source_digest(),
            "repository_git_sha": _git_sha(world_builder.PACKAGE_ROOT),
        },
    }
    if extra:
        entry.update(extra)
    return entry


def classify_log(text: str) -> dict:
    """Split a Gazebo log into errors, known cosmetic warnings, and new ones."""
    errors, known, new = [], [], []
    for line in text.splitlines():
        if "[Err]" in line:
            errors.append(line.strip())
        elif "[Wrn]" in line:
            (known if any(k in line for k in KNOWN_COSMETIC_WARNINGS)
             else new).append(line.strip())
    return {"errors": errors,
            "known_cosmetic_warnings": sorted(set(known)),
            "new_warnings": sorted(set(new))}


def write(path: Path, **kwargs) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(record(**kwargs), indent=2, sort_keys=True) + "\n")
    return target


if __name__ == "__main__":
    try:
        print(json.dumps(record(), indent=2, sort_keys=True))
    except toolchain.ToolchainError as error:
        print(f"TOOLCHAIN FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
