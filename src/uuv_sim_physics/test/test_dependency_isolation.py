"""The dependency must run one way: uuv_sim_physics -> uuv_mode_aware_navigation.

Reverse coupling is the failure this package is designed to make impossible, so
it is tested directly rather than left to review. Each check runs in a *fresh
interpreter*: once pytest has imported both packages, ``sys.modules`` can no
longer tell you which import pulled in what.
"""

from __future__ import annotations

import subprocess
import sys
import textwrap

import pytest

#: Anything whose presence would mean the reduced path had acquired a runtime
#: dependency on ROS or Gazebo.
FORBIDDEN_ROOTS = (
    "rclpy", "rosidl_runtime_py", "ament_index_python", "launch", "launch_ros",
    "geometry_msgs", "std_msgs", "sensor_msgs", "nav_msgs", "ros_gz_interfaces",
    "gz", "gz_sim", "ros_gz_sim", "ros_gz_bridge",
)

HEADLESS_MODULES = (
    "acoustics", "availability", "campaign", "comparators", "environment",
    "estimator", "imaging", "manager", "mission", "modes", "optics", "sensors",
)


def _run(body: str) -> str:
    """Execute ``body`` in a clean interpreter and return its stdout."""
    completed = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(body)],
        capture_output=True, text=True, timeout=180,
    )
    assert completed.returncode == 0, (
        f"subprocess failed:\n{completed.stdout}\n{completed.stderr}")
    return completed.stdout.strip()


def test_existing_package_does_not_import_the_physics_package() -> None:
    """The reverse edge must not exist, for any headless module."""
    output = _run(f"""
        import importlib, sys
        for name in {HEADLESS_MODULES!r}:
            importlib.import_module('uuv_mode_aware_navigation.' + name)
        leaked = [m for m in sys.modules if m.split('.')[0] == 'uuv_sim_physics']
        print(','.join(leaked))
    """)
    assert output == "", f"existing package imported uuv_sim_physics: {output}"


def test_study3_does_not_import_the_physics_package() -> None:
    output = _run("""
        import importlib, sys
        importlib.import_module('uuv_mode_aware_navigation.study3')
        leaked = [m for m in sys.modules if m.split('.')[0] == 'uuv_sim_physics']
        print(','.join(leaked))
    """)
    assert output == "", f"study3 imported uuv_sim_physics: {output}"


def test_importing_this_package_pulls_in_no_ros_or_gazebo() -> None:
    output = _run(f"""
        import sys
        import uuv_sim_physics
        from uuv_sim_physics import DynamicsBackend, ReducedBackend
        leaked = sorted({{m.split('.')[0] for m in sys.modules
                         if m.split('.')[0] in {FORBIDDEN_ROOTS!r}}})
        print(','.join(leaked))
    """)
    assert output == "", f"physics package pulled in ROS/Gazebo: {output}"


def test_reduced_backend_runs_without_ros_or_gazebo() -> None:
    """Not merely importable -- usable, with ROS absent from the interpreter."""
    output = _run(f"""
        import sys
        for name in {FORBIDDEN_ROOTS!r}:
            sys.modules[name] = None          # poison: any import raises
        import numpy as np
        from uuv_sim_physics import ReducedBackend
        backend = ReducedBackend((0.0, 0.0, -5.0), (0.05, 0.0, 0.0))
        for _ in range(100):
            backend.step(np.array([0.4, 0.0, 0.0]), 0.5)
        print(f'{{backend.path_length_m:.6f}}')
    """)
    assert float(output) > 0.0


def test_package_namespace_exposes_no_gazebo_backend() -> None:
    """``GazeboBackend`` must never be re-exported from ``__init__``.

    Re-exporting it would make the headless install depend on Gazebo through
    the back door at M3, which is precisely when nobody would be looking.
    """
    import uuv_sim_physics
    assert set(uuv_sim_physics.__all__) == {"DynamicsBackend", "ReducedBackend"}
    assert not hasattr(uuv_sim_physics, "GazeboBackend")
    assert not hasattr(uuv_sim_physics, "gazebo_backend")


@pytest.mark.parametrize("module", HEADLESS_MODULES)
def test_headless_modules_still_import_with_ros_poisoned(module: str) -> None:
    """Restates the existing headless boundary from this side of the edge."""
    output = _run(f"""
        import sys
        for name in {FORBIDDEN_ROOTS!r}:
            sys.modules[name] = None
        import importlib
        importlib.import_module('uuv_mode_aware_navigation.{module}')
        print('ok')
    """)
    assert output == "ok"
