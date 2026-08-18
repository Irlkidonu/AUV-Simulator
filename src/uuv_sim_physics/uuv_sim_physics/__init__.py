"""Physics-capable execution path for the AUV Simulator.

An additive sibling to ``uuv_mode_aware_navigation``. The dependency runs one
way only -- this package imports that one, never the reverse -- so the existing
deterministic simulator, its campaigns and its frozen evidence are unaffected
by anything here.

What this package exports at import time is deliberately ROS-free and
Gazebo-free: the protocol and the reduced backend, nothing more. The Gazebo
backend (M3) must be imported from its own module by a caller that has already
decided to pay for Gazebo::

    from uuv_sim_physics import DynamicsBackend, ReducedBackend   # always safe
    from uuv_sim_physics.gazebo_backend import GazeboBackend      # needs Gazebo

``test_dependency_isolation.py`` enforces that: importing this package must not
pull a single ROS or Gazebo module into ``sys.modules``. Re-exporting the
Gazebo backend here would make the headless install depend on Gazebo through
the back door, and the test would fail.
"""

from .backend import DynamicsBackend
from .reduced_backend import ReducedBackend

__all__ = ["DynamicsBackend", "ReducedBackend"]

__version__ = "2.0.1"
