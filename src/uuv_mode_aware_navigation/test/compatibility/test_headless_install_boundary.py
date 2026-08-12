"""The legacy numerical campaign remains importable without ROS or Gazebo."""

from __future__ import annotations

import importlib


HEADLESS_MODULES = (
    "acoustics",
    "availability",
    "campaign",
    "comparators",
    "environment",
    "estimator",
    "imaging",
    "manager",
    "mission",
    "modes",
    "optics",
    "sensors",
)


def test_campaign_import_closure_has_no_ros_runtime_dependency() -> None:
    for module in HEADLESS_MODULES:
        importlib.import_module(f"uuv_mode_aware_navigation.{module}")
