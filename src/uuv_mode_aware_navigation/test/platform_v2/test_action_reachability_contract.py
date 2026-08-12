"""Documented action-space reachability without changing frozen costs."""

from __future__ import annotations

import inspect

from uuv_mode_aware_navigation.estimator import FusionMode
from uuv_mode_aware_navigation.manager import (
    DEFAULT_CANDIDATES,
    MissionAction,
    MissionCosts,
    ModeAwareManager,
    SPEED_NOMINAL_MPS,
    SPEED_REDUCED_MPS,
)


def test_every_configuration_axis_is_declared() -> None:
    assert {c.optical.name for c in DEFAULT_CANDIDATES} == {
        "camera_coaxial", "camera_offaxis", "lidar"
    }
    assert {c.altitude_m for c in DEFAULT_CANDIDATES} == {1.0, 2.0, 3.0}
    assert {c.speed_mps for c in DEFAULT_CANDIDATES} == {0.25, 0.50}
    assert {c.acoustic.name for c in DEFAULT_CANDIDATES} == {
        "single_beacon", "lbl", "usbl", "terrain_relative"
    }
    assert {c.fusion for c in DEFAULT_CANDIDATES} == {
        FusionMode.GATE, FusionMode.WEIGHT
    }


def test_reduced_speed_is_unreachable_by_frozen_objective_arithmetic() -> None:
    costs = MissionCosts()
    time_loss = (SPEED_NOMINAL_MPS - SPEED_REDUCED_MPS) / SPEED_NOMINAL_MPS
    objective_penalty_m2 = costs.cost_equivalence_m2 * costs.time_weight * time_loss
    declared_benefit_ceiling_m2 = 0.010
    assert objective_penalty_m2 == 0.015
    assert objective_penalty_m2 > declared_benefit_ceiling_m2


def test_abort_leg_has_no_implementation_path() -> None:
    source = inspect.getsource(ModeAwareManager._mission_action)
    assert "return MissionAction.ABORT_LEG" not in source
    assert MissionAction.ABORT_LEG.value == "abort_leg"


def test_surface_is_terminal_safety_action() -> None:
    source = inspect.getsource(ModeAwareManager._mission_action)
    assert "_surfacing_committed" in source
    assert "return MissionAction.SURFACE_FOR_GPS" in source


def test_return_to_last_good_fix_is_ablation_fallback_only() -> None:
    source = inspect.getsource(ModeAwareManager._mission_action)
    assert "if self.ablation.acoustic_aiding" in source
    assert "else MissionAction.RETURN_TO_LAST_GOOD_FIX" in source
