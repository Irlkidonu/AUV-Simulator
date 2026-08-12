"""Mechanism tests: range-only aiding, and Study-3 action reachability.

Two unrelated properties share a file because both are small and both concern
what the *current* Study-3 controller can actually do.

Part 1 -- a range-only service must never be treated as a horizontal position
fix. Single-beacon ranging constrains the vehicle to a spherical locus; the
selector must not enter an absolute-aided mode on it.

Part 2 -- reachability of the actions the Study-3 controller retains. This
audits the Study-3 selector, not the historical Study-2 manager. An action that
appears in the declared space but cannot be produced is reported here rather
than assumed reachable.
"""
from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uuv_mode_aware_navigation.acoustics import (  # noqa: E402
    LBL, SINGLE_BEACON, USBL)
from uuv_mode_aware_navigation.platform_v2 import AcousticServiceEvidence  # noqa: E402
from uuv_mode_aware_navigation import recovery as recovery_module  # noqa: E402
from uuv_mode_aware_navigation.recovery import (  # noqa: E402
    ActiveRecoveryPlanner, RecoveryAction, RecoveryState)
from uuv_mode_aware_navigation.study3.modes import (  # noqa: E402
    NavigationMode, ObservableModeSelector)
from uuv_mode_aware_navigation.study3.policies import STUDY3_RECOVERY_ACTIONS  # noqa: E402


def _service(name, *, gives_position, responding=True):
    return AcousticServiceEvidence(name, responding, gives_position, 1.4, 0.6, 0.0)


def _evidence(**overrides):
    base = dict(optical_probability=0.0, velocity_probability=0.9,
                dvl_bottom_lock=True, dvl_water_track=True,
                services=(), terminal=False)
    base.update(overrides)
    return base


# --- Part 1: range-only aiding ------------------------------------------


def test_single_beacon_is_declared_range_only_in_the_technique_model():
    """The range-only property is a model fact, not a test assumption."""
    assert SINGLE_BEACON.gives_position is False
    assert LBL.gives_position is True
    assert USBL.gives_position is True


def test_range_only_service_cannot_produce_an_absolute_aided_mode():
    """A responding single beacon must not be mistaken for a position fix."""
    selector = ObservableModeSelector()
    beacon = _service("single_beacon", gives_position=False)
    decision = selector.select(0.0, **_evidence(services=(beacon,)))

    assert decision.mode is NavigationMode.RELATIVE_DEAD_RECKONING
    assert decision.absolute_source is None
    assert decision.reason == "no_observable_horizontal_absolute_fix"
    # Healthy bottom-lock DVL remains a viable relative-navigation capability;
    # range-only evidence must neither invent an absolute mode nor force spatial
    # recovery while that velocity aid is usable.
    assert decision.fallback_required is False


def test_range_only_service_does_not_mask_a_genuine_position_service():
    """A real position service alongside the beacon is still selected."""
    selector = ObservableModeSelector()
    services = (_service("single_beacon", gives_position=False),
                _service("lbl", gives_position=True))
    decision = selector.select(0.0, **_evidence(services=services))

    assert decision.mode is NavigationMode.LBL_AIDED
    assert decision.absolute_source == "lbl"


def test_non_responding_position_service_is_not_selected():
    """Availability requires an observed response, not mere configuration."""
    selector = ObservableModeSelector()
    silent = _service("lbl", gives_position=True, responding=False)
    decision = selector.select(0.0, **_evidence(services=(silent,)))

    assert decision.mode is NavigationMode.RELATIVE_DEAD_RECKONING
    assert decision.absolute_source is None


# --- Part 2: Study-3 action reachability ---------------------------------


def test_every_navigation_mode_is_reachable_from_observable_evidence():
    """Each of the six modes must be produced by some physical evidence."""
    reached = {}

    selector = ObservableModeSelector()
    reached[selector.select(0.0, **_evidence(optical_probability=0.9)).mode] = "optical + bottom lock"

    selector = ObservableModeSelector()
    reached[selector.select(0.0, **_evidence(optical_probability=0.9,
                                             dvl_bottom_lock=False)).mode] = "optical, no bottom lock"

    selector = ObservableModeSelector()
    reached[selector.select(0.0, **_evidence(
        services=(_service("lbl", gives_position=True),))).mode] = "responding LBL"

    selector = ObservableModeSelector()
    reached[selector.select(0.0, **_evidence(
        services=(_service("usbl", gives_position=True),))).mode] = "responding USBL"

    selector = ObservableModeSelector()
    reached[selector.select(0.0, **_evidence()).mode] = "no absolute source"

    selector = ObservableModeSelector()
    reached[selector.select(0.0, **_evidence(terminal=True)).mode] = "terminal boundary"

    missing = set(NavigationMode) - set(reached)
    assert not missing, f"unreachable navigation modes: {sorted(m.value for m in missing)}"


def test_declared_recovery_actions_have_distinct_declared_semantics():
    """Every retained recovery action must be a distinct named behaviour.

    This asserts the declared set is coherent. Whether each is *selected* under
    the current objective is a separate development question and is not
    asserted here.
    """
    values = [action.value for action in RecoveryAction]
    assert len(values) == len(set(values))
    assert "continue" in values, "a no-op baseline action must exist"


def test_reduced_speed_is_reachable_in_the_study3_recovery_selector():
    """Study-3 reduced speed is produced by observable motion-blur evidence.

    Study 2's reduced speed was unreachable by cost arithmetic. The Study-3
    selector is a different mechanism: reduced speed is returned when optical
    quality is below floor, altitude is already at the floor, measured image
    blur exceeds its threshold and speed is above the reduced value. That is a
    physical benefit, not a cheaper price, so the action is genuinely reachable.
    """
    planner = ActiveRecoveryPlanner()
    state = RecoveryState(
        optical_quality=0.05, optical_quality_trend_per_s=0.0,
        altitude_m=1.0, speed_mps=0.5, image_blur_m=0.05,
        acoustic_dop=1.0, acoustic_available=False,
        dvl_bottom_lock_probability=0.9, covariance_trace_m2=0.1,
        expected_fix_s=1.0)
    decision = planner.decide(state)
    assert decision.action is RecoveryAction.REDUCE_SPEED
    assert decision.reason == "reduce_motion_blur"


def test_change_heading_is_explicitly_outside_current_study3_action_set():
    """Do not claim an action for which Study 3 has no physical mechanism."""
    source = Path(recovery_module.__file__).read_text()
    returns = [line for line in source.splitlines()
               if "RecoveryAction.CHANGE_HEADING" in line and "return" in line]
    assert returns == []
    assert RecoveryAction.CHANGE_HEADING.value == "change_heading"
    assert RecoveryAction.CHANGE_HEADING.value not in STUDY3_RECOVERY_ACTIONS
    assert {action.value for action in RecoveryAction}-{RecoveryAction.CHANGE_HEADING.value} == \
        STUDY3_RECOVERY_ACTIONS
