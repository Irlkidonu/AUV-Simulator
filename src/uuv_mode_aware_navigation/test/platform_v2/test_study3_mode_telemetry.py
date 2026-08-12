"""Unit tests for the analysis-only mode telemetry.

The accumulator must record what happened and influence nothing. These tests
pin its arithmetic so a later change cannot silently alter a reported number.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uuv_mode_aware_navigation.study3.modes import (  # noqa: E402
    ModeDecision, NavigationMode)
from uuv_mode_aware_navigation.study3.telemetry import ModeTelemetry  # noqa: E402


def _decision(mode, reason, absolute, velocity, fallback=False):
    return ModeDecision(mode, reason, absolute, velocity, fallback)


OPTICAL = _decision(NavigationMode.OPTICAL_DVL, "optical_and_bottom_lock_usable",
                    "optical", "bottom_lock_dvl")
RELATIVE = _decision(NavigationMode.RELATIVE_DEAD_RECKONING,
                     "no_observable_horizontal_absolute_fix", None,
                     "water_track_dvl", True)
TERMINAL = _decision(NavigationMode.TERMINAL_DEGRADED, "terminal_safety_boundary",
                     None, "inertial", True)


def test_dwell_and_direction_are_recorded_per_mode():
    telemetry = ModeTelemetry()
    for step in range(3):
        telemetry.observe(step * 2.0, OPTICAL, 2.0, aided=True)
    for step in range(3, 5):
        telemetry.observe(step * 2.0, RELATIVE, 2.0, aided=False)

    assert telemetry.dwell_s["optical_dvl"] == 6.0
    assert telemetry.dwell_s["relative_dead_reckoning"] == 4.0
    assert telemetry.transition_count == 1
    assert telemetry.directed_transition_counts() == (
        ("optical_dvl->relative_dead_reckoning", 1),)
    assert telemetry.modes_visited == ("optical_dvl", "relative_dead_reckoning")


def test_transition_reason_and_entries_are_observable_values():
    telemetry = ModeTelemetry()
    telemetry.observe(0.0, OPTICAL, 2.0, aided=True)
    telemetry.observe(2.0, RELATIVE, 2.0, aided=False)
    telemetry.observe(4.0, TERMINAL, 2.0, aided=False)

    assert telemetry.transition_reasons() == (
        ("no_observable_horizontal_absolute_fix", 1),
        ("terminal_safety_boundary", 1))
    assert telemetry.first_relative_entry == (2.0, "no_observable_horizontal_absolute_fix")
    assert telemetry.first_terminal_entry == (4.0, "terminal_safety_boundary")
    # Fallback is flagged at the first decision that declares it, which is the
    # relative entry rather than the terminal one.
    assert telemetry.first_fallback_entry == (2.0, "no_observable_horizontal_absolute_fix")


def test_source_attribution_and_unaided_time_accumulate():
    telemetry = ModeTelemetry()
    telemetry.observe(0.0, OPTICAL, 2.0, aided=True)
    telemetry.observe(2.0, RELATIVE, 2.0, aided=False)
    telemetry.observe(4.0, RELATIVE, 2.0, aided=False)

    assert telemetry.velocity_source_s == {"bottom_lock_dvl": 2.0, "water_track_dvl": 4.0}
    assert telemetry.absolute_source_s == {"optical": 2.0, "none": 4.0}
    assert telemetry.unaided_s == 4.0


def test_recovery_execution_is_counted_only_when_executed():
    telemetry = ModeTelemetry()
    telemetry.observe(0.0, OPTICAL, 2.0, aided=True,
                      recovery_action="continue", recovery_executed=False)
    telemetry.observe(2.0, OPTICAL, 2.0, aided=True,
                      recovery_action="lower_altitude", recovery_executed=True)
    telemetry.observe(4.0, OPTICAL, 2.0, aided=True,
                      recovery_action="lower_altitude", recovery_executed=True)

    assert telemetry.recovery_executed == {"lower_altitude": 2}


def test_mean_dwell_is_infinite_when_the_mode_never_changes():
    telemetry = ModeTelemetry()
    for step in range(4):
        telemetry.observe(step * 2.0, OPTICAL, 2.0, aided=True)
    assert telemetry.transition_count == 0
    assert math.isinf(telemetry.mean_dwell_s())
    assert telemetry.longest_dwell() == ("optical_dvl", 8.0)


def test_record_is_flat_and_serialisable():
    telemetry = ModeTelemetry()
    telemetry.observe(0.0, OPTICAL, 2.0, aided=True)
    telemetry.observe(2.0, RELATIVE, 2.0, aided=False)
    record = telemetry.as_record()

    import json
    json.dumps(record, default=str)          # must not raise
    for key in ("mode_dwell_s", "mode_transitions", "mode_transitions_directed",
                "mode_transition_reasons", "modes_visited_in_order",
                "velocity_source_s", "absolute_source_s",
                "time_without_horizontal_absolute_s", "recovery_executed"):
        assert key in record
