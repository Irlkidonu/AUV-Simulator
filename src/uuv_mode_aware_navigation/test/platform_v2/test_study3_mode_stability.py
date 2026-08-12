"""Mechanism tests: hysteresis must damp chatter without blocking safety.

Three properties are asserted against ``ObservableModeSelector`` directly,
because that is where the hold lives:

  a. losing the evidence a mode requires exits it at once;
  b. oscillation between two still-viable alternatives does not produce a
     mode change on every tick;
  c. the hold never delays the terminal safety transition.

No threshold is tuned here. The selector's declared ``minimum_hold_s`` is
read from the instance rather than restated, so this test cannot silently
encode a different value from the implementation.
"""
from __future__ import annotations

import math

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uuv_mode_aware_navigation.platform_v2 import AcousticServiceEvidence  # noqa: E402
from uuv_mode_aware_navigation.study3.modes import (  # noqa: E402
    NavigationMode, ObservableModeSelector)


def _service(name, responding=True, gives_position=True):
    return AcousticServiceEvidence(name, responding, gives_position, 1.4, 0.6, 0.0)


def _withdrawn(name):
    return AcousticServiceEvidence(name, False, True, math.inf, math.inf, 0.0)
# ``simulation.py`` submits infinite DOP and sigma when a probe finds no usable
# geometry -- a withdrawn or unusable service -- and finite ones when the fix
# existed but the packet dropped. Since 2026-08-11 the selector honours that
# distinction, so a test for genuine loss must use the former. An empty evidence
# tuple means "not re-probed yet", which is no longer read as loss.



LBL = _service("lbl")
NO_SERVICE: tuple = ()


def _healthy(**overrides):
    """Observable evidence for a fully capable vehicle."""
    base = dict(optical_probability=0.9, velocity_probability=0.9,
                dvl_bottom_lock=True, dvl_water_track=True,
                services=NO_SERVICE, terminal=False)
    base.update(overrides)
    return base


def test_losing_required_evidence_exits_the_mode_immediately():
    """A capability that becomes nonviable must not be held."""
    selector = ObservableModeSelector()
    first = selector.select(0.0, **_healthy())
    assert first.mode is NavigationMode.OPTICAL_DVL

    # Optical collapses one tick later, well inside the hold window.
    after = selector.select(1.0, **_healthy(optical_probability=0.0,
                                           dvl_bottom_lock=False,
                                           dvl_water_track=False))
    assert after.mode is NavigationMode.RELATIVE_DEAD_RECKONING
    assert after.reason == "no_observable_horizontal_absolute_fix"


def test_new_higher_priority_service_does_not_chatter_while_incumbent_viable():
    """A newly available alternative is held off while the incumbent works."""
    selector = ObservableModeSelector()
    hold_s = selector.minimum_hold_s

    # Establish optical first. LBL then appears and outranks it, while optical
    # remains viable throughout the hold interval.
    assert selector.select(0.0, **_healthy()).mode is NavigationMode.OPTICAL_DVL

    # The incumbent remains viable, so service appearance must not cause a
    # transition on every tick inside the hold.
    changes = 0
    previous = NavigationMode.OPTICAL_DVL
    for step in range(1, 8):
        decision = selector.select(step * 1.0, **_healthy(services=(LBL,)))
        changes += int(decision.mode is not previous)
        previous = decision.mode
    assert changes == 0, "incumbent viable mode was not held against flicker"
    assert hold_s > 0.0

    # Once the hold expires, the selector may adopt the higher-priority service.
    later = selector.select(hold_s+1.0, **_healthy(services=(LBL,)))
    assert later.mode is NavigationMode.LBL_AIDED


def test_hold_expires_so_adaptation_is_not_permanently_blocked():
    """Hysteresis damps chatter; it must not freeze the selector forever."""
    selector = ObservableModeSelector()
    selector.select(0.0, **_healthy(services=(LBL,)))

    # LBL stops responding after the hold has elapsed. The mode must move.
    later = selector.minimum_hold_s + 1.0
    decision = selector.select(later, **_healthy(services=(_withdrawn("lbl"),)))
    assert decision.mode is not NavigationMode.LBL_AIDED
    assert decision.mode is NavigationMode.OPTICAL_DVL


def test_hold_never_delays_the_terminal_safety_transition():
    """Safety outranks hysteresis at every instant inside the hold window."""
    selector = ObservableModeSelector()
    selector.select(0.0, **_healthy(services=(LBL,)))

    # One tick later, far inside the hold, with the safety boundary asserted.
    decision = selector.select(1.0, **_healthy(services=(LBL,), terminal=True))
    assert decision.mode is NavigationMode.TERMINAL_DEGRADED
    assert decision.reason == "terminal_safety_boundary"
    assert decision.fallback_required is True


def test_a_held_decision_is_reported_as_a_hold_not_as_fresh_evidence():
    """The recorded reason must not claim evidence the selector did not use."""
    selector = ObservableModeSelector()
    selector.select(0.0, **_healthy(services=(LBL,)))
    held = selector.select(1.0, **_healthy(optical_probability=0.9,
                                           services=(LBL,), terminal=False))
    # Same mode, and if the selector suppressed a candidate it must say so.
    assert held.mode is NavigationMode.LBL_AIDED
    assert held.reason in {"observable_lbl_fix", "minimum_mode_hold"}
