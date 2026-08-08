"""Terminal self-preservation must fire where it is needed and nowhere else.

These tests exist because the action was implemented three times before it ever
executed, and each failure was invisible to the suite that existed at the time.

The first criterion tested channel silence, which was too eager: an intermittent
beacon going quiet was read as total loss, so the vehicle abandoned the
turbid/DVL-loss family on every run and failed every mission there, in a family
the fixed policy completes without a single failure.

The second tested geometric reachability, which is impossible to satisfy: the LBL
array spans the survey area by design, so some technique is always reachable in
principle. Surfacing was never commanded once in 150 runs, and the tier-3
ablation was consequently bit-identical to the full manager.

The third fired correctly but was not terminal. It triggered at t = 103.5 s in
the compound family, held for two seconds, and reverted when conditions
momentarily improved -- against an ascent that needs sixty-six. The vehicle
decided to save itself roughly thirty times per run and never once arrived.

What all three share is that the *code was present* and the *behaviour was
absent*. Presence was never the failure mode, so these tests assert outcomes:
that the action fires, that it completes, and that it stays away from scenarios
the vehicle can still finish.
"""

import numpy as np
import pytest

from uuv_mode_aware_navigation.manager import (
    BLACKOUT_TIMEOUT_S,
    UNSURVEYABLE_COVARIANCE_M2,
    MissionAction,
    ModeAwareManager,
)
from uuv_mode_aware_navigation.modes import Mode, Observables


def _blind(trace, **kw):
    """A vehicle with no optical fix, no bottom lock and no water track."""
    base = dict(
        optical_quality=0.02,
        optical_available=False,
        dvl_bottom_lock=False,
        dvl_water_track=False,
        dvl_age_s=9.0,
        acoustic_fix_age_s=99.0,
        imu_age_s=0.0,
        depth_age_s=0.0,
        position_covariance_trace=trace,
        covariance_growth_rate=0.4,
    )
    base.update(kw)
    return Observables(**base)


def test_blackout_needs_a_degraded_estimate_not_merely_silence():
    """Silence alone is not evidence: the estimate must actually have gone."""
    m = ModeAwareManager()
    quiet_but_tight = _blind(trace=0.05)
    assert not m._blackout(quiet_but_tight), (
        "declared a blackout while the position estimate was still tight; that "
        "is the criterion that abandoned recoverable surveys"
    )
    assert m._blackout(_blind(trace=UNSURVEYABLE_COVARIANCE_M2 + 1.0))


def test_blackout_is_reachable_at_all():
    """The criterion must be satisfiable.

    Its predecessor asked whether any acoustic technique could deliver *in
    principle*. Because the transponder array spans the survey area that is
    always true, so the terminal action could never be reached and the whole
    tier was dead code that read as implemented.
    """
    m = ModeAwareManager()
    assert m._blackout(_blind(trace=UNSURVEYABLE_COVARIANCE_M2 * 2)), (
        "no observable state satisfies the blackout criterion"
    )


def test_healthy_vehicle_never_declares_a_blackout():
    m = ModeAwareManager()
    healthy = Observables(
        optical_quality=0.9,
        optical_available=True,
        dvl_bottom_lock=True,
        dvl_age_s=0.0,
        acoustic_fix_age_s=0.0,
        imu_age_s=0.0,
        depth_age_s=0.0,
        position_covariance_trace=0.01,
        covariance_growth_rate=0.0,
    )
    assert not m._blackout(healthy)


def test_surfacing_is_commanded_once_the_dwell_completes():
    m = ModeAwareManager()
    obs = _blind(trace=UNSURVEYABLE_COVARIANCE_M2 * 2)
    dt = 0.5
    actions = [
        m._mission_action(Mode.DR_CRITICAL, dt, obs)
        for _ in range(int(BLACKOUT_TIMEOUT_S / dt) + 4)
    ]
    assert MissionAction.SURFACE_FOR_GPS in actions, (
        f"a sustained blackout of {BLACKOUT_TIMEOUT_S:.0f} s never produced the "
        f"terminal action; observed {set(a.value for a in actions)}"
    )


def test_surfacing_is_terminal_once_committed():
    """The regression proper: a transient improvement must not cancel it.

    Without the latch the action lasted two seconds against an ascent needing
    sixty-six, so it was commanded repeatedly and completed never.
    """
    m = ModeAwareManager()
    dt = 0.5
    blackout = _blind(trace=UNSURVEYABLE_COVARIANCE_M2 * 2)
    for _ in range(int(BLACKOUT_TIMEOUT_S / dt) + 4):
        m._mission_action(Mode.DR_CRITICAL, dt, blackout)

    # Conditions improve, and the mode leaves the critical state entirely.
    recovered = Observables(
        optical_quality=0.9,
        optical_available=True,
        dvl_bottom_lock=True,
        dvl_age_s=0.0,
        acoustic_fix_age_s=0.0,
        imu_age_s=0.0,
        depth_age_s=0.0,
        position_covariance_trace=0.01,
        covariance_growth_rate=0.0,
    )
    for mode in (Mode.NOMINAL, Mode.OPTICAL_DEGRADED, Mode.RECOVERY):
        assert (
            m._mission_action(mode, dt, recovered) is MissionAction.SURFACE_FOR_GPS
        ), "self-preservation was abandoned because one fix arrived"


def test_a2_ablation_removes_the_terminal_action():
    """Tier 3 off must mean tier 3 off, including its terminal rung."""
    from uuv_mode_aware_navigation.manager import ManagerAblation

    m = ModeAwareManager(ablation=ManagerAblation(mission_actions=False))
    obs = _blind(trace=UNSURVEYABLE_COVARIANCE_M2 * 2)
    dt = 0.5
    for _ in range(int(BLACKOUT_TIMEOUT_S / dt) + 8):
        assert (
            m._mission_action(Mode.DR_CRITICAL, dt, obs) is MissionAction.CONTINUE
        ), "the A2 ablation still reached the terminal action"


def test_manager_source_has_no_duplicate_methods():
    """No method may be defined twice in the manager.

    Scripted edits produced a duplicated block in which Python bound the later
    copy while the earlier one was being edited, so three verification cycles
    ran against code that never executed. Presence of a fix means nothing if a
    second definition shadows it.
    """
    import inspect
    import re

    from uuv_mode_aware_navigation import manager as module

    source = inspect.getsource(module)
    names = re.findall(r"^    def (\w+)\(", source, flags=re.MULTILINE)
    duplicates = {n for n in names if names.count(n) > 1}
    assert not duplicates, f"methods defined more than once: {sorted(duplicates)}"
