"""Mode inference and transition-stability tests (MODE_MANAGER_SPEC sections 1-2)."""

import math

import pytest

from uuv_mode_aware_navigation.modes import (
    CONSERVATISM,
    Mode,
    ModeStateMachine,
    ModeThresholds,
    Observables,
    infer_capability,
)

TH = ModeThresholds()


def healthy(**overrides) -> Observables:
    base = dict(
        optical_quality=0.90,
        optical_available=True,
        dvl_bottom_lock=True,
        dvl_age_s=0.05,
        acoustic_fix_age_s=1.0,
        imu_age_s=0.01,
        depth_age_s=0.01,
        position_covariance_trace=0.05,
        covariance_growth_rate=0.0,
        innovation_exceedance_rate=0.0,
        altitude_m=3.0,
    )
    base.update(overrides)
    return Observables(**base)


# ---------------------------------------------------------------------------
# Inference
# ---------------------------------------------------------------------------
def test_healthy_observables_give_nominal():
    assert infer_capability(healthy(), TH)[0] is Mode.NOMINAL


def test_low_optical_quality_degrades():
    mode, reason = infer_capability(healthy(optical_quality=0.30), TH)
    assert mode is Mode.OPTICAL_DEGRADED
    assert reason == "optical_quality_low"


def test_optical_unavailable_is_lost():
    assert infer_capability(healthy(optical_available=False), TH)[0] is Mode.OPTICAL_LOST


def test_dvl_bottom_lock_loss():
    mode, reason = infer_capability(healthy(dvl_bottom_lock=False), TH)
    assert mode is Mode.VELOCITY_AIDING_LOST
    assert reason == "dvl_bottom_lock_lost"


def test_compound_loss_is_critical():
    mode, reason = infer_capability(
        healthy(dvl_bottom_lock=False, optical_available=False,
                acoustic_fix_age_s=999.0),
        TH,
    )
    assert mode is Mode.DR_CRITICAL
    assert reason == "compound_aiding_loss"


def test_projected_covariance_triggers_critical_before_the_threshold_is_reached():
    """The manager must act on where uncertainty is heading, not only where it is."""
    obs = healthy(position_covariance_trace=0.5, covariance_growth_rate=0.5)
    mode, reason = infer_capability(obs, TH)
    assert mode is Mode.DR_CRITICAL
    assert reason == "covariance_projected_critical"


def test_innovation_exceedance_degrades_even_with_good_quality():
    obs = healthy(optical_quality=0.95, innovation_exceedance_rate=0.9)
    assert infer_capability(obs, TH)[0] is Mode.OPTICAL_DEGRADED


# ---------------------------------------------------------------------------
# Fail-closed
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    "bad",
    [
        dict(optical_quality=float("nan")),
        dict(optical_quality=1.7),
        dict(position_covariance_trace=float("inf")),
        dict(covariance_growth_rate=float("nan")),
        dict(altitude_m=0.0),
        dict(position_covariance_trace=-1.0),
    ],
)
def test_invalid_observables_fail_closed(bad):
    mode, reason = infer_capability(healthy(**bad), TH)
    assert mode is Mode.DR_CRITICAL
    assert reason == "invalid_observables"


def test_stale_inertial_is_critical():
    assert infer_capability(healthy(imu_age_s=5.0), TH)[0] is Mode.DR_CRITICAL


# ---------------------------------------------------------------------------
# Monotonicity in capability loss
# ---------------------------------------------------------------------------
def test_losing_a_modality_never_becomes_less_conservative():
    """Adding a fault may only move the decision to an equally or more
    conservative mode. A model that relaxes when something breaks is unsafe."""
    base = healthy()
    base_mode = infer_capability(base, TH)[0]
    faults = [
        dict(optical_quality=0.30),
        dict(optical_available=False),
        dict(dvl_bottom_lock=False),
        dict(imu_age_s=5.0),
        dict(acoustic_fix_age_s=999.0),
        dict(innovation_exceedance_rate=0.9),
        dict(covariance_growth_rate=0.5),
    ]
    for fault in faults:
        mode = infer_capability(healthy(**fault), TH)[0]
        assert CONSERVATISM[mode] >= CONSERVATISM[base_mode], f"relaxed under {fault}"


def test_accumulating_faults_is_monotone():
    accumulated, previous = {}, 0
    for fault in (
        dict(optical_quality=0.30),
        dict(optical_available=False),
        dict(dvl_bottom_lock=False),
        dict(acoustic_fix_age_s=999.0),
    ):
        accumulated.update(fault)
        level = CONSERVATISM[infer_capability(healthy(**accumulated), TH)[0]]
        assert level >= previous
        previous = level


# ---------------------------------------------------------------------------
# Transition graph
# ---------------------------------------------------------------------------
def test_recovery_from_critical_cannot_go_straight_to_nominal():
    """Direct M4 -> M0 is forbidden: leaving a critical state must pass through
    confirmed re-acquisition."""
    m = ModeStateMachine(TH, initial=Mode.DR_CRITICAL)
    seen = set()
    for _ in range(80):
        seen.add(m.update(healthy(), 0.5).mode)
    assert Mode.RECOVERY in seen, "never entered recovery"
    assert Mode.NOMINAL in seen, "never recovered to nominal"
    # and recovery must have preceded nominal
    m2 = ModeStateMachine(TH, initial=Mode.DR_CRITICAL)
    order = []
    for _ in range(80):
        d = m2.update(healthy(), 0.5)
        if not order or order[-1] is not d.mode:
            order.append(d.mode)
    assert order.index(Mode.RECOVERY) < order.index(Mode.NOMINAL)


# ---------------------------------------------------------------------------
# Stability: escalate fast, de-escalate slowly, do not chatter
# ---------------------------------------------------------------------------
def test_escalation_is_not_blocked_by_dwell():
    """Refusing to escalate because a timer has not elapsed would be unsafe."""
    m = ModeStateMachine(TH)
    m.update(healthy(), 0.1)
    for _ in range(int(TH.debounce_s / 0.1) + 2):
        d = m.update(healthy(dvl_bottom_lock=False), 0.1)
    assert d.mode is Mode.VELOCITY_AIDING_LOST


def test_de_escalation_requires_dwell():
    m = ModeStateMachine(TH)
    for _ in range(20):
        m.update(healthy(dvl_bottom_lock=False), 0.1)
    assert m.mode is Mode.VELOCITY_AIDING_LOST
    # healthy again, but not yet for the minimum dwell
    d = m.update(healthy(), 0.1)
    assert d.mode is Mode.VELOCITY_AIDING_LOST, "de-escalated instantly"


def test_oscillating_evidence_does_not_chatter():
    """The earlier system produced ~30 mode flips per second from an
    unnormalised quality signal with no hysteresis. Alternating evidence at
    10 Hz for 20 s must not reproduce that."""
    m = ModeStateMachine(TH)
    for i in range(200):
        quality = 0.9 if i % 2 == 0 else 0.1
        m.update(healthy(optical_quality=quality), 0.1)
    assert m.transitions <= 4, f"chattered: {m.transitions} transitions in 20 s"


def test_sustained_change_does_transition():
    """Hysteresis must not become deafness."""
    m = ModeStateMachine(TH)
    for _ in range(100):
        m.update(healthy(optical_quality=0.10), 0.1)
    assert m.mode is Mode.OPTICAL_DEGRADED
    assert m.transitions >= 1


def test_state_machine_is_deterministic():
    def run():
        m = ModeStateMachine(TH)
        out = []
        for i in range(120):
            obs = healthy(optical_quality=0.9 if i < 40 else 0.1,
                          dvl_bottom_lock=i < 80)
            d = m.update(obs, 0.1)
            out.append((d.mode, d.candidate, d.reason, d.transitioned))
        return out
    assert run() == run()


def test_decision_reports_its_evidence():
    """Explainability: the paper must be able to report *why* the manager
    switched, not merely that it did."""
    m = ModeStateMachine(TH)
    d = m.update(healthy(dvl_bottom_lock=False), 0.1)
    assert d.reason == "dvl_bottom_lock_lost"
    assert d.candidate is Mode.VELOCITY_AIDING_LOST
    assert math.isfinite(d.projected_covariance_m2)
