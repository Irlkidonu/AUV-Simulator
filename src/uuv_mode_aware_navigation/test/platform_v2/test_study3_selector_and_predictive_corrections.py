"""Mechanism tests for the two Study 3 corrections.

Selector: a viable acoustic mode must not be abandoned because its evidence was
not refreshed, while a *refuted* service must be abandoned at once. Switching
between two simultaneously viable services requires an observable uncertainty
improvement of at least the sqrt(2) derived in
``STUDY3_CORRECTION_SPECIFICATION_V1.md``.

PREDICTIVE: a capability that is still usable and is projected to reach its
floor within the horizon must be able to open a pre-emptive recovery episode,
and that episode must be able to act while the mode is still viable.

Nothing here asserts an outcome metric. These are behavioural contracts.
"""
from __future__ import annotations

import math
from dataclasses import asdict

import pytest

from uuv_mode_aware_navigation.capability.prediction import CapabilityDegradationPredictor
from uuv_mode_aware_navigation.platform_v2 import AcousticServiceEvidence
from uuv_mode_aware_navigation.study3 import FixedConfiguration, PolicyKind, Study3Policy
from uuv_mode_aware_navigation.study3.modes import NavigationMode, ObservableModeSelector
from uuv_mode_aware_navigation.study3.interactive import (
    interactive_environment_config, run_interactive_session)

VIABLE = dict(optical_probability=0.0, velocity_probability=1.0,
              dvl_bottom_lock=True, dvl_water_track=True)


def evidence(name, *, responding=True, sigma=0.03, dop=1.5, age=0.0, position=True):
    return AcousticServiceEvidence(name, responding, position, dop, sigma, age)


def withdrawn(name, age=0.0):
    """What the simulator submits when a probe finds no usable geometry.

    ``simulation.py`` submits infinite DOP and sigma when the service is not
    deployed or is unusable, and finite ones when the fix existed but the packet
    dropped. That is the observable difference between a service that is gone
    and one that merely missed a packet.
    """
    return AcousticServiceEvidence(name, False, True, math.inf, math.inf, age)


def dropped_packet(name, age=0.0):
    """A working service whose packet was lost: geometry is still valid."""
    return AcousticServiceEvidence(name, False, True, 1.5, 0.03, age)


@pytest.fixture
def selector():
    return ObservableModeSelector()


# --- stale evidence versus confirmed service loss ---------------------------

def test_absent_evidence_does_not_abandon_a_viable_mode(selector):
    """No news is not bad news: an un-refreshed service stays selected."""
    first = selector.select(0., services=[evidence("lbl")], **VIABLE)
    assert first.mode is NavigationMode.LBL_AIDED
    # LBL is simply not re-probed this step; USBL answers instead.
    later = selector.select(20., services=[evidence("usbl")], **VIABLE)
    assert later.mode is NavigationMode.LBL_AIDED, "stale evidence caused a switch"


def test_a_refuted_service_is_abandoned_in_the_same_step(selector):
    """A probe that completed without an answer is evidence of absence."""
    selector.select(0., services=[evidence("lbl")], **VIABLE)
    selector.select(4., services=[evidence("usbl")], **VIABLE)
    refuted = selector.select(20., services=[withdrawn("lbl")], **VIABLE)
    assert refuted.mode is NavigationMode.USBL_AIDED
    assert refuted.reason == "observable_usbl_fix"


def test_a_range_only_response_cannot_support_an_absolute_mode(selector):
    selector.select(0., services=[evidence("lbl")], **VIABLE)
    downgraded = selector.select(20., services=[evidence("lbl", position=False)], **VIABLE)
    assert downgraded.mode is not NavigationMode.LBL_AIDED


# --- LBL/USBL simultaneous viability ----------------------------------------

def test_equal_quality_services_do_not_cause_a_switch(selector):
    """The observed pathology: identical sigma, alternating refresh."""
    selector.select(0., services=[evidence("lbl", sigma=.03)], **VIABLE)
    modes = [selector.select(t, services=[evidence(
                 "usbl" if (t // 4) % 2 else "lbl", sigma=.03)], **VIABLE).mode
             for t in range(4, 120, 4)]
    assert set(modes) == {NavigationMode.LBL_AIDED}, (
        "alternating refresh of two equally good services still chatters")


def test_a_meaningfully_better_service_is_adopted(selector):
    """sqrt(2) better in sigma is four times... twice the information."""
    selector.select(0., services=[evidence("lbl", sigma=.30)], **VIABLE)
    better = .30 / math.sqrt(ObservableModeSelector.SWITCH_INFORMATION_RATIO)
    decision = selector.select(20., services=[evidence("usbl", sigma=better)], **VIABLE)
    assert decision.mode is NavigationMode.USBL_AIDED


def test_a_marginally_better_service_is_not_adopted(selector):
    selector.select(0., services=[evidence("lbl", sigma=.30)], **VIABLE)
    decision = selector.select(20., services=[evidence("usbl", sigma=.29)], **VIABLE)
    assert decision.mode is NavigationMode.LBL_AIDED


def test_the_switching_margin_is_the_declared_information_ratio():
    assert ObservableModeSelector.SWITCH_INFORMATION_RATIO == 2.0


# --- genuine modality loss and recovery -------------------------------------

def test_genuine_loss_then_recovery_round_trips(selector):
    selector.select(0., services=[evidence("lbl"), evidence("usbl")], **VIABLE)
    lost = selector.select(20., services=[withdrawn("lbl"), withdrawn("usbl")], **VIABLE)
    assert lost.mode is NavigationMode.RELATIVE_DEAD_RECKONING
    recovered = selector.select(60., services=[evidence("lbl")], **VIABLE)
    assert recovered.mode is NavigationMode.LBL_AIDED


def test_total_acoustic_refutation_falls_back_without_waiting(selector):
    """Immediate adaptation on real loss: no hold, no margin."""
    selector.select(0., services=[evidence("usbl")], **VIABLE)
    lost = selector.select(1., services=[withdrawn("usbl")], **VIABLE)
    assert lost.mode is NavigationMode.RELATIVE_DEAD_RECKONING


# --- rapid environmental transitions ----------------------------------------

def test_rapid_alternating_refutation_and_recovery_is_tracked(selector):
    """Refutation and confirmation on every step must both be honoured."""
    observed = []
    for step in range(20):
        entry = evidence("usbl") if step % 2 == 0 else withdrawn("usbl")
        observed.append(selector.select(
            float(step), services=[entry], **VIABLE).mode)
    assert NavigationMode.USBL_AIDED in observed
    assert NavigationMode.RELATIVE_DEAD_RECKONING in observed


# --- terminal safety --------------------------------------------------------

def test_terminal_safety_overrides_a_viable_acoustic_mode(selector):
    selector.select(0., services=[evidence("lbl")], **VIABLE)
    terminal = selector.select(1., services=[evidence("lbl")], terminal=True, **VIABLE)
    assert terminal.mode is NavigationMode.TERMINAL_DEGRADED
    assert terminal.reason == "terminal_safety_boundary"


def test_terminal_safety_is_not_delayed_by_the_hold(selector):
    selector.select(0., services=[evidence("lbl")], **VIABLE)
    selector.select(0.5, services=[evidence("usbl", sigma=.001)], **VIABLE)
    terminal = selector.select(1., services=[], terminal=True, **VIABLE)
    assert terminal.mode is NavigationMode.TERMINAL_DEGRADED


# --- PREDICTIVE -------------------------------------------------------------

def _session(policy, events, horizon_s=300., dt_s=2.):
    record = {"schema": "study3_interactive_disturbance_v1",
              "policy_recorded": policy, "root": 34_950_000, "index": 0,
              "seed": 34_950_000, "dt_s": dt_s, "horizon_s": horizon_s,
              "base_config": asdict(interactive_environment_config()),
              "service_catalogue": ["lbl", "usbl"], "events": events}
    samples = []

    class Probe(Study3Policy):
        pass

    _, completion = run_interactive_session(
        policy_kind=PolicyKind(policy), horizon_s=horizon_s, dt_s=dt_s,
        pace=False, replay_record=record, on_telemetry=samples.append)
    return completion, samples


#: A slow turbidity ramp: optical declines observably before it fails.
GRADUAL = [{"effective_step": 20 + n, "simulation_time_s": (20 + n) * 2.,
            "sequence": n, "control": "turbidity", "value": .10 + .04 * n}
           for n in range(20)]


def test_predictive_now_takes_preemptive_actions():
    """The repaired gate must actually open on a predicted, unrealised loss."""
    completion, _ = _session("predictive", GRADUAL)
    assert completion["status"] == "complete", completion.get("error")
    assert completion["result"]["preemptive_actions"] > 0, (
        "PREDICTIVE still never acts pre-emptively")


def test_reactive_never_takes_preemptive_actions():
    """Pre-emption stays exclusive to PREDICTIVE."""
    completion, _ = _session("reactive", GRADUAL)
    assert completion["status"] == "complete"
    assert completion["result"]["preemptive_actions"] == 0


def test_predictive_and_reactive_now_differ():
    """The contrast is no longer degenerate."""
    predictive, _ = _session("predictive", GRADUAL)
    reactive, _ = _session("reactive", GRADUAL)
    assert predictive["result"]["trace_digest"] != reactive["result"]["trace_digest"]


def test_time_to_loss_separates_prediction_from_detection():
    """Why ``impending`` could not gate pre-emption and ``time_to_loss_s`` can.

    ``_time_to_floor`` returns 0.0 for a capability already at or below its
    floor and a positive, finite value for one still above it but declining.
    ``impending`` is ``time_to_loss <= horizon``, so it contains both; only the
    strictly-positive test isolates an unrealised loss.
    """
    predictor = CapabilityDegradationPredictor(horizon_s=10.)
    already_lost = predictor._time_to_floor(
        predictor.optical_quality_floor - .05, -.01, predictor.optical_quality_floor)
    declining = predictor._time_to_floor(
        predictor.optical_quality_floor + .05, -.01, predictor.optical_quality_floor)
    steady = predictor._time_to_floor(
        predictor.optical_quality_floor + .05, 0., predictor.optical_quality_floor)

    assert already_lost == 0.0
    assert 0.0 < declining <= 10.0
    assert math.isinf(steady)
    # `impending` cannot tell the first two apart; the gate must.
    assert already_lost <= 10.0 and declining <= 10.0
    assert not (0. < already_lost) and (0. < declining)


def test_terminal_safety_still_commits_under_prediction():
    """Pre-emption must not be able to defer a terminal commitment."""
    events = [{"effective_step": 20, "simulation_time_s": 40., "sequence": 0,
               "control": "optical_failure", "value": True},
              {"effective_step": 21, "simulation_time_s": 42., "sequence": 1,
               "control": "dvl_crashout", "value": True},
              {"effective_step": 22, "simulation_time_s": 44., "sequence": 2,
               "control": "acoustic_failure", "value": True},
              {"effective_step": 23, "simulation_time_s": 46., "sequence": 3,
               "control": "lbl_available", "value": False},
              {"effective_step": 24, "simulation_time_s": 48., "sequence": 4,
               "control": "usbl_available", "value": False}]
    completion, samples = _session("predictive", events)
    assert completion["status"] == "complete"
    assert any(s["terminal_or_surfacing"] for s in samples), (
        "no terminal commitment despite total loss of submerged aiding")


def test_a_dropped_packet_is_not_evidence_of_loss(selector):
    """A stochastic packet drop from a service with valid geometry must hold.

    This is the distinction the simulator already encodes: a lost packet still
    carries finite DOP and sigma, a withdrawn service carries infinite ones.
    """
    selector.select(0., services=[evidence("usbl")], **VIABLE)
    held = selector.select(4., services=[dropped_packet("usbl")], **VIABLE)
    assert held.mode is NavigationMode.USBL_AIDED
    gone = selector.select(8., services=[withdrawn("usbl")], **VIABLE)
    assert gone.mode is NavigationMode.RELATIVE_DEAD_RECKONING
