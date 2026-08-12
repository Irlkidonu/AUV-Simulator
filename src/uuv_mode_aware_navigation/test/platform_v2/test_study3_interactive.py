from dataclasses import asdict
import json
import threading
import time

import pytest

from uuv_mode_aware_navigation.study3.environment_generator import generate_environment
from uuv_mode_aware_navigation.study3.interactive import (
    CONTROL_DEFAULTS, InteractiveEnvironment, interactive_environment_config,
    load_recording, recording_record, run_interactive_session, SessionStopped,
)
from uuv_mode_aware_navigation.study3 import PolicyKind
from uuv_mode_aware_navigation.study3.policies import TerminalSafetyPrecedence


def environment(horizon=20., dt=1.):
    base = generate_environment(interactive_environment_config(), 31_895_000, horizon, dt)
    return InteractiveEnvironment(base, pace=False)


def sample(env, step, altitude=5.):
    return env.physical_state(step, altitude_m=altitude, position_xy=(.2 * step, 0.))


def test_controls_change_truth_side_physics_only_at_next_sensor_step():
    env = environment()
    nominal = sample(env, 0)
    env.set_control("turbidity", .8)
    env.set_control("current_north_mps", .3)
    env.set_control("dvl_crashout", True)
    degraded = sample(env, 1)
    assert nominal.turbidity == pytest.approx(.1)
    assert degraded.turbidity == pytest.approx(.8)
    assert degraded.current_north_mps == pytest.approx(.3)
    assert degraded.dvl_lock_probability == pytest.approx(.01)
    assert degraded.dvl_water_track_probability == pytest.approx(.01)
    assert degraded.dvl_forced_unavailable
    assert [x.effective_step for x in env.events] == [1, 1, 1]


def test_acoustic_assets_and_failures_require_sensor_response_not_free_labels():
    env = environment()
    sample(env, 0)
    env.set_control("usbl_available", False)
    env.set_control("lbl_geometry_scale", .2)
    env.set_control("acoustic_failure", True)
    state = sample(env, 1)
    assert state.deployed_acoustic_services == frozenset({"lbl"})
    assert state.response_probability("lbl") == pytest.approx(.01)
    assert state.response_probability("usbl") == pytest.approx(.01)
    # Catalogue identity is pre-mission information; current service quality is
    # absent and still has to traverse the existing serialized probe path.
    assert env.service_catalogue == ("lbl", "usbl")


def test_compound_failure_and_recovery_are_explicit_recorded_events():
    env = environment()
    sample(env, 0)
    env.apply_compound("all_horizontal")
    failed = sample(env, 1)
    assert failed.turbidity == 1.0
    assert failed.dvl_lock_probability == pytest.approx(.01)
    assert not failed.deployed_acoustic_services
    env.apply_compound("recover_all")
    restored = sample(env, 2)
    assert restored.turbidity == pytest.approx(CONTROL_DEFAULTS["turbidity"])
    assert restored.dvl_lock_probability == pytest.approx(.98)
    assert restored.deployed_acoustic_services == frozenset({"lbl", "usbl"})
    assert len(env.events) > 10


def test_recording_checksum_and_exact_replay(tmp_path):
    original = environment()
    states = [sample(original, 0)]
    original.set_control("turbidity", .72)
    states.append(sample(original, 1))
    original.set_control("lbl_available", False)
    original.set_control("dvl_noise_scale", 4.)
    states.extend(sample(original, i) for i in range(2, 6))
    record = recording_record(original, "reactive", root=31_895_100, index=0)
    path = tmp_path / "recording.json"
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    loaded, base = load_recording(path)
    replay = InteractiveEnvironment(base, replay_events=loaded["events"], pace=False)
    replay_states = [sample(replay, i) for i in range(6)]
    assert [asdict(x) for x in states] == [asdict(x) for x in replay_states]
    assert original.base.digest == replay.base.digest


def test_tampered_recording_is_rejected(tmp_path):
    env = environment()
    record = recording_record(env, "reactive", root=31_895_100, index=0)
    record["events"].append({"effective_step": 1, "simulation_time_s": 1.,
                             "sequence": 0, "control": "turbidity", "value": 1.})
    path = tmp_path / "tampered.json"
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match="checksum"):
        load_recording(path)


def test_future_evaluator_query_neither_advances_clock_nor_consumes_replay():
    base = generate_environment(interactive_environment_config(), 31_895_000, 20., 1.)
    event = {"effective_step": 3, "simulation_time_s": 3., "sequence": 0,
             "control": "turbidity", "value": .9}
    env = InteractiveEnvironment(base, replay_events=[event], pace=False)
    sample(env, 0)
    future = env.physical_state(10, altitude_m=5., position_xy=(0., 0.))
    assert future.turbidity == pytest.approx(.1)
    assert env.current_step == 0
    sample(env, 1); sample(env, 2)
    assert sample(env, 3).turbidity == pytest.approx(.9)


def test_pause_freezes_sensor_step_and_stop_unblocks_reset():
    env = environment()
    env.pause(True)
    outcome = []
    def blocked_sample():
        try:
            sample(env, 0)
        except SessionStopped:
            outcome.append("stopped")
    worker = threading.Thread(target=blocked_sample)
    worker.start()
    time.sleep(.05)
    assert env.current_step == -1 and worker.is_alive()
    env.stop()
    worker.join(1.)
    assert not worker.is_alive() and outcome == ["stopped"]


def test_terminal_safety_requires_complete_observable_loss_for_full_confirmation():
    probabilities = {"optical": .05, "acoustic": .05, "velocity": .05}
    guard = TerminalSafetyPrecedence(.35, 4.0, confirmation_s=8.0)
    conditions = dict(optical_evidence=False, position_acoustic_evidence=False,
        dvl_bottom_lock=False, dvl_water_track=False,
        capability_probability=probabilities, uncertainty_m2=5.0)
    assert not any(guard.update(1.0, **conditions) for _ in range(7))
    assert guard.update(1.0, **conditions)

    # Any genuinely viable observable mode resets confirmation; uncertainty by
    # itself must never trigger terminal behavior.
    blockers = (
        {"optical_evidence": True}, {"position_acoustic_evidence": True},
        {"dvl_bottom_lock": True}, {"dvl_water_track": True},
        {"capability_probability": {**probabilities, "velocity": .35}},
        {"uncertainty_m2": 3.99},
    )
    for change in blockers:
        modified = {**conditions, **change}
        assert not guard.update(20.0, **modified)
        assert guard.loss_duration_s == 0.0


def test_recorded_compound_fault_changes_real_evidence_mode_and_vehicle_action(tmp_path):
    base = generate_environment(interactive_environment_config(), 31_895_010, 70., 1.)
    author = InteractiveEnvironment(base, pace=False)
    author._current_step = 7
    author.set_control("optical_failure", True)
    author._current_step = 19
    author.set_control("dvl_crashout", True)
    author._current_step = 31
    for control, value in (("acoustic_failure", True),
                           ("lbl_available", False), ("usbl_available", False)):
        author.set_control(control, value)
    record = recording_record(author, "reactive", root=31_895_110, index=0)
    path = tmp_path / "mechanism.json"
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    telemetry = []
    _environment, completion = run_interactive_session(
        policy_kind=PolicyKind.REACTIVE, root=31_895_110, index=0,
        replay_record=path, pace=False, on_telemetry=telemetry.append)
    assert completion["status"] == "complete"
    assert any(x["optical_available"] for x in telemetry[:8])
    assert not any(x["optical_available"] for x in telemetry[12:20])
    assert any(x["dvl_bottom_lock"] for x in telemetry[:18])
    assert not any(x["dvl_bottom_lock"] for x in telemetry[24:32])
    assert {service for x in telemetry[:30] for service in x["responding_services"]}
    assert not {service for x in telemetry[40:] for service in x["responding_services"]}
    modes = {x["navigation_mode"] for x in telemetry}
    assert {"optical_dvl", "relative_dead_reckoning", "terminal_degraded"} <= modes
    # Until 2026-08-11 this asserted the vehicle visited USBL. With both LBL and
    # USBL deployed and equally good, that visit was the LBL/USBL chatter, not
    # adaptation: the corrected selector retains LBL because USBL is no better
    # by any observable measure. What must hold is that the selected technique
    # is one that actually answered, and that it is released when both are lost.
    answered = {service for x in telemetry[:30] for service in x["responding_services"]}
    assert any(x["selected_acoustic_technique"] in answered for x in telemetry[:30])
    terminal = [x for x in telemetry if x["mission_action"] == "surface_for_gps"]
    assert terminal
    assert terminal[0]["recovery_reason"] == "complete_navigation_loss_critical_uncertainty"
    # Crashout is a hardware state, not a low probability: even altitude gain
    # during surfacing cannot resurrect bottom- or water-track measurements.
    crashed = [x for x in telemetry if x["controls"]["dvl_crashout"]]
    assert crashed and any(x["terminal_or_surfacing"] for x in crashed)
    assert not any(x["dvl_bottom_lock"] or x["dvl_water_track"] for x in crashed)
