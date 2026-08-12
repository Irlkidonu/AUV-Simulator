from uuv_mode_aware_navigation.recovery import ActiveRecoveryPlanner, RecoveryAction, RecoveryState


def state(**changes):
    values=dict(optical_quality=.8,optical_quality_trend_per_s=0,altitude_m=3,
      speed_mps=.5,image_blur_m=0,acoustic_dop=1,acoustic_available=True,
      dvl_bottom_lock_probability=1,covariance_trace_m2=.2,expected_fix_s=0)
    values.update(changes); return RecoveryState(**values)


def test_predictive_recovery_acts_before_optical_loss() -> None:
    decision=ActiveRecoveryPlanner().decide(state(optical_quality=.3,optical_quality_trend_per_s=-.01))
    assert decision.action is RecoveryAction.LOWER_ALTITUDE
    assert decision.predictive


def test_actions_have_physical_observability_causes() -> None:
    planner=ActiveRecoveryPlanner()
    assert planner.decide(state(optical_quality=.1)).reason=="shorten_optical_path"
    assert planner.decide(state(optical_quality=.1,altitude_m=1,image_blur_m=.02)).action is RecoveryAction.REDUCE_SPEED
    assert planner.decide(state(acoustic_dop=10)).action is RecoveryAction.REPOSITION_FOR_ACOUSTICS
    assert planner.decide(state(dvl_bottom_lock_probability=.1)).reason=="restore_dvl_bottom_lock"


def test_terminal_recovery_is_not_masked_by_waiting() -> None:
    decision=ActiveRecoveryPlanner().decide(state(covariance_trace_m2=5,expected_fix_s=0,altitude_m=1))
    assert decision.action is RecoveryAction.SURFACE_FOR_GPS
