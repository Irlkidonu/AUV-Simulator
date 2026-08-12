import inspect
from dataclasses import replace

import numpy as np

from uuv_mode_aware_navigation.study3 import FixedConfiguration,PolicyKind,Study3Action,Study3Policy,run_one
from uuv_mode_aware_navigation.study3.simulation import navigation_velocity
from uuv_mode_aware_navigation.localization import OpticalLocalizationSignal
from uuv_mode_aware_navigation.platform_v2 import AcousticSignal,DVLSignal,PlatformStepInput


def action(mission):
    return Study3Action(.5,5.,mission,"lidar","usbl","weight",False)


def test_every_mission_action_has_declared_motion_semantics():
    estimate=np.array([2.,1.]);last=np.array([1.,1.])
    assert np.allclose(navigation_velocity(action("continue"),estimate,last),[.075,0])
    assert np.allclose(navigation_velocity(action("hold_for_fix"),estimate,last),[0,0])
    assert np.allclose(navigation_velocity(action("abort_leg"),estimate,last),[0,.0375])
    assert np.allclose(navigation_velocity(action("surface_for_gps"),estimate,last),[0,0])
    assert np.allclose(navigation_velocity(action("return_to_last_good_fix"),estimate,last),[-.075,0])


def test_return_guidance_uses_estimated_state_only_and_changes_trajectory():
    assert set(inspect.signature(navigation_velocity).parameters)=={
        "action","estimated_xy","last_good_xy"}
    estimated=np.array([4.,3.]);target=np.array([1.,1.]);truth_a=np.array([20.,-8.])
    command=navigation_velocity(action("return_to_last_good_fix"),estimated,target)
    truth_b=truth_a+command*2.
    assert np.linalg.norm(estimated+command*2-target)<np.linalg.norm(estimated-target)
    assert not np.allclose(truth_a,truth_b)


def observation(time_s,optical_available=True,quality=.8):
    optical=OpticalLocalizationSignal(optical_available,quality,.03 if optical_available else np.inf,
        0.,30 if optical_available else 0,.8 if optical_available else 0,.4,.1,
        "available" if optical_available else "rejected_inliers")
    return PlatformStepInput(time_s,1.,optical,0.,DVLSignal(True,False,0.,.9,0.),
        AcousticSignal(False,time_s,time_s,None,None,np.inf,False,4.),0.,0.,5.,.5,.005)


def test_nominal_fusion_preserves_locked_covariance_weighting():
    policy=Study3Policy(PolicyKind.REACTIVE,FixedConfiguration(fusion_mode="weight"))
    selected,_=policy.step(observation(0.))
    assert selected.fusion_mode=="weight"


def test_failed_altitude_trial_is_suppressed_until_new_fix():
    config=FixedConfiguration(fusion_mode="weight",recovery_dwell_s=2.,recovery_cooldown_s=0.,
                              minimum_action_hold_s=0.)
    policy=Study3Policy(PolicyKind.REACTIVE,config)
    # Force the already inferred observable loss state; this test concerns
    # recovery outcome memory, not capability-filter calibration.
    policy._recovery_until_s=2.;policy._altitude_trial_active=True
    policy._recovery_cooldown_until_s=2.
    policy.estimator.predict(np.zeros(3),2.)
    selected,_=policy.step(observation(2.,False,.05))
    assert policy._altitude_trial_failed
    assert selected.altitude_m==config.altitude_m
    policy.estimator.predict(np.zeros(3),1.)
    policy.step(observation(3.,True,.8))
    assert not policy._altitude_trial_failed


def test_successful_lbl_transition_mechanism_is_preserved():
    fixed=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                             acoustic_technique="usbl",fusion_mode="weight")
    baseline=run_one(31_825_000,"S3_ACOUSTIC_GEOMETRY_ASYNC",0,PolicyKind.FIXED,
                     fixed,horizon_s=60.,redesign_version=3)
    reactive=run_one(31_825_000,"S3_ACOUSTIC_GEOMETRY_ASYNC",0,PolicyKind.REACTIVE,
                     fixed,horizon_s=60.,redesign_version=3)
    assert baseline.acoustic_fixes==0
    assert reactive.acoustic_fixes>=10
    assert "lbl" in reactive.acoustic_techniques_used
    assert reactive.rmse_transition_m<baseline.rmse_transition_m


def test_p5_reacquires_after_visibility_and_overlap_recover():
    fixed=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                             acoustic_technique="usbl",fusion_mode="weight",
                             minimum_action_hold_s=12.,recovery_cooldown_s=56.)
    result,trace=run_one(31_825_000,"S3_RECOVERY",2,PolicyKind.REACTIVE,
                         fixed,horizon_s=180.,redesign_version=3,keep_trace=True)
    assert any(row[0]>=135 and row[2]=="available" for row in trace)
    assert result.optical_fixes>=20
    assert result.completed and not result.safety_violation
