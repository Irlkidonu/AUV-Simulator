import inspect
from dataclasses import replace
import numpy as np

from uuv_mode_aware_navigation.localization import OpticalLocalizationSignal
from uuv_mode_aware_navigation.platform_v2 import AcousticSignal,DVLSignal,PlatformStepInput
from uuv_mode_aware_navigation.study3 import FixedConfiguration,PolicyKind,Study3Policy


def observation():
    return PlatformStepInput(0.,1.,OpticalLocalizationSignal(True,.8,.03,0.,30,.8,.4,.1,"available"),
        -.01,DVLSignal(True,False,0.,.9,-.01),
        AcousticSignal(False,0.,0.,None,None,2.,True,4.),0.,0.,3.,.5,.005)


def test_policy_boundary_has_no_truth_or_scenario_argument():
    names=set(inspect.signature(Study3Policy.step).parameters)
    assert names=={"self","observable"}
    fields=set(PlatformStepInput.__dataclass_fields__)
    assert not fields.intersection({"truth","true_pose","family","scenario","turbidity","fault_time"})


def test_hidden_truth_perturbation_cannot_change_identical_observation_decision():
    # Hidden worlds are deliberately not arguments. Two fresh policies supplied
    # the same serialized onboard observation must decide identically.
    a,_=Study3Policy(PolicyKind.PREDICTIVE).step(observation())
    hidden_truth_a={"true_pose":[0,0,-3],"turbidity":0.1}
    hidden_truth_b={"true_pose":[90,-40,-8],"turbidity":1.0}
    assert hidden_truth_a!=hidden_truth_b
    b,_=Study3Policy(PolicyKind.PREDICTIVE).step(observation())
    assert a==b


def test_four_frame_evidence_confirmation_is_reachable_and_healthy_channel_is_fixed_baseline():
    config=FixedConfiguration(optical_channel="lidar",trend_confirmation_frames=4,
                              minimum_cumulative_quality_decline=.01)
    policy=Study3Policy(PolicyKind.PREDICTIVE,config)
    base=observation()
    for i in range(4):
        if i:policy.estimator.predict(np.zeros(3),1.)
        optical=replace(base.optical,age_s=0.,keypoints_a=80-i*10,keypoints_b=80-i*10,
                        matches=70-i*10,inliers=60-i*10,inlier_fraction=.9-i*.1,
                        reprojection_px=.4+i*.2,sigma_m=.01+i*.01)
        action,_=policy.step(replace(base,time_s=float(i),optical=optical,
                                     optical_quality_trend_per_s=0.0))
    assert len(policy._optical_forecaster._history)==4
    assert action.optical_channel=="lidar"
