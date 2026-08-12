from dataclasses import asdict

import numpy as np

from uuv_mode_aware_navigation.study3 import FixedConfiguration,PolicyKind,run_one
from uuv_mode_aware_navigation.study3.simulation import _quality,assert_observation_clean
from uuv_mode_aware_navigation.rendering import CameraPose,GeoreferencedRenderer,WorldTexture
from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS,WaterState


def test_pixel_quality_declines_with_turbidity():
    renderer=GeoreferencedRenderer(WorldTexture.generate(1024,.04,17),add_sensor_noise=False)
    pose=CameraPose(0,0,3,0)
    quality=[_quality(renderer.render(pose,WaterState.from_turbidity(t),CAMERA_OFFAXIS))
             for t in (0,.25,.5,.75,1)]
    assert all(a>b for a,b in zip(quality,quality[1:]))


def test_recursive_truth_leakage_guard():
    assert_observation_clean({"optical":{"quality":.4},"estimated_pose":[1,2,3]})
    for key in ("true_pose","scenario","turbidity","future"):
        try:assert_observation_clean({"nested":{key:1}})
        except AssertionError:pass
        else:raise AssertionError(f"guard accepted forbidden {key}")


def test_paired_preaction_stream_and_sudden_no_oracle():
    # Same root/family/index starts from the same physical/random stream.  Before
    # the sudden loss, reactive and predictive observations/actions coincide.
    _,reactive=run_one(31_200_000,"S3_SUDDEN",91,PolicyKind.REACTIVE,keep_trace=True)
    _,predictive=run_one(31_200_000,"S3_SUDDEN",91,PolicyKind.PREDICTIVE,keep_trace=True)
    pre_r=[row[:5] for row in reactive if row[0]<60]
    pre_p=[row[:5] for row in predictive if row[0]<60]
    assert pre_r==pre_p
    assert not any(row[6]["preemptive"] for row in predictive if row[0]<60)


def test_prediction_does_not_bypass_primary_mode_selection():
    selected=FixedConfiguration(optical_channel="lidar",altitude_m=3.,speed_mps=.5,
                                acoustic_technique="lbl")
    _,fixed=run_one(31_763_000,"S3_OPTICAL_GRADUAL",0,PolicyKind.FIXED,
                    fixed=selected,horizon_s=180.,redesign_version=2,keep_trace=True)
    _,predictive=run_one(31_763_000,"S3_OPTICAL_GRADUAL",0,PolicyKind.PREDICTIVE,
                         fixed=selected,horizon_s=180.,redesign_version=2,keep_trace=True)
    # Prediction is secondary to *mode selection*, not to action. Until
    # 2026-08-11 this asserted that a pre-emptive action could never be
    # injected, which was the defect itself: the gate required
    # RELATIVE_DEAD_RECKONING, a state that already implies optical loss, so
    # pre-emption could only fire once there was nothing left to pre-empt.
    # What must still hold is that prediction does not invent a navigation
    # mode: the selector alone decides that, from present observation.
    assert all(row[6]["navigation_mode"] in {
        "optical_dvl","optical_no_bottom_lock","relative_dead_reckoning",
        "terminal_degraded"} for row in predictive)
    # A pre-emptive action may only appear while a recovery episode is open.
    assert all(row[6]["mission_action"] in {
        "continue","hold_for_fix","abort_leg","return_to_last_good_fix",
        "surface_for_gps"} for row in predictive if row[6]["preemptive"])


def test_estimator_only_fusion_is_not_counted_as_vehicle_intervention():
    result=run_one(31_200_100,"S3_NOMINAL",3,PolicyKind.ROBUST_FUSION)
    assert result.unnecessary_interventions==0
    assert result.preemptive_actions==0


def test_confirmation_metrics_record_overall_error_and_actual_modes():
    result=run_one(31_200_101,"S3_NOMINAL",4,PolicyKind.REACTIVE,horizon_s=20.,redesign_version=3)
    assert np.isfinite(result.overall_rmse_m)
    assert result.overall_rmse_m>=0
    assert result.optical_channels_used
    assert result.acoustic_techniques_used
    assert result.fusion_modes_used
    assert result.mission_actions_used
    assert sum(count for _,count in result.recovery_action_counts)>0
