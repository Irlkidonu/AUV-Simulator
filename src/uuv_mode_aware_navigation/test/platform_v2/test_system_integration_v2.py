import math

import numpy as np

from uuv_mode_aware_navigation.capability import CapabilityDegradationPredictor, CapabilityTrendEvidence
from uuv_mode_aware_navigation.estimator import GRAVITY
from uuv_mode_aware_navigation.localization import P5V4CapabilityAdapter, P5V4Configuration
from uuv_mode_aware_navigation.platform_v2 import (AcousticSignal, DVLSignal,
                                                    PlatformStepInput,
                                                    PlatformV2Coordinator)
from uuv_mode_aware_navigation.recovery import RecoveryAction


def optical_result(**changes):
    values=dict(localization_success=True,inliers=40,inlier_fraction=.8,
                median_reprojection_px=.5,alternative_inliers=2,estimated_scale=1.0,
                covariance_eigenvalues_m2=[.0001,.0004])
    values.update(changes)
    return values


def test_selected_p5_v4_parameters_and_fail_closed_adapter():
    config=P5V4Configuration()
    assert (config.detector_threshold,config.minimum_inliers,config.covariance_inflation)==(5e-5,12,2.3)
    adapter=P5V4CapabilityAdapter(config)
    assert adapter.observe(optical_result(),.8).available
    assert not adapter.observe(optical_result(estimated_scale=0.0),.8).available
    assert not adapter.observe(optical_result(covariance_eigenvalues_m2=[.01,.04]),.8).available


def test_prediction_identifies_impending_observable_capability_losses():
    predictor=CapabilityDegradationPredictor(horizon_s=10)
    forecast=predictor.predict(CapabilityTrendEvidence(
        .30,-.01,.02,.30,-.02,1,5,2,True))
    assert {"optical","velocity"} <= forecast.impending
    assert math.isclose(forecast.time_to_loss_s["optical"],5.0)
    assert "acoustic" not in forecast.impending


def test_delayed_acoustic_belief_and_predictive_recovery_share_one_loop():
    coordinator=PlatformV2Coordinator()
    for _ in range(30):
        coordinator.estimator.predict(-GRAVITY,.1)
        coordinator.estimator.update_velocity(np.array([1.,0.,0.]))
    adapter=P5V4CapabilityAdapter()
    optical=adapter.observe(optical_result(),.30)
    acoustic=AcousticSignal(True,1.0,3.0,np.array([1.,0.,-17.]),
                            np.eye(2)*.01,2.0,True,5.0)
    output=coordinator.step(PlatformStepInput(
        3.0,.1,optical,-.01,DVLSignal(True,False,.1,.95),acoustic,
        .1,0.0,3.0,.5,0.0))
    assert output.acoustic_update_accepted and output.delayed_acoustic_reason=="replayed"
    assert output.belief.usable_probability["acoustic"]>.5
    assert "optical" in output.forecast.impending
    assert output.recovery.action is RecoveryAction.LOWER_ALTITUDE
    assert output.recovery.predictive
    assert output.selected_altitude_m<3.0


def test_compound_observable_loss_drives_conservative_belief_without_truth():
    coordinator=PlatformV2Coordinator()
    adapter=P5V4CapabilityAdapter()
    unavailable=adapter.observe({},.05)
    output=None
    for _ in range(5):
        coordinator.estimator.predict(-GRAVITY,.1)
        signal=AcousticSignal(False,coordinator.estimator.current_time_s,
                              coordinator.estimator.current_time_s,None,None,
                              math.inf,False,5.0)
        output=coordinator.step(PlatformStepInput(
            coordinator.estimator.current_time_s,.1,unavailable,-.02,
            DVLSignal(False,False,5.0,.05,-.01),signal,.1,.4,3,.5,0))
    assert output.belief.usable_probability["optical"]<.1
    assert output.belief.usable_probability["velocity"]<.1
    assert output.belief.usable_probability["acoustic"]<.1
