from dataclasses import replace

from uuv_mode_aware_navigation.capability.prediction import OpticalEvidenceForecaster
from uuv_mode_aware_navigation.localization import OpticalLocalizationSignal


BASE=OpticalLocalizationSignal(True,.5,.01,0.,80,.95,.3,.02,"available",100,100,90)


def test_texture_quality_changes_alone_cannot_trigger_forecast():
    predictor=OpticalEvidenceForecaster()
    results=[]
    for i,q in enumerate((.9,.3,.8,.2,.7,.4)):
        results.append(predictor.observe(i*4.,replace(BASE,quality=q)))
    assert not any(x.warning for x in results)


def test_sustained_p5_evidence_decline_can_trigger_before_unavailable():
    predictor=OpticalEvidenceForecaster();results=[]
    for i in range(6):
        signal=replace(BASE,inliers=80-i*12,inlier_fraction=.95-i*.09,
                       reprojection_px=.3+i*.25,sigma_m=.01+i*.012,
                       keypoints_a=100-i*8,keypoints_b=100-i*8,matches=90-i*9)
        results.append(predictor.observe(i*4.,signal))
    assert any(x.warning for x in results)
    assert all(BASE.available for _ in results)


def test_abrupt_loss_without_prior_evidence_decline_has_no_warning():
    predictor=OpticalEvidenceForecaster()
    before=[predictor.observe(i*4.,BASE) for i in range(4)]
    lost=predictor.observe(16.,replace(BASE,available=False,reason="rejected_frontend"))
    assert not any(x.warning for x in before)
    assert not lost.warning
