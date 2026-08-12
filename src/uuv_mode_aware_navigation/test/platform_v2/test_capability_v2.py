from uuv_mode_aware_navigation.availability import AvailabilityModel, AvailabilitySample
from uuv_mode_aware_navigation.capability import (
    CapabilityEvidence, ProbabilisticCapabilityFilter, OpticalCandidatePredictor,
)


def test_probabilistic_filter_accumulates_and_recovers_evidence() -> None:
    filt=ProbabilisticCapabilityFilter()
    bad=CapabilityEvidence(.1,False,False,False,100,.1)
    for _ in range(3): belief=filt.update(bad,.5)
    assert belief.usable_probability["optical"]<.5
    good=CapabilityEvidence(.9,True,True,False,1,.1)
    for _ in range(5): belief=filt.update(good,.5)
    assert belief.usable_probability["optical"]>.8


def test_silence_is_operational_unavailability_not_fault_diagnosis() -> None:
    filt=ProbabilisticCapabilityFilter()
    belief=filt.update(CapabilityEvidence(.8,True,True,False,100,.1),1)
    assert belief.usable_probability["acoustic"]<belief.usable_probability["optical"]


def test_candidate_predictor_reports_information_without_selecting() -> None:
    samples=[AvailabilitySample(q,3,3,"camera",q>.5) for q in (.1,.2,.8,.9)]
    model=AvailabilityModel().fit(samples)
    prediction=OpticalCandidatePredictor(model).predict("camera",.8,3,2,.1,1.0)
    assert 0<=prediction.p_accepted<=1
    assert prediction.expected_information_gain>0
    assert not hasattr(prediction,"selected")
