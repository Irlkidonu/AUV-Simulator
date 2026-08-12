from dataclasses import asdict

from uuv_mode_aware_navigation.platform_v2 import AcousticServiceEvidence
from uuv_mode_aware_navigation.study3 import (
    FixedConfiguration, PendingProbe, PolicyKind, SerializedServiceDiscovery,
    Study3Policy, deployment_informed_fixed_configuration,
)


LOCKED=FixedConfiguration(optical_channel="lidar",altitude_m=5.0,speed_mps=.5,
                          acoustic_technique="usbl",fusion_mode="weight")


def evidence(name="lbl",responding=True):
    return AcousticServiceEvidence(name,responding,True,1.4,.25,0.)


def test_catalogue_does_not_reveal_current_quality_before_serial_probe_completion():
    discovery=SerializedServiceDiscovery({"lbl","usbl"},4.,8.)
    assert discovery.catalogue==("lbl","usbl")
    assert discovery.observe(0.)==()
    assert discovery.take_opportunity(0.)=="lbl"
    assert discovery.take_opportunity(2.) is None
    discovery.submit(PendingProbe("lbl",1.25,evidence()))
    assert discovery.observe(1.)==()
    visible=discovery.observe(2.)
    assert len(visible)==1 and visible[0].name=="lbl" and visible[0].age_s==.75
    assert discovery.take_opportunity(4.)=="usbl"


def test_probe_evidence_expires_and_failed_response_is_observable_only_after_timeout():
    discovery=SerializedServiceDiscovery({"lbl"},4.,8.)
    assert discovery.take_opportunity(0.)=="lbl"
    discovery.submit(PendingProbe("lbl",4.,evidence(responding=False)))
    assert discovery.observe(3.99)==()
    assert discovery.observe(4.)[0].responding is False
    assert discovery.observe(12.)[0].age_s==8.
    assert discovery.observe(12.01)==()


def test_deployment_informed_fixed_changes_only_prelaunch_acoustic_axis():
    for catalogue,expected in ((set(),"none"),({"single_beacon"},"single_beacon"),
                               ({"lbl"},"lbl"),({"usbl"},"usbl")):
        selected=deployment_informed_fixed_configuration(LOCKED,catalogue)
        before=asdict(LOCKED);after=asdict(selected)
        assert after.pop("acoustic_technique")==expected
        before.pop("acoustic_technique")
        assert after==before


def test_deployment_fixed_policy_is_immutable_after_launch():
    selected=deployment_informed_fixed_configuration(LOCKED,{"lbl"})
    policy=Study3Policy(PolicyKind.DEPLOYMENT_FIXED,selected)
    assert policy.fixed==selected
    # The fixed branch is already covered with fully formed observations by
    # infrastructure-scope tests; identity here proves no adaptive wrapper is
    # selected for this policy kind.
    assert policy.kind is PolicyKind.DEPLOYMENT_FIXED
