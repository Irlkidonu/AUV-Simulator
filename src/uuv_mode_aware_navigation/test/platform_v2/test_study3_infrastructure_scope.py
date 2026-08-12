from dataclasses import replace
import numpy as np

from uuv_mode_aware_navigation.localization import OpticalLocalizationSignal
from uuv_mode_aware_navigation.platform_v2 import (AcousticServiceEvidence,
    AcousticSignal,DVLSignal,PlatformStepInput)
from uuv_mode_aware_navigation.study3 import (
    FAMILY_INFRASTRUCTURE, FixedConfiguration, InfrastructureContext,
    PolicyKind, Study3Policy, deployed_acoustic_services,
)


def observation():
    return PlatformStepInput(0.,1.,OpticalLocalizationSignal(True,.8,.03,0.,30,.8,.4,.1,"available"),
        0.,DVLSignal(True,False,0.,.9,0.),
        AcousticSignal(False,0.,0.,None,None,2.,False,4.),0.,0.,3.,.5,.005)


def test_every_family_has_one_explicit_infrastructure_context():
    assert set(FAMILY_INFRASTRUCTURE)=={
        "S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
        "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
        "S3_COMPOUND_DVL_ACOUSTIC","S3_NOMINAL","S3_SUDDEN","S3_NO_RECOVERY",
    }
    assert set(FAMILY_INFRASTRUCTURE.values())==set(InfrastructureContext)


def test_services_require_their_assets_and_transition_retires_usbl():
    assert deployed_acoustic_services("S3_OPTICAL_GRADUAL",0)==frozenset()
    assert deployed_acoustic_services("S3_ACOUSTIC_GEOMETRY_ASYNC",0)==frozenset({"lbl"})
    assert deployed_acoustic_services("S3_COMPOUND_DVL_ACOUSTIC",0)==frozenset({"usbl"})
    assert deployed_acoustic_services("S3_DVL_GRADUAL",0)==frozenset({"single_beacon"})
    assert deployed_acoustic_services("S3_INFRASTRUCTURE_WARNING",80,120)==frozenset({"usbl"})
    assert deployed_acoustic_services("S3_INFRASTRUCTURE_WARNING",82,120)==frozenset()


def test_adaptive_policy_uses_only_observed_service_not_hidden_context():
    fixed=FixedConfiguration(acoustic_technique="lbl")
    policy=Study3Policy(PolicyKind.REACTIVE,fixed)
    base=observation()
    acoustic=replace(base.acoustic,infrastructure_available=False,
                     observable_services=frozenset({"usbl"}),
                     service_evidence=(AcousticServiceEvidence("usbl",True,True,1.2,.2,0.),))
    action,_=policy.step(replace(base,acoustic=acoustic))
    assert action.acoustic_technique=="usbl"


def test_catalogue_identity_alone_is_not_current_service_quality():
    fixed=FixedConfiguration(acoustic_technique="lbl")
    base=observation()
    acoustic=replace(base.acoustic,infrastructure_available=False,
                     observable_services=frozenset({"usbl"}),service_evidence=())
    action,_=Study3Policy(PolicyKind.REACTIVE,fixed).step(replace(base,acoustic=acoustic))
    assert action.acoustic_technique=="lbl"


def test_fixed_policy_does_not_oracle_switch_to_deployed_service():
    fixed=FixedConfiguration(acoustic_technique="lbl")
    base=observation()
    acoustic=replace(base.acoustic,infrastructure_available=False,
                     observable_services=frozenset({"usbl"}))
    action,_=Study3Policy(PolicyKind.FIXED,fixed).step(replace(base,acoustic=acoustic))
    assert action.acoustic_technique=="lbl"
