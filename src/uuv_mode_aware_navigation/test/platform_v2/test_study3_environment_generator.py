from dataclasses import fields,replace
from pathlib import Path

from uuv_mode_aware_navigation.study3 import (
    FixedConfiguration,PolicyKind,deployment_informed_environment_configuration,
    generate_environment,load_environment_config,run_one,
    truth_side_best_viable_mode,
)


#: Resolved from this file, not the process working directory, so the suite
#: runs identically from the repository root and from the package directory.
#: The rest of test/platform_v2 already uses this `__file__`-relative pattern.
REPOSITORY_ROOT=Path(__file__).resolve().parents[4]
CONFIG_PATH=REPOSITORY_ROOT/"experiments/study3/examples/moderate_severe_variable_environment.json"
SEED=31_891_000
ROOT=31_891_100
FIXED=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                         acoustic_technique="usbl",fusion_mode="weight")


def realization(seed=SEED,horizon=120.):
    return generate_environment(load_environment_config(CONFIG_PATH),seed,horizon,2.)


def execute(environment,kind):
    fixed=deployment_informed_environment_configuration(FIXED,environment)
    return run_one(ROOT,environment.config.name,0,kind,fixed,
        horizon_s=environment.horizon_s,dt_s=environment.dt_s,image_period_s=4.,
        keep_trace=True,redesign_version=3,environment_realization=environment)


def test_configuration_contains_ranges_and_hazards_but_no_event_times():
    config=load_environment_config(CONFIG_PATH)
    names={x.name for x in fields(config)}
    assert not any("time" in x or "second" in x or "schedule" in x for x in names)
    assert config.turbidity.minimum==.05 and config.turbidity.maximum==.90
    assert config.usbl_infrastructure.failure_hazard_per_s>0
    assert config.usbl_infrastructure.recovery_hazard_per_s>0


def test_same_seed_is_byte_identical_and_different_seed_changes_realization():
    a=realization();b=realization();c=realization(SEED+1)
    assert a.digest==b.digest and a.frames==b.frames
    assert a.digest!=c.digest and a.frames!=c.frames


def test_failure_and_recovery_times_are_seeded_outcomes_not_fixed_schedule():
    a=realization(SEED,180.);b=realization(SEED+1,180.)
    def changes(environment,attribute):
        values=[getattr(x,attribute) for x in environment.frames]
        return [i for i in range(1,len(values)) if values[i]!=values[i-1]]
    assert changes(a,"usbl_deployed")
    assert changes(a,"usbl_deployed")!=changes(b,"usbl_deployed")
    assert changes(a,"dvl_healthy")!=changes(b,"dvl_healthy")


def test_latent_environment_is_paired_but_altitude_changes_physical_dvl_capability():
    environment=realization();assert environment.frame_at(10)==environment.frame_at(10)
    low=environment.physical_state(10,altitude_m=2.,position_xy=(0.,0.))
    high=environment.physical_state(10,altitude_m=7.,position_xy=(0.,0.))
    assert low.dvl_lock_probability>high.dvl_lock_probability
    near=environment.physical_state(10,altitude_m=2.,position_xy=(0.,0.))
    far=environment.physical_state(10,altitude_m=2.,position_xy=(100.,100.))
    assert near.vessel_offset_m<far.vessel_offset_m


def test_generated_realization_contains_online_capability_losses_and_recoveries():
    environment=realization(SEED,180.);modes=[]
    for i in range(len(environment.frames)):
        state=environment.physical_state(i,altitude_m=5.,position_xy=(.1*i,0.))
        modes.append(truth_side_best_viable_mode(state))
    assert len(set(modes))>=4
    assert any(a!=b for a,b in zip(modes,modes[1:]))
    first_loss=next(i for i,m in enumerate(modes) if m!=modes[0])
    assert any(m==modes[0] for m in modes[first_loss+1:])


def test_fixed_and_reactive_use_same_immutable_environment_realization():
    environment=realization()
    fixed,_=execute(environment,PolicyKind.DEPLOYMENT_FIXED)
    reactive,_=execute(environment,PolicyKind.REACTIVE)
    assert environment.digest==realization().digest
    assert fixed.acoustic_techniques_used==("lbl",)
    assert set(reactive.acoustic_techniques_used)=={"lbl","usbl"}


def test_future_latent_changes_cannot_affect_reactive_or_predictive_prefix():
    original=realization();split=40
    changed_frames=original.frames[:split]+tuple(reversed(original.frames[split:]))
    alternate=replace(original,frames=changed_frames,digest="different_future")
    for kind in (PolicyKind.REACTIVE,PolicyKind.PREDICTIVE):
        _,a=execute(original,kind);_,b=execute(alternate,kind)
        cutoff=(split-1)*original.dt_s
        assert [x for x in a if x[0]<=cutoff]==[x for x in b if x[0]<=cutoff]


def test_deployment_informed_fixed_uses_initial_assets_and_never_switches():
    environment=realization();fixed=deployment_informed_environment_configuration(FIXED,environment)
    assert fixed.acoustic_technique=="lbl"
    result,trace=execute(environment,PolicyKind.DEPLOYMENT_FIXED)
    assert result.acoustic_techniques_used==("lbl",)
    assert {row[6]["acoustic_technique"] for row in trace}=={"lbl"}


def test_reactive_online_mode_change_alters_applied_acoustic_configuration():
    environment=realization();result,trace=execute(environment,PolicyKind.REACTIVE)
    assert set(result.acoustic_techniques_used)=={"lbl","usbl"}
    assert any(row[6]["navigation_mode"]=="usbl_aided" and
               row[6]["acoustic_technique"]=="usbl" for row in trace)
