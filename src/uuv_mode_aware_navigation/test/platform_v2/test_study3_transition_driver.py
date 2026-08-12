from dataclasses import replace
from pathlib import Path

from uuv_mode_aware_navigation.study3 import (
    FixedConfiguration,PolicyKind,Study3Policy,TransitionPhase,TransitionScenario,
    TransitionTarget,deployment_informed_transition_configuration,
    load_transition_scenario,run_one,standard_transition_scenarios,
    truth_side_best_viable_mode,
)


ROOT=31_890_000
BASE=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                        acoustic_technique="usbl",fusion_mode="weight")


def execute(scenario,kind=PolicyKind.REACTIVE,fixed=BASE,policy_factory=None):
    return run_one(ROOT,scenario.name,0,kind,fixed,horizon_s=scenario.horizon_s,
                   dt_s=2.,image_period_s=4.,keep_trace=True,redesign_version=3,
                   policy_factory=policy_factory,transition_scenario=scenario)


def test_physical_phases_and_truth_side_best_mode_change_as_declared():
    for scenario in standard_transition_scenarios().values():
        modes=[]
        for expected in scenario.expectations:
            state=scenario.state_at(expected.after_s)
            actual=truth_side_best_viable_mode(state)
            assert actual==expected.expected_mode
            modes.append(actual)
        assert len(set(modes))>1


def test_physical_changes_reach_actual_optical_dvl_acoustic_and_current_generators():
    scenarios=standard_transition_scenarios()
    optical=scenarios["optical_lbl_recovery"]
    assert optical.state_at(0.).turbidity<optical.state_at(48.).turbidity
    assert optical.state_at(64.).response_probability("lbl")<.5
    assert optical.state_at(64.).response_probability("usbl")>.5
    dvl=scenarios["dvl_acoustic_handover"]
    assert dvl.state_at(44.).dvl_lock_probability<.1
    assert dvl.state_at(44.).dvl_water_track_probability>.5
    assert dvl.state_at(44.).current_north_mps>.05
    assert dvl.state_at(68.).deployed_acoustic_services==frozenset({"lbl"})


def test_fixed_configuration_remains_fixed_through_service_handover():
    scenario=standard_transition_scenarios()["dvl_acoustic_handover"]
    result,trace=execute(scenario,PolicyKind.DEPLOYMENT_FIXED,BASE)
    assert result.acoustic_techniques_used==("usbl",)
    assert {row[6]["acoustic_technique"] for row in trace}=={"usbl"}
    assert {row[6]["optical_channel"] for row in trace}=={"lidar"}
    assert {row[6]["altitude_m"] for row in trace}=={5.}


def test_deployment_informed_choice_uses_launch_deployment_not_future_handover():
    scenarios=standard_transition_scenarios()
    assert deployment_informed_transition_configuration(
        BASE,scenarios["optical_lbl_recovery"]).acoustic_technique=="lbl"
    assert deployment_informed_transition_configuration(
        BASE,scenarios["dvl_acoustic_handover"]).acoustic_technique=="usbl"


def test_json_example_loads_to_the_same_declared_mode_sequence():
    # `__file__`-relative so the suite is invariant to the working directory.
    scenario=load_transition_scenario(
        Path(__file__).resolve().parents[4]
        /"experiments/study3/examples/custom_transition_sequence.json")
    assert [truth_side_best_viable_mode(scenario.state_at(x.after_s))
            for x in scenario.expectations]==[x.expected_mode for x in scenario.expectations]


def test_reactive_has_no_advance_knowledge_of_different_future_scripts():
    original=standard_transition_scenarios()["dvl_acoustic_handover"]
    alternate=replace(original,phases=original.phases[:-1]+(
        TransitionPhase("different_future",82.,102.,TransitionTarget(
            dvl_lock_probability=.03,current_north_mps=.18)),))
    for kind in (PolicyKind.REACTIVE,PolicyKind.PREDICTIVE):
        _,trace_a=execute(original,kind)
        _,trace_b=execute(alternate,kind)
        # Same hidden state and paired streams through 80 s must produce
        # identical observations/actions despite different future events.
        assert [row for row in trace_a if row[0]<=80.]==[row for row in trace_b if row[0]<=80.]


def test_reactive_service_mode_changes_the_applied_configuration_and_fixes():
    scenario=standard_transition_scenarios()["dvl_acoustic_handover"]
    result,trace=execute(scenario)
    transitions=[(row[0],row[6]["navigation_mode"],row[6]["acoustic_technique"])
                 for row in trace]
    assert any(t>=54. and mode=="lbl_aided" and technique=="lbl"
               for t,mode,technique in transitions)
    assert result.acoustic_techniques_used==("lbl","usbl")
    assert result.acoustic_fixes>0


def test_mode_selection_changes_estimator_inputs_not_only_the_logged_label():
    class LabelOnlyAcousticPolicy(Study3Policy):
        def step(self,observable):
            action,output=super().step(observable)
            return replace(action,acoustic_technique=self.fixed.acoustic_technique),output
    scenario=standard_transition_scenarios()["dvl_acoustic_handover"]
    real,_=execute(scenario)
    label_only,_=execute(scenario,policy_factory=LabelOnlyAcousticPolicy)
    # Selecting the newly responding LBL yields real delayed estimator updates;
    # retaining only the new label while holding USBL does not.
    assert real.acoustic_fixes>label_only.acoustic_fixes
    assert real.acoustic_techniques_used==("lbl","usbl")
    assert label_only.acoustic_techniques_used==("usbl",)


def test_capability_recovery_returns_to_stronger_onboard_mode_from_observations():
    scenario=standard_transition_scenarios()["optical_lbl_recovery"]
    fixed=replace(BASE,acoustic_technique="lbl")
    _,trace=execute(scenario,fixed=fixed)
    assert any(row[0]>=58. and row[6]["navigation_mode"]=="usbl_aided" and
               row[6]["acoustic_technique"]=="usbl" for row in trace)
    assert any(row[0]>=98. and row[6]["navigation_mode"]=="optical_dvl"
               for row in trace)


def test_no_viable_submerged_mode_enters_terminal_and_surfaces_for_gps():
    scenario=standard_transition_scenarios()["compound_terminal"]
    result,trace=execute(scenario)
    assert any(row[6]["navigation_mode"]=="terminal_degraded" for row in trace)
    assert trace[-1][6]["mission_action"]=="surface_for_gps"
    assert result.surfaced_for_gps and result.gps_reacquired
    assert result.post_gps_mission_rule=="terminate_after_gps_reacquisition"
