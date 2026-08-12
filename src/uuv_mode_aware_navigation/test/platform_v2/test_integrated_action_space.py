from pathlib import Path

from uuv_mode_aware_navigation.selection.action_space import ActionSpaceV2, SelectionConditions


ROOT=Path(__file__).resolve().parents[4]
SPACE=ActionSpaceV2.load(ROOT/"benchmarks/platform_v2_action_space.json")


def test_every_speed_is_reachable_for_a_physical_operating_condition():
    conditions=(
        SelectionConditions(exposure_s=.10,texture_scale_m=.02),
        SelectionConditions(),
        SelectionConditions(time_pressure=2.0),
    )
    winners={SPACE.rank_speeds(c)[0].action for c in conditions}
    assert winners=={"speed:0.25","speed:0.50","speed:0.75"}


def test_every_mission_action_is_reachable_without_fault_schedule_labels():
    samples=(
        SelectionConditions(),
        SelectionConditions(fix_expected_s=5),
        SelectionConditions(estimator_drift_m=3),
        SelectionConditions(estimator_drift_m=6),
        SelectionConditions(total_blackout_s=31),
    )
    assert {SPACE.reachable_mission_action(c) for c in samples}==set(SPACE.declaration["mission_actions"])


def test_selector_scores_share_finite_documented_scale():
    for conditions in (SelectionConditions(),SelectionConditions(
            optical_attenuation_m_inv=1.2,collision_risk=.8,time_pressure=1.0)):
        scores=SPACE.rank_speeds(conditions)+SPACE.rank_altitudes(conditions)
        assert all(abs(item.score)<100 for item in scores)
        assert all(item.operational_cost>=0 for item in scores)
