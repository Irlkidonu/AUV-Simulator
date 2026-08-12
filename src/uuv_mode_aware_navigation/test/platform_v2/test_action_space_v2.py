from pathlib import Path

from uuv_mode_aware_navigation.selection import ActionSpaceV2, SelectionConditions


DECLARATION = Path(__file__).parents[4] / "benchmarks" / "platform_v2_action_space.json"


def test_every_speed_is_reachable_for_a_physical_reason() -> None:
    space=ActionSpaceV2.load(DECLARATION)
    assert space.rank_speeds(SelectionConditions(exposure_s=.20,texture_scale_m=.02))[0].action=="speed:0.25"
    assert space.rank_speeds(SelectionConditions())[0].action=="speed:0.50"
    assert space.rank_speeds(SelectionConditions(time_pressure=1.0))[0].action=="speed:0.75"


def test_altitude_trade_changes_with_optics_and_collision_risk() -> None:
    space=ActionSpaceV2.load(DECLARATION)
    assert space.rank_altitudes(SelectionConditions(optical_attenuation_m_inv=1.2))[0].action=="altitude:1.0"
    assert space.rank_altitudes(SelectionConditions(optical_attenuation_m_inv=.02,collision_risk=1.0))[0].action=="altitude:3.0"


def test_every_mission_action_has_a_declared_reachable_condition() -> None:
    space=ActionSpaceV2.load(DECLARATION)
    cases=[
      (SelectionConditions(),"continue"),
      (SelectionConditions(fix_expected_s=5),"hold_for_fix"),
      (SelectionConditions(estimator_drift_m=2.5),"return_to_last_good_fix"),
      (SelectionConditions(estimator_drift_m=6),"abort_leg"),
      (SelectionConditions(total_blackout_s=31),"surface_for_gps"),
    ]
    assert [space.reachable_mission_action(c) for c,_ in cases]==[expected for _,expected in cases]
