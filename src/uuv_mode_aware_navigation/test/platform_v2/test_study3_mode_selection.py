import math
import inspect

from uuv_mode_aware_navigation.platform_v2 import AcousticServiceEvidence
from uuv_mode_aware_navigation.study3 import (
    FixedConfiguration,NavigationMode,ObservableModeSelector,PolicyKind,run_one,
)


def service(name,responding=True,gives_position=True):
    return AcousticServiceEvidence(name,responding,gives_position,1.2,.1,0.)


def withdrawn(name):
    return AcousticServiceEvidence(name,False,True,math.inf,math.inf,0.)
# ``simulation.py`` submits infinite DOP and sigma when a probe finds no usable
# geometry -- a withdrawn or unusable service -- and finite ones when the fix
# existed but the packet dropped. Since 2026-08-11 the selector honours that
# distinction, so a test for genuine loss must use the former. An empty evidence
# tuple means "not re-probed yet", which is no longer read as loss.



def select(selector,time,optical,velocity,bottom=True,water=False,services=(),terminal=False):
    return selector.select(time,optical_probability=optical,velocity_probability=velocity,
                           dvl_bottom_lock=bottom,dvl_water_track=water,
                           services=services,terminal=terminal)


def test_selector_interface_has_no_truth_or_scenario_input():
    fields=set(inspect.signature(ObservableModeSelector.select).parameters)
    assert not fields.intersection({"truth","true_pose","scenario","family","turbidity","future"})


def test_optical_loss_selects_observable_lbl_not_spatial_recovery_mode():
    selector=ObservableModeSelector(minimum_hold_s=0.)
    assert select(selector,0,.9,.9).mode is NavigationMode.OPTICAL_DVL
    decision=select(selector,1,.05,.9,services=(service("lbl"),))
    assert decision.mode is NavigationMode.LBL_AIDED
    assert decision.absolute_source=="lbl" and not decision.fallback_required


def test_bottom_lock_loss_retains_optical_absolute_mode():
    selector=ObservableModeSelector(minimum_hold_s=0.)
    select(selector,0,.9,.9)
    decision=select(selector,1,.9,.4,bottom=False,water=True)
    assert decision.mode is NavigationMode.OPTICAL_NO_BOTTOM_LOCK
    assert not decision.fallback_required


def test_usbl_departure_selects_onboard_mode_without_oracle_label():
    selector=ObservableModeSelector(minimum_hold_s=0.)
    assert select(selector,0,.8,.8,services=(service("usbl"),)).mode is NavigationMode.USBL_AIDED
    decision=select(selector,1,.8,.8,services=(withdrawn("usbl"),))
    assert decision.mode is NavigationMode.OPTICAL_DVL


def test_observed_capability_recovery_restores_multimodal_mode():
    selector=ObservableModeSelector(minimum_hold_s=0.)
    assert select(selector,0,.05,.8,services=()).mode is NavigationMode.RELATIVE_DEAD_RECKONING
    assert select(selector,1,.8,.8,services=()).mode is NavigationMode.OPTICAL_DVL


def test_no_absolute_aid_enters_relative_then_terminal_safety():
    selector=ObservableModeSelector(minimum_hold_s=0.)
    relative=select(selector,0,.05,.8,services=(service("single_beacon",True,False),))
    assert relative.mode is NavigationMode.RELATIVE_DEAD_RECKONING
    assert relative.absolute_source is None and not relative.fallback_required
    assert relative.velocity_source=="bottom_lock_dvl"
    assert select(selector,31,.05,.05,bottom=False,services=(),terminal=True).mode is NavigationMode.TERMINAL_DEGRADED


def test_dvl_loss_changes_mode_without_unnecessary_altitude_recovery():
    fixed=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                             acoustic_technique="usbl",fusion_mode="weight")
    result,trace=run_one(31_826_000,"S3_DVL_GRADUAL",0,PolicyKind.REACTIVE,
                         fixed,horizon_s=120.,redesign_version=3,keep_trace=True)
    assert "optical_no_bottom_lock" in result.navigation_modes_used
    rows=[row for row in trace if row[6]["navigation_mode"]=="optical_no_bottom_lock"]
    assert rows
    assert all(row[6]["altitude_m"]==fixed.altitude_m for row in rows)
    assert all(row[6]["mission_action"]=="continue" for row in rows)


def test_lbl_mode_changes_actual_acoustic_fusion_behavior():
    fixed=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                             acoustic_technique="usbl",fusion_mode="weight")
    baseline=run_one(31_826_001,"S3_ACOUSTIC_GEOMETRY_ASYNC",0,PolicyKind.FIXED,
                     fixed,horizon_s=60.,redesign_version=3)
    reactive=run_one(31_826_001,"S3_ACOUSTIC_GEOMETRY_ASYNC",0,PolicyKind.REACTIVE,
                     fixed,horizon_s=60.,redesign_version=3)
    assert "lbl_aided" in reactive.navigation_modes_used
    assert reactive.acoustic_fixes>baseline.acoustic_fixes
    assert reactive.longest_unaided_gap_s<baseline.longest_unaided_gap_s


def test_nominal_mode_is_stable_without_spatial_recovery():
    fixed=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                             acoustic_technique="usbl",fusion_mode="weight")
    result,trace=run_one(31_826_002,"S3_NOMINAL",0,PolicyKind.REACTIVE,
                         fixed,horizon_s=60.,redesign_version=3,keep_trace=True)
    # Serialized discovery starts in an onboard mode; LBL becomes selectable
    # only after a real probe response arrives.
    assert "lbl_aided" in result.navigation_modes_used
    assert all(row[6]["mission_action"]=="continue" for row in trace)
    assert all(row[6]["altitude_m"]==fixed.altitude_m for row in trace)
