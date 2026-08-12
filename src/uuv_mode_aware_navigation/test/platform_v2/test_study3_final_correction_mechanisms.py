"""Mechanism gates for the final authorized Study-3 DEVELOPMENT correction."""

import math
import numpy as np

from uuv_mode_aware_navigation.delayed_estimator import FixedLagNavigationFilter
from uuv_mode_aware_navigation.platform_v2 import AcousticServiceEvidence
from uuv_mode_aware_navigation.study3 import (
    FixedConfiguration,NavigationMode,ObservableModeSelector,PolicyKind,run_one,
)
from uuv_mode_aware_navigation.study3.scenarios import (
    deployed_acoustic_services,physical_state,
)


FIXED=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                         acoustic_technique="usbl",fusion_mode="weight")


def _service(name,responding=True,gives_position=True):
    return AcousticServiceEvidence(name,responding,gives_position,1.,.2,0.)


def _withdrawn(name):
    return AcousticServiceEvidence(name,False,True,math.inf,math.inf,0.)
# ``simulation.py`` submits infinite DOP and sigma when a probe finds no usable
# geometry -- a withdrawn or unusable service -- and finite ones when the fix
# existed but the packet dropped. Since 2026-08-11 the selector honours that
# distinction, so a test for genuine loss must use the former. An empty evidence
# tuple means "not re-probed yet", which is no longer read as loss.



def test_bottom_and_water_track_are_independent_physical_capabilities():
    gradual=physical_state("S3_DVL_GRADUAL",120.,120.)
    sudden=physical_state("S3_SUDDEN",120.,120.)
    assert gradual.dvl_lock_probability<.1
    assert gradual.dvl_water_track_probability>.8
    assert sudden.dvl_lock_probability<.1
    assert sudden.dvl_water_track_probability<.1


def test_optical_loss_retains_real_bottom_lock_relative_navigation():
    s=ObservableModeSelector(minimum_hold_s=0.)
    decision=s.select(0.,optical_probability=.05,velocity_probability=.9,
                      dvl_bottom_lock=True,dvl_water_track=False,services=())
    assert decision.mode is NavigationMode.RELATIVE_DEAD_RECKONING
    assert decision.velocity_source=="bottom_lock_dvl"
    assert not decision.fallback_required


def test_water_track_is_used_only_when_observed_and_total_loss_is_possible():
    s=ObservableModeSelector(minimum_hold_s=0.)
    water=s.select(0.,optical_probability=.8,velocity_probability=.8,
                   dvl_bottom_lock=False,dvl_water_track=True,services=())
    assert water.mode is NavigationMode.OPTICAL_NO_BOTTOM_LOCK
    assert water.velocity_source=="water_track_dvl"
    none=s.select(1.,optical_probability=.05,velocity_probability=.05,
                  dvl_bottom_lock=False,dvl_water_track=False,services=())
    assert none.velocity_source=="inertial" and none.fallback_required


def test_infrastructure_gates_and_range_only_semantics_remain_exact():
    assert deployed_acoustic_services("S3_OPTICAL_GRADUAL",0.)==frozenset()
    assert deployed_acoustic_services("S3_ACOUSTIC_GEOMETRY_ASYNC",0.)==frozenset({"lbl"})
    assert deployed_acoustic_services("S3_COMPOUND_DVL_ACOUSTIC",0.)==frozenset({"usbl"})
    assert deployed_acoustic_services("S3_INFRASTRUCTURE_WARNING",119.)==frozenset()
    s=ObservableModeSelector(minimum_hold_s=0.)
    d=s.select(0.,optical_probability=.05,velocity_probability=.8,
               dvl_bottom_lock=True,dvl_water_track=True,
               services=(_service("single_beacon",True,False),))
    assert d.mode is NavigationMode.RELATIVE_DEAD_RECKONING


def test_observable_lbl_and_usbl_loss_exit_their_modes():
    for name,mode in (("lbl",NavigationMode.LBL_AIDED),("usbl",NavigationMode.USBL_AIDED)):
        s=ObservableModeSelector(minimum_hold_s=0.)
        assert s.select(0.,optical_probability=.05,velocity_probability=.8,
                        dvl_bottom_lock=True,dvl_water_track=True,
                        services=(_service(name),)).mode is mode
        assert s.select(1.,optical_probability=.05,velocity_probability=.8,
                        dvl_bottom_lock=True,dvl_water_track=True,
                        services=(_withdrawn(name),)).mode is NavigationMode.RELATIVE_DEAD_RECKONING


def test_surface_gps_reinitialization_contract():
    f=FixedLagNavigationFilter(initial_position=(20.,-20.,-5.))
    f.predict(np.zeros(3),1.)
    f.reinitialize_position(np.array([1.,2.,0.]),1.5)
    assert np.allclose(f.position,[1.,2.,0.])
    assert np.allclose(np.diag(f.P)[:3],1.5**2)
    assert f.current_time_s==1.


def test_water_track_uses_water_relative_estimator_update():
    f=FixedLagNavigationFilter(initial_position=(0.,0.,-3.))
    before=f.filter.current_covariance_trace
    outcome=f.update_water_velocity(np.array([.4,0.,0.]))
    assert outcome.accepted
    assert f.filter.current_covariance_trace<before


def test_sustained_unobservable_loss_surfaces_then_gets_gps_and_terminates():
    result,trace=run_one(31_860_000,"S3_NO_RECOVERY",0,PolicyKind.REACTIVE,
                         FIXED,horizon_s=120.,redesign_version=3,keep_trace=True)
    assert "terminal_degraded" in result.navigation_modes_used
    assert "surface_for_gps" in result.mission_actions_used
    assert result.surfaced_for_gps and result.gps_reacquired
    assert result.post_gps_mission_rule=="terminate_after_gps_reacquisition"
    assert math.isfinite(result.gps_pre_reset_error_m)
    assert math.isfinite(result.gps_post_reset_error_m)
    assert trace[-1][7]==0.0


def test_nominal_and_lbl_switching_remain_stable_without_recovery():
    nominal=run_one(31_860_001,"S3_NOMINAL",0,PolicyKind.REACTIVE,FIXED,
                    horizon_s=60.,redesign_version=3)
    assert "lbl_aided" in nominal.navigation_modes_used
    assert nominal.mission_actions_used==("continue",)
    switched=run_one(31_860_002,"S3_ACOUSTIC_GEOMETRY_ASYNC",0,PolicyKind.REACTIVE,
                     FIXED,horizon_s=120.,redesign_version=3)
    assert "lbl_aided" in switched.navigation_modes_used
    assert "relative_dead_reckoning" in switched.navigation_modes_used


def test_recovered_optical_evidence_reenters_optical_mode_without_return_action():
    result,trace=run_one(31_860_003,"S3_RECOVERY",0,PolicyKind.REACTIVE,FIXED,
                         horizon_s=120.,redesign_version=3,keep_trace=True)
    sequence=[]
    for row in trace:
        mode=row[6]["navigation_mode"]
        if not sequence or sequence[-1]!=mode:sequence.append(mode)
    relative=sequence.index("relative_dead_reckoning")
    assert any(mode.startswith("optical_") for mode in sequence[relative+1:])
    assert "return_to_last_good_fix" not in result.mission_actions_used
