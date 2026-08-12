import numpy as np

from uuv_mode_aware_navigation.delayed_estimator import DelayedPositionMeasurement,FixedLagNavigationFilter
from uuv_mode_aware_navigation.estimator import FusionMode,GRAVITY


ZERO_SPECIFIC_FORCE=-GRAVITY


def _propagate(estimator,steps=30):
    for _ in range(steps):
        estimator.predict(ZERO_SPECIFIC_FORCE,.1)
        estimator.update_velocity(np.array([1.,0,0]))


def test_delayed_replay_matches_chronological_update() -> None:
    chronological=FixedLagNavigationFilter(fixed_lag_s=5); delayed=FixedLagNavigationFilter(fixed_lag_s=5)
    for i in range(30):
        for estimator in (chronological,delayed):
            estimator.predict(ZERO_SPECIFIC_FORCE,.1); estimator.update_velocity(np.array([1.,0,0]))
        if i==9: chronological.update_position(np.array([1.,0,-17]),.1)
    result=delayed.update_delayed_acoustic_position(DelayedPositionMeasurement(1.,3.,np.array([1.,0,-17]),.1))
    assert result.accepted
    assert np.allclose(delayed.position,chronological.position,atol=1e-12)
    assert np.allclose(delayed.P,chronological.P,atol=1e-12)


def test_stale_and_not_yet_arrived_measurements_fail_closed() -> None:
    estimator=FixedLagNavigationFilter(fixed_lag_s=1);_propagate(estimator)
    stale=estimator.update_delayed_acoustic_position(DelayedPositionMeasurement(1,3,np.zeros(3),.1))
    future=estimator.update_delayed_acoustic_position(DelayedPositionMeasurement(3,4,np.zeros(3),.1))
    assert stale.reason=="outside_fixed_lag" and future.reason=="not_arrived"


def test_dropped_packet_never_changes_state() -> None:
    estimator=FixedLagNavigationFilter();_propagate(estimator,10);before=estimator.position.copy()
    result=estimator.update_delayed_acoustic_position(DelayedPositionMeasurement(.5,1,np.ones(3),.1,True))
    assert result.reason=="packet_dropped" and np.array_equal(before,estimator.position)


def test_replay_preserves_fusion_mode_recorded_for_each_update() -> None:
    estimator=FixedLagNavigationFilter(fixed_lag_s=5)
    estimator.filter.fusion=FusionMode.WEIGHT
    estimator.predict(ZERO_SPECIFIC_FORCE,1.)
    surprising=estimator.update_position(np.array([20.,0.,-17.]),.1)
    assert surprising.accepted and surprising.inflated
    delayed=estimator.update_delayed_acoustic_position(
        DelayedPositionMeasurement(.5,1.,np.array([.5,0.,-17.]),.2))
    assert delayed.accepted
    assert estimator.filter.fusion is FusionMode.WEIGHT
