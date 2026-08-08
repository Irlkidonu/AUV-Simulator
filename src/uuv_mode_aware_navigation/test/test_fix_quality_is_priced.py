"""The manager must value a fix it can use, not a fix that merely arrives.

These tests exist because the 108-configuration campaign produced a result that
looked like a defeat and was actually a defect. In the compound scenario --
turbidity and DVL loss together -- the manager obtained *more* aiding than the
best fixed configuration (availability 0.203 against 0.083) and produced an
estimate eleven times worse (7.791 m against 0.682 m), failing every run.

The cause was that ``_projected_uncertainty`` mixed the aided and unaided
branches by the probability that a measurement would *arrive*. With the aided
branch pinned at a constant floor, that objective is monotonically decreasing in
availability: the manager could only ever prefer whichever configuration emitted
the most measurements, however wrong they were. Turbid water and multipath
returns produce measurements that are plentiful and wrong, so the manager chased
them and poisoned its own filter.

Two inputs it needed were also never supplied. ``innovation_exceedance_rate``
defaulted to 0.0 in every run because nothing computed it, which also silently
disabled the mode-classification branch that reads it. ``covariance_growth_rate``
was the constant 0.05 whenever unaided, so the projection of the unaided branch
was identical in still water and in a strong current.
"""

import numpy as np

from uuv_mode_aware_navigation.estimator import EXCEEDANCE_WINDOW, NavigationFilter
from uuv_mode_aware_navigation.modes import Observables


def _obs(**kw):
    base = dict(
        optical_quality=0.8,
        optical_available=True,
        dvl_bottom_lock=True,
        dvl_age_s=0.0,
        acoustic_fix_age_s=0.0,
        imu_age_s=0.0,
        depth_age_s=0.0,
        position_covariance_trace=1.0,
        covariance_growth_rate=0.05,
    )
    base.update(kw)
    return Observables(**base)


def test_exceedance_rate_is_actually_computed():
    """The observable must move. It was hard-wired to zero for the whole study."""
    filt = NavigationFilter()
    assert filt.innovation_exceedance_rate == 0.0, "no measurements yet"

    filt.predict(np.array([0.0, 0.0, 9.81]), 0.1)
    # Measurements the filter cannot reconcile: far outside the gate.
    for _ in range(EXCEEDANCE_WINDOW):
        filt.update_position(filt.position + np.array([50.0, 0.0, 0.0]), 0.10)
    assert filt.innovation_exceedance_rate > 0.9, (
        f"a stream of 50 m outliers produced an exceedance rate of "
        f"{filt.innovation_exceedance_rate:.3f}"
    )


def test_consistent_measurements_give_a_low_exceedance_rate():
    filt = NavigationFilter()
    filt.predict(np.array([0.0, 0.0, 9.81]), 0.1)
    for _ in range(EXCEEDANCE_WINDOW):
        filt.update_position(filt.position, 0.10)
    assert filt.innovation_exceedance_rate < 0.1, (
        "measurements agreeing with the filter were counted as exceedances"
    )


def test_exceedance_rate_tracks_recent_conditions():
    """Bounded window: conditions that have cleared must stop being reported."""
    filt = NavigationFilter()
    filt.predict(np.array([0.0, 0.0, 9.81]), 0.1)
    for _ in range(EXCEEDANCE_WINDOW):
        filt.update_position(filt.position + np.array([50.0, 0.0, 0.0]), 0.10)
    assert filt.innovation_exceedance_rate > 0.9
    for _ in range(EXCEEDANCE_WINDOW):
        filt.update_position(filt.position, 0.10)
    assert filt.innovation_exceedance_rate < 0.1, (
        "the rate never recovered after measurements became consistent"
    )


def test_rejected_fixes_do_not_count_as_aiding():
    """The regression proper: availability alone must not decide the objective.

    Two configurations with identical predicted availability, one whose fixes
    the filter accepts and one whose fixes it rejects, must not be valued the
    same. Before the fix they were, which is why the manager preferred a
    configuration that produced many unusable measurements.
    """
    from uuv_mode_aware_navigation.manager import ModeAwareManager

    manager = ModeAwareManager()
    clean = manager._projected_uncertainty(_obs(innovation_exceedance_rate=0.0), 0.9)
    dirty = manager._projected_uncertainty(_obs(innovation_exceedance_rate=0.9), 0.9)
    assert dirty > clean, (
        f"a configuration whose fixes are 90% rejected was valued no worse "
        f"than one whose fixes are accepted ({dirty:.4f} vs {clean:.4f})"
    )


def test_fully_rejected_aiding_is_worth_nothing():
    """If every fix is thrown away, the projection must equal the unaided case."""
    from uuv_mode_aware_navigation.manager import ModeAwareManager

    manager = ModeAwareManager()
    obs = _obs(innovation_exceedance_rate=1.0)
    projected = manager._projected_uncertainty(obs, 1.0)
    unaided = manager._projected_uncertainty(_obs(innovation_exceedance_rate=0.0), 0.0)
    assert abs(projected - unaided) < 1e-9, (
        "measurements that are all rejected still earned aided credit"
    )


def test_covariance_growth_rate_is_measured_not_constant():
    """It was 0.05 in every unaided run regardless of how fast drift actually was."""
    from collections import deque

    from uuv_mode_aware_navigation.campaign import _covariance_growth_rate

    slow = deque([1.0, 1.1, 1.2, 1.3, 1.4])
    fast = deque([1.0, 3.0, 5.0, 7.0, 9.0])
    r_slow = _covariance_growth_rate(slow, 0.5)
    r_fast = _covariance_growth_rate(fast, 0.5)
    assert r_fast > r_slow * 5, (
        f"a fivefold difference in observed growth produced {r_slow:.4f} "
        f"and {r_fast:.4f}"
    )
    # Shrinking uncertainty is the aided branch's business, not a negative rate.
    assert _covariance_growth_rate(deque([5.0, 4.0, 3.0]), 0.5) == 0.0
    assert _covariance_growth_rate(deque([1.0]), 0.5) == 0.0
