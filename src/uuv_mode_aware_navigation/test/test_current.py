"""Ocean-current estimation, compensation, and loss of observability.

The paper's title claims adaptation to ocean currents. These tests check the
three things that claim rests on: that the current is recovered from sensing
rather than supplied, that recovering it improves the track, and that when it
stops being observable the vehicle knows.
"""

import numpy as np
import pytest

from uuv_mode_aware_navigation.campaign import (
    DEVELOPMENT_SEED_ROOT,
    CurrentProfile,
    Scenario,
    WaterProfile,
    run_scenario,
)
from uuv_mode_aware_navigation.comparators import FixedPolicy
from uuv_mode_aware_navigation.estimator import NavigationFilter
from uuv_mode_aware_navigation.manager import VehicleConfiguration
from uuv_mode_aware_navigation.mission import (
    MAX_WATER_SPEED_MPS,
    Guidance,
    SurveyMission,
    saturate_command,
)
from uuv_mode_aware_navigation.modes import Mode, Observables, infer_capability
from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS
from uuv_mode_aware_navigation.sensors import (
    FaultKind,
    FaultSchedule,
    FaultWindow,
    SensorSuite,
    total_dvl_loss_schedule,
)

STILL = np.array([0.0, 0.0, 9.81])


def _run_filter(filt, ticks, velocity, current, rng, bottom=True, water=True):
    for _ in range(ticks):
        filt.predict(STILL, 0.1)
        if bottom:
            filt.update_velocity(velocity + rng.normal(0.0, 0.0025, 3))
        if water:
            filt.update_water_velocity(
                velocity - current + rng.normal(0.0, 0.025, 3)
            )


# ---------------------------------------------------------------------------
# Estimation
# ---------------------------------------------------------------------------
def test_current_is_recovered_from_the_two_dvl_modes():
    """Bottom track minus water track is the flow, and the filter finds it."""
    filt = NavigationFilter()
    truth = np.array([0.15, -0.08, 0.0])
    _run_filter(filt, 400, np.array([0.4, 0.1, 0.0]), truth,
                np.random.default_rng(20_000_801))
    assert np.linalg.norm(filt.current - truth) < 0.02, filt.current


def test_the_filter_is_not_told_the_current():
    """Rule N2 for the current: it enters the sensor layer and stops there."""
    import inspect

    for method in (NavigationFilter.update_velocity,
                   NavigationFilter.update_water_velocity):
        params = set(inspect.signature(method).parameters)
        assert "current" not in params and "true_current" not in params, params


def test_current_uncertainty_grows_when_neither_dvl_mode_reports():
    """Losing both modes must show up as doubt, not as a confident stale value.

    This is the quantity that distinguishes "the flow is weak" from "the flow
    was weak when I last had a way to measure it".
    """
    filt = NavigationFilter()
    truth = np.array([0.15, -0.08, 0.0])
    rng = np.random.default_rng(20_000_802)
    _run_filter(filt, 400, np.array([0.4, 0.1, 0.0]), truth, rng)

    locked = filt.current_covariance_trace
    held = filt.current.copy()
    for _ in range(1000):  # 100 s with no DVL at all
        filt.predict(STILL, 0.1)

    assert filt.current_covariance_trace > 10.0 * locked
    assert np.array_equal(filt.current, held), "the estimate moved with no data"


def test_water_track_alone_does_not_manufacture_ground_velocity():
    """With no bottom track from the start, v and c are not separable.

    Water track constrains ``v - c`` only. A filter that treated it as a ground
    velocity would report a confident current it has no way to know, so the
    covariance must stay large.
    """
    filt = NavigationFilter()
    rng = np.random.default_rng(20_000_803)
    _run_filter(filt, 400, np.array([0.4, 0.1, 0.0]), np.array([0.15, -0.08, 0.0]),
                rng, bottom=False)
    assert filt.current_covariance_trace > 0.1, filt.current_covariance_trace


# ---------------------------------------------------------------------------
# Compensation
# ---------------------------------------------------------------------------
def test_compensation_commands_water_relative_velocity():
    command = saturate_command(np.array([0.5, 0.0, 0.0]), np.array([0.0, 0.2, 0.0]))
    assert command == pytest.approx([0.5, -0.2, 0.0]), command


def test_compensation_cannot_exceed_the_thrust_limit():
    """Crabbing costs thrust, and the vehicle has a finite amount of it.

    Without this bound the compensation would be free, current strength would
    stop mattering, and the claim to adapt to currents would be a claim about an
    unphysical vehicle.
    """
    command = saturate_command(np.array([0.5, 0.0, 0.0]), np.array([-2.0, 0.0, 0.0]))
    assert float(np.linalg.norm(command)) == pytest.approx(MAX_WATER_SPEED_MPS)


def test_guidance_crabs_into_a_beam_current():
    mission = SurveyMission()
    guidance = Guidance(mission)
    guidance.index = 1
    position = mission.waypoints[0].copy()
    straight = guidance.command(position, 0.5, 3.0)
    guidance.index = 1
    crabbed = guidance.command(position, 0.5, 3.0,
                               current_estimate_mps=np.array([0.0, 0.15, 0.0]))
    assert crabbed[1] < straight[1] - 0.1, (straight, crabbed)


def test_compensation_improves_the_track_in_a_real_run():
    """The end-to-end claim, scored on truth.

    The same scenario is flown twice by the same policy through the same shared
    guidance, differing only in whether the estimated current is fed forward.
    """
    mission = SurveyMission()
    config = VehicleConfiguration(CAMERA_OFFAXIS, 3.0, 0.5)
    scenario = Scenario(
        "current", DEVELOPMENT_SEED_ROOT + 810, WaterProfile.constant(0.20),
        FaultSchedule(), mission=mission,
        current=CurrentProfile.constant((0.15, -0.10, 0.0)),
    )
    compensated = run_scenario(scenario, FixedPolicy(config)).outcome

    import uuv_mode_aware_navigation.campaign as campaign_module

    original = campaign_module.Guidance.command

    def uncompensated(self, position, speed, altitude, radius=None,
                      current_estimate_mps=None):
        return original(self, position, speed, altitude, radius, None)

    campaign_module.Guidance.command = uncompensated
    try:
        plain = run_scenario(scenario, FixedPolicy(config)).outcome
    finally:
        campaign_module.Guidance.command = original

    assert compensated.rms_cross_track_m < plain.rms_cross_track_m, (
        f"compensated {compensated.rms_cross_track_m:.3f} vs "
        f"uncompensated {plain.rms_cross_track_m:.3f}"
    )


# ---------------------------------------------------------------------------
# Capability
# ---------------------------------------------------------------------------
def test_water_track_survival_is_less_severe_than_total_dvl_loss():
    """A vehicle still measuring its motion through the water is not dead
    reckoning, and the mode must say so."""
    common = dict(
        optical_quality=0.1, optical_available=False, dvl_bottom_lock=False,
        dvl_age_s=5.0, acoustic_fix_age_s=120.0, imu_age_s=0.0, depth_age_s=0.0,
        position_covariance_trace=0.5, covariance_growth_rate=0.0,
    )
    degraded, _ = infer_capability(Observables(**common, dvl_water_track=True))
    lost, reason = infer_capability(Observables(**common, dvl_water_track=False))

    assert lost is Mode.DR_CRITICAL, reason
    assert degraded is not Mode.DR_CRITICAL


def test_total_dvl_loss_schedule_removes_both_modes():
    suite = SensorSuite(schedule=total_dvl_loss_schedule(50.0, 100.0),
                        seed=20_000_804)
    reading = suite.sample(80.0, np.array([0.0, 0.0, -17.0]), np.zeros(3),
                           np.zeros(3), 3.0, WaterProfile.constant(0.2).at(0.0),
                           true_current_mps=(0.1, 0.0, 0.0))
    assert reading.dvl_velocity_mps is None
    assert reading.dvl_water_velocity_mps is None


def test_bottom_lock_loss_alone_leaves_water_track_working():
    suite = SensorSuite(
        schedule=FaultSchedule(
            windows=(FaultWindow(FaultKind.DVL_BOTTOM_LOCK_LOSS, 50.0, 100.0),)
        ),
        seed=20_000_805,
    )
    reading = suite.sample(80.0, np.array([0.0, 0.0, -17.0]), np.zeros(3),
                           np.zeros(3), 3.0, WaterProfile.constant(0.2).at(0.0),
                           true_current_mps=(0.1, 0.0, 0.0))
    assert reading.dvl_velocity_mps is None
    assert reading.dvl_water_velocity_mps is not None


# ---------------------------------------------------------------------------
# The DVL error model that makes dead reckoning drift
# ---------------------------------------------------------------------------
def test_dvl_systematic_errors_are_constant_within_a_scenario():
    """Scale and misalignment must be drawn once, not per tick.

    Redrawn each tick they would average out and behave as extra white noise,
    dead reckoning would not drift, and absolute aiding would have nothing to
    contribute -- which is exactly the defect this model was added to fix.
    """
    suite = SensorSuite(seed=20_000_806)
    velocity = np.array([0.5, 0.0, 0.0])
    a = suite._apply_dvl_errors(velocity)
    b = suite._apply_dvl_errors(velocity)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, velocity), "no systematic error applied at all"


def test_both_dvl_modes_share_the_same_distortion():
    """One transducer head, one scale factor and one mounting rotation.

    Because the distortion is common, it largely cancels in the difference the
    current estimate is built from while remaining in the dead-reckoned position.
    """
    suite = SensorSuite(seed=20_000_807)
    truth_v = np.array([0.5, 0.1, 0.0])
    truth_c = np.array([0.12, -0.05, 0.0])
    bottom, _ = suite._dvl(0.0, truth_v)
    water = suite._dvl_water_track(0.0, truth_v, truth_c)
    # Noise aside, the difference recovers the distorted current, and the
    # distortion of a small vector is small.
    assert np.linalg.norm((bottom - water) - truth_c) < 0.1
