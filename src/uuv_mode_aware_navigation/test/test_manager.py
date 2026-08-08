"""Mode-aware manager tests (MODE_MANAGER_SPEC sections 2-5).

The behavioural tests here are the ones that decide whether Paper 2 has a
navigation contribution at all. In particular ``test_ablation_a1_*`` implements
the control behind falsification condition F4.
"""

import numpy as np
import pytest

from uuv_mode_aware_navigation.availability import (
    AvailabilityModel,
    AvailabilitySample,
)
from uuv_mode_aware_navigation.manager import (
    ALTITUDE_FLOOR_M,
    DEFAULT_CANDIDATES,
    ManagerAblation,
    MissionAction,
    MissionCosts,
    ModeAwareManager,
    SPEED_NOMINAL_MPS,
    VehicleConfiguration,
)
from uuv_mode_aware_navigation.modes import Mode, Observables
from uuv_mode_aware_navigation.optics import (
    ALTITUDE_LOW_M,
    ALTITUDE_NOMINAL_M,
    CAMERA_OFFAXIS,
    CONFIGURATIONS,
    WaterState,
    channel_response,
)

ALTITUDES = (1.0, 1.5, 2.0, 2.5, 3.0, 4.0)
TURBIDITIES = (0.15, 0.35, 0.60, 0.90, 1.20, 1.60, 2.00)


@pytest.fixture(scope="module")
def model() -> AvailabilityModel:
    rng = np.random.default_rng(20_000_101)
    samples = []
    for c in TURBIDITIES:
        water = WaterState(c=c)
        for observed_alt in ALTITUDES:
            observed = channel_response(water, observed_alt, CAMERA_OFFAXIS, rng=rng)
            for candidate_alt in ALTITUDES:
                for cfg in CONFIGURATIONS:
                    r = channel_response(water, candidate_alt, cfg, rng=rng)
                    samples.append(
                        AvailabilitySample(
                            quality=observed.quality,
                            observed_altitude_m=observed_alt,
                            candidate_altitude_m=candidate_alt,
                            configuration=cfg.name,
                            available=r.available,
                        )
                    )
    return AvailabilityModel().fit(samples)


def obs(**overrides) -> Observables:
    base = dict(
        optical_quality=0.90,
        optical_available=True,
        dvl_bottom_lock=True,
        dvl_age_s=0.05,
        acoustic_fix_age_s=1.0,
        imu_age_s=0.01,
        depth_age_s=0.01,
        position_covariance_trace=0.05,
        covariance_growth_rate=0.0,
        innovation_exceedance_rate=0.0,
        altitude_m=ALTITUDE_NOMINAL_M,
    )
    base.update(overrides)
    return Observables(**base)


def settle(manager, observation, seconds=12.0, dt=0.5):
    decision = None
    for _ in range(int(seconds / dt)):
        decision = manager.update(observation, dt)
    return decision


# ---------------------------------------------------------------------------
# Basic behaviour
# ---------------------------------------------------------------------------
def test_healthy_vehicle_stays_nominal_and_high(model):
    m = ModeAwareManager(model)
    d = settle(m, obs())
    assert d.mode is Mode.NOMINAL
    assert d.configuration.altitude_m >= ALTITUDE_NOMINAL_M
    assert d.mission_action is MissionAction.CONTINUE


def test_never_offers_an_unsafe_altitude(model):
    for c in DEFAULT_CANDIDATES:
        assert c.altitude_m >= ALTITUDE_FLOOR_M


def test_decision_reports_its_evidence(model):
    m = ModeAwareManager(model)
    d = settle(m, obs(optical_quality=0.1))
    assert d.reason
    assert d.considered > 0
    assert 0.0 <= d.predicted_availability <= 1.0
    assert np.isfinite(d.predicted_uncertainty_m2)


# ---------------------------------------------------------------------------
# The optical-feedback behaviour the title claims
# ---------------------------------------------------------------------------
def test_degraded_optics_provokes_reconfiguration(model):
    """When the camera stops working the manager must change something --
    altitude, channel, or speed. Doing nothing is the failure mode this whole
    project exists to avoid."""
    m = ModeAwareManager(model)
    healthy = settle(m, obs())
    degraded = settle(
        m,
        obs(optical_quality=0.12, optical_available=False,
            covariance_growth_rate=0.05),
    )
    changed = (
        degraded.configuration.optical.name != healthy.configuration.optical.name
        or degraded.configuration.altitude_m != healthy.configuration.altitude_m
        or degraded.configuration.speed_mps != healthy.configuration.speed_mps
    )
    assert changed, "manager did not reconfigure when optical aiding failed"


def test_switches_channel_before_paying_to_descend(model):
    """Descending costs survey swath; switching channel does not. At moderate
    degradation the laser still reaches the seabed from survey altitude, so
    changing channel is the correct and cheaper answer. A manager that dives at
    the first sign of trouble is over-reacting, not being clever."""
    m = ModeAwareManager(model)
    d = settle(
        m,
        obs(optical_quality=0.30, optical_available=False,
            covariance_growth_rate=0.08),
    )
    assert d.configuration.optical.name == "lidar"
    assert d.configuration.altitude_m == ALTITUDE_NOMINAL_M


def test_descends_when_no_channel_reaches_from_survey_altitude(model):
    """The altitude lever is the load-bearing navigation action. It must engage
    when, and only when, changing channel is no longer enough -- water so
    degraded that even the laser cannot reach the seabed from 3 m.

    The observed quality is zero rather than 0.02, and the difference matters.
    At 0.02 the availability model still gives the laser a 0.44 chance from
    survey altitude, so switching channel *is* enough and holding altitude is
    the correct answer; only at zero does that fall to 0.26 and descending
    become worth its cost in swath.

    This test previously used 0.02 and passed, for the wrong reason. The
    incumbent test in the switch-margin hysteresis omitted the acoustic and
    fusion axes, so several candidates answered to "the incumbent" and its
    objective was whichever the search loop evaluated last. That made the
    manager readier to switch than the margin intended, and it descended in a
    condition where it should not have. Correcting the hysteresis made the
    behaviour correct and this test's premise false at the same time.
    """
    m = ModeAwareManager(model)
    d = settle(
        m,
        obs(optical_quality=0.0, optical_available=False,
            covariance_growth_rate=0.08),
    )
    assert d.configuration.altitude_m < ALTITUDE_NOMINAL_M


def test_critical_mode_suspends_the_survey_when_a_fix_could_arrive(model):
    """Tier 3: with velocity aiding gone and the estimate diverging, but acoustic
    fixes still arriving, waiting is the right answer."""
    m = ModeAwareManager(model)
    d = settle(
        m,
        obs(dvl_bottom_lock=False, optical_available=False,
            optical_quality=0.05, acoustic_fix_age_s=1.0,
            covariance_growth_rate=0.2),
    )
    assert d.mode is Mode.DR_CRITICAL
    assert d.mission_action is not MissionAction.CONTINUE


def test_critical_mode_does_not_wait_for_a_fix_that_cannot_arrive(model):
    """Tier 3: holding is only worth its price when there is something to wait for.

    Holding is not free and not neutral. The vehicle station-keeps on its own
    estimate, so a diverging estimate is faithfully converted into physical
    displacement, and the survey clock runs the whole time. With every aiding
    modality down at once -- no bottom lock, no optical, no acoustic for a long
    while -- nothing the vehicle can do or wait for will produce a fix, and the
    hold buys nothing with real money.

    Note what this test does *not* assert. It does not claim that continuing is
    good; the vehicle is in trouble either way. It claims only that waiting for
    something that is not coming is worse.

    ``RETURN_TO_LAST_GOOD_FIX`` is in the tier-3 repertoire but is not the answer
    here either, and the reason is a property of this study rather than of the
    action: degradation is scheduled in **time**, not in space. Turbidity is
    uniform across the survey area and the faults are temporal windows, so a
    position where a fix was obtained ten seconds ago has no better prospect of
    yielding one now than anywhere else. Implementing it would be a no-op dressed
    as a decision. A study with spatially structured degradation -- a turbid
    plume, a beacon shadow -- would need it, and would need to say so.
    """
    m = ModeAwareManager(model)
    d = settle(
        m,
        obs(dvl_bottom_lock=False, optical_available=False,
            optical_quality=0.05, acoustic_fix_age_s=999.0,
            covariance_growth_rate=0.2),
    )
    assert d.mode is Mode.DR_CRITICAL
    assert d.mission_action is MissionAction.CONTINUE


def test_hold_is_bounded_in_time(model):
    """An unbounded hold is a mission failure with extra steps.

    MODE_MANAGER_SPEC section 4 declares a hold timeout. The first implementation
    omitted it, and the vehicle held until the mission time limit -- failing every
    run of the decisive scenario while the ablation with tier 3 switched off
    failed none of them.
    """
    m = ModeAwareManager(model, hold_timeout_s=30.0)
    condition = obs(dvl_bottom_lock=False, optical_available=False,
                    optical_quality=0.05, acoustic_fix_age_s=1.0,
                    covariance_growth_rate=0.2)
    actions = [m.update(condition, 0.5).mission_action for _ in range(200)]
    assert MissionAction.HOLD_FOR_FIX in actions, "never held at all"
    assert actions[-1] is MissionAction.CONTINUE, "hold never released"
    held_s = sum(0.5 for a in actions if a is MissionAction.HOLD_FOR_FIX)
    assert held_s <= 30.5, f"held for {held_s} s against a 30 s timeout"


# ---------------------------------------------------------------------------
# Cost discipline -- the guard against a flattering result
# ---------------------------------------------------------------------------
def test_budget_is_enforced(model):
    m = ModeAwareManager(model, costs=MissionCosts(budget=0.05))
    d = settle(m, obs(optical_available=False, covariance_growth_rate=0.3))
    assert d.rejected_over_budget > 0
    assert d.cost <= MissionCosts().budget


def test_a_healthy_vehicle_does_not_buy_perception_it_does_not_need(model):
    """Minimum altitude with the laser running maximises aiding and would look
    like a triumph. It is also useless: it destroys swath and burns power. A
    healthy vehicle must not choose it."""
    m = ModeAwareManager(model)
    d = settle(m, obs())
    assert d.configuration.altitude_m >= ALTITUDE_NOMINAL_M
    assert d.cost < MissionCosts().budget


def test_cost_rises_as_the_vehicle_descends_and_slows():
    costs = MissionCosts()
    high = costs.evaluate(
        VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS)
    )
    low = costs.evaluate(
        VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_LOW_M, SPEED_NOMINAL_MPS)
    )
    slow = costs.evaluate(
        VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, 0.25)
    )
    assert low > high, "descending must cost swath and risk"
    assert slow > high, "slowing must cost mission time"


def test_suspending_the_mission_is_expensive():
    costs = MissionCosts()
    base = VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS)
    held = VehicleConfiguration(
        CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS,
        MissionAction.HOLD_FOR_FIX,
    )
    assert costs.evaluate(held) > costs.evaluate(base)


# ---------------------------------------------------------------------------
# Ablation A1 -- the decisive control for falsification condition F4
# ---------------------------------------------------------------------------
def test_ablation_a1_cannot_move_the_vehicle(model):
    """A1 keeps mode inference but removes tier 2 and tier 3, leaving
    covariance-only management. If this variant can still change altitude or
    speed, the ablation is not actually ablating anything and F4 would be
    untestable."""
    m = ModeAwareManager(model, ablation=ManagerAblation.covariance_only())
    for observation in (
        obs(),
        obs(optical_available=False, optical_quality=0.1),
        obs(dvl_bottom_lock=False, covariance_growth_rate=0.3),
    ):
        d = settle(m, observation)
        assert d.configuration.altitude_m == ALTITUDE_NOMINAL_M
        assert d.configuration.speed_mps == SPEED_NOMINAL_MPS
        assert d.mission_action is MissionAction.CONTINUE


def test_ablation_a1_still_infers_modes(model):
    """A1 must remain a fair control: it keeps everything except the actions, so
    any difference in outcome is attributable to the actions alone."""
    m = ModeAwareManager(model, ablation=ManagerAblation.covariance_only())
    d = settle(m, obs(dvl_bottom_lock=False))
    assert d.mode is Mode.VELOCITY_AIDING_LOST


def test_full_manager_and_a1_diverge_under_degradation(model):
    """If these two never differ, Paper 2 has no navigation contribution. This
    test is the code-level statement of falsification condition F4."""
    full = ModeAwareManager(model)
    a1 = ModeAwareManager(model, ablation=ManagerAblation.covariance_only())
    observation = obs(
        optical_quality=0.02, optical_available=False, covariance_growth_rate=0.1
    )
    d_full = settle(full, observation)
    d_a1 = settle(a1, observation)
    assert d_full.configuration.name != d_a1.configuration.name


def test_ablation_a2_removes_mission_actions_only(model):
    m = ModeAwareManager(
        model, ablation=ManagerAblation(mission_actions=False)
    )
    d = settle(
        m,
        obs(dvl_bottom_lock=False, optical_available=False,
            optical_quality=0.08, acoustic_fix_age_s=999.0,
            covariance_growth_rate=0.3),
    )
    assert d.mission_action is MissionAction.CONTINUE

    # Guidance must still be available. This previously asserted that the
    # manager descends, as a proxy: with the acoustic axis absent from the
    # objective, losing optical aiding left dead reckoning as the only
    # alternative, so descending was always the answer and "did it descend"
    # stood in for "can it act".
    #
    # That proxy stopped being valid once the acoustic technique entered the
    # objective. The manager now has a genuinely better option here -- USBL
    # bounds position at 0.61 m^2 from nominal altitude, against 0.70 m^2 for
    # the same fix bought by descending -- so declining to descend is the
    # correct decision, not a disabled tier. Asserting the descent would force
    # the manager to pay for an action it has correctly priced as unnecessary.
    #
    # What A2 actually claims is that it removes tier 3 and nothing else, so
    # that is what is checked: the guidance action space is unrestricted, in
    # contrast to A1, which pins altitude and speed to nominal.
    permitted = m._permitted(d.mode)
    assert len({c.altitude_m for c in permitted}) > 1, (
        "A2 pinned the altitude action space; it must remove mission actions only"
    )
    assert len({c.speed_mps for c in permitted}) > 1, (
        "A2 pinned the speed action space; it must remove mission actions only"
    )


def test_ablation_a5_degrades_to_present_tense_only(model):
    """Without the availability model the manager cannot answer counterfactual
    questions; it can only report what is happening now."""
    m = ModeAwareManager(model, ablation=ManagerAblation(availability_model=False))
    d = settle(m, obs(optical_available=False))
    assert d.predicted_availability in (0.0, 1.0)


def test_ablation_a3_removes_stability_machinery(model):
    """A3 must chatter more than the full manager under oscillating evidence,
    or it is not ablating the hysteresis."""
    def run(ablation):
        m = ModeAwareManager(model, ablation=ablation)
        for i in range(200):
            m.update(obs(optical_quality=0.95 if i % 2 == 0 else 0.05), 0.1)
        return m.transitions

    assert run(ManagerAblation(hysteresis=False)) > run(ManagerAblation())


# ---------------------------------------------------------------------------
# Robustness
# ---------------------------------------------------------------------------
def test_fails_closed_on_invalid_observables(model):
    m = ModeAwareManager(model)
    d = settle(m, obs(optical_quality=float("nan")))
    assert d.mode is Mode.DR_CRITICAL


def test_manager_is_deterministic(model):
    def run():
        m = ModeAwareManager(model)
        out = []
        for i in range(80):
            observation = obs(
                optical_quality=0.9 if i < 30 else 0.1,
                optical_available=i < 30,
                dvl_bottom_lock=i < 60,
                covariance_growth_rate=0.0 if i < 30 else 0.05,
            )
            d = m.update(observation, 0.5)
            out.append((d.mode, d.configuration.name, round(d.cost, 9)))
        return out

    assert run() == run()


def test_selected_configuration_is_always_within_budget_or_flagged(model):
    m = ModeAwareManager(model)
    for growth in (0.0, 0.05, 0.2, 0.5):
        d = settle(m, obs(covariance_growth_rate=growth, optical_available=False))
        assert d.cost <= MissionCosts().budget or d.reason == "no_affordable_configuration"


# ---------------------------------------------------------------------------
# Configuration stability
#
# Added after an end-to-end run showed the manager toggling between the laser
# and the camera every ~1.5 s. Mode transitions had hysteresis; configuration
# selection did not, so an arbitrarily small predicted gain justified an
# arbitrarily large reconfiguration.
# ---------------------------------------------------------------------------
def test_switching_configuration_costs_something():
    costs = MissionCosts()
    from uuv_mode_aware_navigation.optics import LIDAR
    current = VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS)
    stay = costs.evaluate(current, current=current)
    swap = costs.evaluate(
        VehicleConfiguration(LIDAR, ALTITUDE_NOMINAL_M, SPEED_NOMINAL_MPS),
        current=current,
    )
    move = costs.evaluate(
        VehicleConfiguration(CAMERA_OFFAXIS, ALTITUDE_LOW_M, SPEED_NOMINAL_MPS),
        current=current,
    )
    assert swap > stay + costs.switch_channel_penalty - 1e-9
    assert move > stay


def test_manager_does_not_chatter_between_channels(model):
    """No real vehicle powers a laser on and off at 0.7 Hz."""
    m = ModeAwareManager(model)
    rng = np.random.default_rng(4242)
    cov, alt = 0.05, ALTITUDE_NOMINAL_M
    channels = []
    for step in range(60):
        t = step / 59.0
        c = 0.20 + (2.00 - 0.20) * min(1.0, max(0.0, (t - 0.15) / 0.5))
        r = channel_response(WaterState(c=c), alt, m._current.optical, rng=rng)
        growth = 0.0 if r.available else 0.06
        cov = max(0.02, cov * 0.7) if r.available else cov + growth * 0.5
        d = m.update(
            Observables(
                optical_quality=r.quality, optical_available=r.available,
                dvl_bottom_lock=True, dvl_age_s=0.05, acoustic_fix_age_s=2.0,
                imu_age_s=0.01, depth_age_s=0.01,
                position_covariance_trace=cov, covariance_growth_rate=growth,
                altitude_m=alt,
            ),
            0.5,
        )
        alt = d.configuration.altitude_m
        channels.append(d.configuration.optical.name)
    swaps = sum(1 for a, b in zip(channels, channels[1:]) if a != b)
    assert swaps <= 6, f"channel chatter: {swaps} swaps in 30 s"
