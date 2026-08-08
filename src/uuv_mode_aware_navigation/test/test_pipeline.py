"""End-to-end pipeline tests: sensors, estimator, mission, comparators, campaign.

The information-boundary tests here implement protocol rules N1-N4. They are the
evidence for the claim that this study evaluates *navigation* and not
localization, and they are part of the freeze record.
"""

import inspect
import math

import numpy as np
import pytest

from uuv_mode_aware_navigation import campaign as campaign_module
from uuv_mode_aware_navigation.availability import (
    AvailabilityModel,
    AvailabilitySample,
)
from uuv_mode_aware_navigation.campaign import (
    DEVELOPMENT_SEED_ROOT,
    HELDOUT_SEED_ROOT,
    Scenario,
    WaterProfile,
    run_scenario,
)
from uuv_mode_aware_navigation.comparators import build_policies
from uuv_mode_aware_navigation.estimator import GRAVITY, NavigationFilter
from uuv_mode_aware_navigation.mission import (
    Guidance,
    MissionEvaluator,
    SurveyMission,
    Vehicle,
    lawnmower,
)
from uuv_mode_aware_navigation.modes import Observables
from uuv_mode_aware_navigation.optics import (
    CAMERA_OFFAXIS,
    CONFIGURATIONS,
    WaterState,
    channel_response,
)
from uuv_mode_aware_navigation.sensors import (
    BeaconGeometry,
    FaultKind,
    FaultSchedule,
    SensorSuite,
    compound_schedule,
    coupled_turbidity_dvl_schedule,
    dvl_loss_schedule,
)


@pytest.fixture(scope="module")
def model():
    rng = np.random.default_rng(DEVELOPMENT_SEED_ROOT + 1)
    altitudes = (1.0, 1.5, 2.0, 2.5, 3.0, 4.0)
    samples = []
    for c in (0.15, 0.35, 0.6, 0.9, 1.2, 1.6, 2.0):
        water = WaterState(c=c)
        for ho in altitudes:
            observed = channel_response(water, ho, CAMERA_OFFAXIS, rng=rng)
            for hc in altitudes:
                for cfg in CONFIGURATIONS:
                    samples.append(
                        AvailabilitySample(
                            observed.quality, ho, hc, cfg.name,
                            channel_response(water, hc, cfg, rng=rng).available,
                        )
                    )
    return AvailabilityModel().fit(samples)


# ---------------------------------------------------------------------------
# Inertial convention -- the bug that diverged the estimate by kilometres
# ---------------------------------------------------------------------------
def test_accelerometer_reports_specific_force():
    """At rest an accelerometer reads +9.81 m/s^2 upward, not zero."""
    suite = SensorSuite(seed=DEVELOPMENT_SEED_ROOT)
    reading = suite.sample(
        0.0, np.array([0.0, 0.0, -17.0]), np.zeros(3), np.zeros(3), 3.0,
        WaterState(c=0.2),
    )
    assert reading.accel_mps2[2] == pytest.approx(9.81, abs=0.2)


def test_stationary_dead_reckoning_drifts_slowly_not_catastrophically():
    """Unaided INS must drift like ~0.5*bias*t^2, not diverge by kilometres.

    A sign error between the sensor's specific force and the filter's gravity
    compensation injected 9.81 m/s^2 into every predict step and produced
    kilometre-scale error that no test noticed until the pipeline was run
    end-to-end."""
    suite = SensorSuite(seed=DEVELOPMENT_SEED_ROOT)
    nav = NavigationFilter(initial_position=(0.0, 0.0, -17.0))
    truth = np.array([0.0, 0.0, -17.0])
    for i in range(600):  # 60 s
        reading = suite.sample(
            i * 0.1, truth, np.zeros(3), np.zeros(3), 3.0, WaterState(c=0.2)
        )
        nav.predict(reading.accel_mps2, 0.1)
    drift = float(np.linalg.norm(nav.position - truth))
    assert drift < 30.0, f"inertial divergence: {drift:.1f} m in 60 s"
    assert drift > 0.05, "drift suspiciously small; bias may not be modelled"


# ---------------------------------------------------------------------------
# Filter health
# ---------------------------------------------------------------------------
def test_covariance_stays_symmetric_and_positive_semidefinite():
    suite = SensorSuite(seed=DEVELOPMENT_SEED_ROOT + 2)
    nav = NavigationFilter(initial_position=(0.0, 0.0, -17.0))
    truth = np.array([0.0, 0.0, -17.0])
    for i in range(400):
        r = suite.sample(i * 0.1, truth, np.zeros(3), np.zeros(3), 3.0,
                         WaterState(c=0.2))
        nav.predict(r.accel_mps2, 0.1)
        if r.dvl_velocity_mps is not None:
            nav.update_velocity(r.dvl_velocity_mps)
        nav.update_depth(r.depth_m)
        if r.optical_position_m is not None:
            nav.update_position(r.optical_position_m, r.optical_sigma_m)
    assert np.allclose(nav.P, nav.P.T, atol=1e-9), "covariance lost symmetry"
    assert np.min(np.linalg.eigvalsh(nav.P)) > -1e-9, "covariance not PSD"


def test_aiding_bounds_drift_relative_to_dead_reckoning():
    def drift(aided: bool) -> float:
        suite = SensorSuite(seed=DEVELOPMENT_SEED_ROOT + 3)
        nav = NavigationFilter(initial_position=(0.0, 0.0, -17.0))
        truth = np.array([0.0, 0.0, -17.0])
        for i in range(900):
            r = suite.sample(i * 0.1, truth, np.zeros(3), np.zeros(3), 3.0,
                             WaterState(c=0.2))
            nav.predict(r.accel_mps2, 0.1)
            if aided and r.dvl_velocity_mps is not None:
                nav.update_velocity(r.dvl_velocity_mps)
            if aided:
                nav.update_depth(r.depth_m)
        return float(np.linalg.norm(nav.position - truth))
    assert drift(aided=True) < drift(aided=False)


# ---------------------------------------------------------------------------
# Information boundary -- protocol rules N1-N4
# ---------------------------------------------------------------------------
def test_observables_carry_no_hidden_state():
    """Rule N2. If a quantity is not a field here, the manager cannot reach it."""
    forbidden = {
        "c", "tau", "turbidity", "water", "attenuation", "optical_depth",
        "true_position", "true_position_m", "ground_truth", "schedule",
        "fault_schedule", "bias_m", "optical_bias_m",
    }
    fields = set(Observables.__dataclass_fields__)
    assert not (fields & forbidden), f"hidden state reachable: {fields & forbidden}"


def test_guidance_cannot_receive_true_state():
    """Rule N1. Guidance takes an estimated position and nothing else."""
    params = set(inspect.signature(Guidance.command).parameters)
    forbidden = {"true_position", "truth", "ground_truth", "vehicle", "true_state"}
    assert not (params & forbidden), f"guidance sees truth: {params & forbidden}"
    assert "estimated_position" in params


def test_runner_passes_only_the_estimate_to_guidance():
    """Rule N1, at the call site rather than the signature."""
    source = inspect.getsource(campaign_module.run_scenario)
    call = source.split("guidance.command(", 1)[1].split(")", 1)[0]
    assert "estimator.position" in call
    assert "vehicle.position" not in call, "guidance was handed true position"


def test_evaluator_output_never_feeds_a_decision():
    """Rule N3. The evaluator is scoring only; nothing it returns is consumed."""
    source = inspect.getsource(campaign_module.run_scenario)
    after = source.split("evaluator.record(", 1)[1]
    assert "policy.update" not in after.split("t += scenario.dt")[0]


def test_paper2_seeds_are_disjoint_from_paper1():
    """Rule D1, stated as a floor on Paper 2 rather than a ceiling on Paper 1."""
    assert DEVELOPMENT_SEED_ROOT >= 20_000_000
    assert HELDOUT_SEED_ROOT >= 20_000_000
    assert DEVELOPMENT_SEED_ROOT != HELDOUT_SEED_ROOT


# ---------------------------------------------------------------------------
# Fault injection
# ---------------------------------------------------------------------------
def test_faults_are_binary_not_a_tunable_severity():
    """A dropout is a dropout. No knob controls how bad it is."""
    fields = set(campaign_module.FaultSchedule.__dataclass_fields__)
    assert fields == {"windows"}
    from uuv_mode_aware_navigation.sensors import FaultWindow
    assert set(FaultWindow.__dataclass_fields__) == {"kind", "start_s", "duration_s"}


def test_dvl_outage_actually_removes_velocity_aiding():
    suite = SensorSuite(schedule=dvl_loss_schedule(10.0, 20.0),
                        seed=DEVELOPMENT_SEED_ROOT)
    truth = np.array([0.0, 0.0, -17.0])
    during = suite.sample(15.0, truth, np.zeros(3), np.zeros(3), 3.0,
                          WaterState(c=0.2))
    after = suite.sample(35.0, truth, np.zeros(3), np.zeros(3), 3.0,
                         WaterState(c=0.2))
    assert during.dvl_velocity_mps is None and not during.dvl_bottom_lock
    assert after.dvl_velocity_mps is not None and after.dvl_bottom_lock


def test_acoustic_aiding_is_actually_available_on_the_mission():
    """A geometry gate once blocked every acoustic fix, silently deleting the
    non-optical aiding modality from the study."""
    suite = SensorSuite(seed=DEVELOPMENT_SEED_ROOT)
    period = suite.beacon.interrogation_period_s
    # Sampled one interrogation cycle apart, so this tests the geometry gate and
    # not the ping rate. Rate is covered separately by the intermittency test.
    fixes = sum(
        suite.sample(i * period, np.array([x, 0.0, -17.0]), np.zeros(3),
                     np.zeros(3), 3.0, WaterState(c=0.2)).acoustic_range_m is not None
        for i, x in enumerate(np.linspace(-10, 10, 20))
    )
    assert fixes > 10, f"acoustic aiding almost never available ({fixes}/20)"


# ---------------------------------------------------------------------------
# Mission scoring
# ---------------------------------------------------------------------------
def test_cross_track_is_measured_against_truth_not_the_estimate():
    """A vehicle that believes it is on-path while actually off-path must be
    penalised. That failure is invisible to a localization-only evaluation."""
    mission = SurveyMission()
    ev = MissionEvaluator(mission)
    # Mid-leg on the first survey line, so the sample is scored: line-keeping is
    # not measured while the vehicle is turning, and the leg index must name a
    # leg the vehicle is actually flying.
    start, end = mission.waypoints[0], mission.waypoints[1]
    on_path = (start + end) / 2.0
    along = (end - start)[:2]
    along = along / np.linalg.norm(along)
    # Displace perpendicular to the leg, AWAY from the pattern. Moving inward by
    # 5 m would land 1 m from the adjacent leg, which a nearest-segment metric
    # would score as on-path.
    perpendicular = np.array([-along[1], along[0], 0.0])
    off_path = on_path - perpendicular * 5.0
    # The estimate claims we are exactly on the path; the truth is 5 m away.
    ev.record(true_position=off_path, estimated_position=on_path, aided=True,
              waypoints_captured=1)
    outcome = ev.finish(elapsed_s=1.0, path_length_m=1.0, completed=False)
    assert outcome.rms_cross_track_m > 4.0


def test_safety_violations_are_counted_from_truth():
    mission = SurveyMission()
    ev = MissionEvaluator(mission)
    too_low = np.array([0.0, 0.0, mission.seabed_depth_m + 0.1])
    ev.record(too_low, too_low, aided=True, waypoints_captured=0)
    assert ev.safety_violations >= 1


def test_lawnmower_is_a_closed_survey_pattern():
    points = lawnmower()
    assert len(points) == 8
    assert all(p.shape == (3,) for p in points)


# ---------------------------------------------------------------------------
# Campaign-level properties
# ---------------------------------------------------------------------------
def test_run_is_deterministic(model):
    sc = Scenario("det", DEVELOPMENT_SEED_ROOT + 7, WaterProfile.constant(0.6))
    a = run_scenario(sc, build_policies(model, sc.schedule)["fixed"])
    b = run_scenario(sc, build_policies(model, sc.schedule)["fixed"])
    assert a.outcome.rms_cross_track_m == b.outcome.rms_cross_track_m
    assert a.outcome.rms_position_error_m == b.outcome.rms_position_error_m


def test_every_policy_receives_the_same_measurement_realisation(model):
    """Fairness rule R2: the sensor stream is a function of the scenario and seed
    only, never of which method is running."""
    sc = Scenario("parity", DEVELOPMENT_SEED_ROOT + 8, WaterProfile.constant(0.4))
    streams = []
    for _ in range(2):
        suite = SensorSuite(schedule=sc.schedule, seed=sc.seed)
        streams.append([
            suite.sample(i * 0.1, np.array([0.0, 0.0, -17.0]), np.zeros(3),
                         np.zeros(3), 3.0, sc.water.at(i * 0.1)).accel_mps2.copy()
            for i in range(50)
        ])
    for a, b in zip(*streams):
        assert np.array_equal(a, b)


def test_nominal_conditions_leave_every_method_equivalent(model):
    """With nothing to manage, a manager must not help -- and must not hurt.
    This is falsification condition F2, the nominal-regression test.

    F2 is a statement about the *manager*: in benign conditions it must select
    what the fixed policy would have selected, so that any advantage it reports
    elsewhere cannot have come from a permanent configuration preference. The
    comparison is therefore against the fixed policy and the manager's own
    ablations, not against every comparator in the suite.

    Requiring all comparators to agree would be a different and wrong claim.
    Comparators exist precisely because they treat measurements differently, and
    one of them does so visibly here: across nominal seeds, ``covariance_only``
    records lower cross-track error than every other method -- 0.182 against
    0.292, 0.192 against 0.318, 0.244 against 0.353 on three of six seeds, and
    within 0.012 on the rest. The pattern follows the scenario's DVL
    misalignment draw. A large residual mounting rotation produces a steady
    lateral dead-reckoning drift, and correcting it needs the optical fixes that
    a hard innovation gate rejects and that covariance weighting admits at
    reduced weight.

    That is a real result about gating versus weighting under systematic
    velocity error, and it is reported rather than suppressed. It is not a
    failure of F2, because the manager is bit-identical to the fixed policy on
    every one of those seeds.
    """
    sc = Scenario("E1", DEVELOPMENT_SEED_ROOT + 9, WaterProfile.constant(0.2))
    policies = build_policies(model, sc.schedule)
    tracked = ("proposed", "fixed", "oracle", "ablation_a1", "ablation_a2")
    results = {
        name: run_scenario(sc, policies[name]).outcome.rms_cross_track_m
        for name in tracked
        if name in policies
    }
    spread = max(results.values()) - min(results.values())
    assert spread < 0.05, f"manager diverges from the fixed policy: {results}"


# ---------------------------------------------------------------------------
# Gating tests on the decisive scenario.
#
# All three average over several seeds. A gate on a single seed is not a weaker
# test, it is a different and misleading one: on seed 101 alone the proposed
# manager holds position error to 0.11 m against the fixed policy's 0.98 m and
# still loses on path tracking, so a single-seed gate would report a failure that
# five seeds do not support. The same fragility works the other way, which is why
# a single-seed gate cannot be trusted when it passes either.
# ---------------------------------------------------------------------------
GATE_SEEDS = 5


def _decisive_scenarios():
    return [
        Scenario("E8", DEVELOPMENT_SEED_ROOT + 600 + k,
                 WaterProfile.ramp(0.20, 1.80, 15.0, 55.0),
                 coupled_turbidity_dvl_schedule())
        for k in range(GATE_SEEDS)
    ]


def _mean_outcomes(model, names):
    totals = {n: [] for n in names}
    for sc in _decisive_scenarios():
        policies = build_policies(model, sc.schedule)
        for n in names:
            totals[n].append(run_scenario(sc, policies[n]).outcome)
    return totals


def _mean(outcomes, attr):
    return float(np.mean([getattr(o, attr) for o in outcomes]))


@pytest.mark.gating
def test_decisive_case_places_proposed_inside_the_bracket(model):
    """The proposed manager must land between the tuned fixed policy and the
    oracle. Beating the oracle is evidence of a defect, not a result.

    Evaluated on E8, the cell where a capability change is both needed and
    available. E7 removes optical aiding by blackout, so no reconfiguration can
    help there and the bracket collapses -- see PROTOCOL section 5.1.
    """
    out = _mean_outcomes(model, ("fixed", "proposed", "oracle"))
    fixed = _mean(out["fixed"], "rms_cross_track_m")
    proposed = _mean(out["proposed"], "rms_cross_track_m")
    oracle = _mean(out["oracle"], "rms_cross_track_m")
    assert proposed <= fixed, (
        f"proposed no better than the fixed policy ({proposed:.3f} vs {fixed:.3f})"
    )
    assert proposed >= oracle - 0.15, (
        f"proposed beat the oracle ({proposed:.3f} vs {oracle:.3f}) -- investigate"
    )


@pytest.mark.gating
def test_f4_actions_not_inference_produce_the_improvement(model):
    """Falsification condition F4. If covariance-only management matches the full
    manager, the contribution belongs to measurement-weighting scope."""
    out = _mean_outcomes(model, ("proposed", "ablation_a1"))
    proposed = _mean(out["proposed"], "rms_cross_track_m")
    a1 = _mean(out["ablation_a1"], "rms_cross_track_m")
    p_fail = 1.0 - np.mean([o.completed for o in out["proposed"]])
    a1_fail = 1.0 - np.mean([o.completed for o in out["ablation_a1"]])
    separated = (proposed < a1 - 0.05) or (p_fail < a1_fail)
    assert separated, (
        f"A1 matches the full manager: xtrack {proposed:.3f} vs {a1:.3f}, "
        f"failure rate {p_fail:.2f} vs {a1_fail:.2f}"
    )


def test_the_improvement_is_paid_for(model):
    """Both sides of the trade must be visible. A manager that improved outcomes
    at no mission cost would be an artefact."""
    out = _mean_outcomes(model, ("proposed", "fixed"))
    p_fail = 1.0 - np.mean([o.completed for o in out["proposed"]])
    f_fail = 1.0 - np.mean([o.completed for o in out["fixed"]])
    improved = (
        p_fail < f_fail
        or _mean(out["proposed"], "rms_cross_track_m")
        < _mean(out["fixed"], "rms_cross_track_m")
    )
    assert improved, "no improvement to account for"
    # Infrastructure is a currency too, and it was missing from this list.
    #
    # The list above predates the acoustic axis. Once the manager could select
    # between techniques it began buying accuracy with *dependency* -- a surface
    # vessel on station, a surveyed transponder array, a prior bathymetric map
    # -- none of which shows up in altitude, elapsed time or path length. The
    # test then read a real and priced cost as no cost at all and called the
    # improvement an artefact.
    #
    # Measured on the decisive scenarios, the manager consumes roughly four
    # times the infrastructure the fixed policy does (0.201 against 0.050 in the
    # cost model's units). That is the payment, it is exactly the quantity
    # MissionCosts charges for, and the paper argues in Section 5 that it is the
    # reason a capable technique cannot be treated as free.
    infrastructure = (
        _mean(out["proposed"], "acoustic_infrastructure_cost")
        > _mean(out["fixed"], "acoustic_infrastructure_cost")
    )
    paid = (
        infrastructure
        or _mean(out["proposed"], "mean_altitude_m")
        < _mean(out["fixed"], "mean_altitude_m")
        or _mean(out["proposed"], "elapsed_s") > _mean(out["fixed"], "elapsed_s")
        or _mean(out["proposed"], "path_length_m")
        > _mean(out["fixed"], "path_length_m")
    )
    assert paid, "improvement came for free -- suspect an artefact"


# ---------------------------------------------------------------------------
# Metric-validity tests
#
# Every test below exists because the metric it guards was, at one point,
# measuring nothing while passing every other test in this file. They assert
# properties of the *measurement*, not of the method, and a method change must
# never be the reason one of them starts failing.
# ---------------------------------------------------------------------------
def test_cross_track_does_not_reward_drifting_onto_the_next_leg():
    """The failure this paper is about must not score as success.

    A vehicle a full line spacing off course sits exactly on the neighbouring
    survey leg. Scoring cross-track against the nearest segment of the pattern
    -- rather than the segment being flown -- gives it near-zero error while it
    surveys entirely the wrong ground.
    """
    mission = SurveyMission()
    spacing = abs(mission.waypoints[2][1] - mission.waypoints[0][1])
    assert spacing > 1.0, "test assumes distinct legs"

    evaluator = MissionEvaluator(mission)
    on_leg = mission.waypoints[0].copy()
    drifted = on_leg.copy()
    drifted[1] += spacing  # exactly onto the adjacent leg

    # Leg index 1: commanded from waypoint 0 toward waypoint 1.
    error = evaluator._cross_track_error(drifted, 1)
    assert error > 0.9 * spacing, (
        f"a full-spacing drift scored {error:.3f} m of cross-track error; "
        "the metric is measuring distance to the wrong path"
    )


def test_completion_is_judged_on_truth_not_on_what_guidance_believed():
    """Scoring completion from the guidance index is a tautology.

    Guidance advances when the *estimate* reaches a waypoint, and guidance is
    what drives the estimate there. Completion scored that way is 100% for every
    method by construction, and the failed-mission rate carries no information.
    """
    mission = SurveyMission()
    evaluator = MissionEvaluator(mission)
    # Fly a course far from every waypoint, while claiming the vehicle believed
    # it had captured all of them.
    for _ in range(50):
        evaluator.record(
            true_position=np.array([100.0, 100.0, -17.0]),
            estimated_position=np.array([100.0, 100.0, -17.0]),
            aided=True,
            waypoints_captured=len(mission.waypoints),
        )
    outcome = evaluator.finish(elapsed_s=10.0, path_length_m=10.0, completed=True)
    assert not outcome.completed, (
        "a mission that never went near a waypoint was scored as completed"
    )
    assert outcome.coverage_fraction == 0.0


def test_survey_tolerance_cannot_alias_adjacent_legs():
    """Coverage credit must not transfer between neighbouring legs."""
    mission = SurveyMission()
    spacing = abs(mission.waypoints[2][1] - mission.waypoints[0][1])
    assert mission.survey_tolerance_m < spacing / 2.0, (
        "survey tolerance exceeds half the line spacing: a vehicle on one leg "
        "would be credited with covering its neighbour"
    )


def test_oracle_recovery_is_not_computed_on_pooled_means():
    """Regression test for the 2.12 artefact.

    ``oracle_recovery`` is a ratio of differences. Pooling it across scenarios
    where the oracle is sometimes worse than the fixed policy produces a negative
    denominator in one term and flips the sign of the whole statistic. Here two
    scenarios are each individually well-bracketed; pooled, they are not.
    """
    from uuv_mode_aware_navigation.analysis import oracle_recovery_report

    def run(scenario, policy, seed, value):
        return {
            "scenario": scenario, "policy": policy, "seed": seed,
            "rms_cross_track_m": value,
        }

    rows = []
    for seed in range(4):
        # A: oracle helps a lot; proposed recovers half.
        rows += [run("A", "fixed", seed, 2.0),
                 run("A", "proposed", seed, 1.5),
                 run("A", "oracle", seed, 1.0)]
        # B: the oracle is WORSE than fixed -- a degenerate bracket.
        rows += [run("B", "fixed", seed, 1.0),
                 run("B", "proposed", seed, 0.9),
                 run("B", "oracle", seed, 1.6)]

    report = oracle_recovery_report(rows)
    assert report["degenerate"] == ["B"], "degenerate bracket was not detected"
    assert report["per_scenario"] == pytest.approx({"A": 0.5})
    assert report["mean"] == pytest.approx(0.5)

    # The pooled computation the first implementation used:
    pooled = ((2.0 + 1.0) / 2 - (1.5 + 0.9) / 2) / ((2.0 + 1.0) / 2 - (1.0 + 1.6) / 2)
    assert pooled > 1.0, "test no longer reproduces the artefact it guards against"
    assert report["mean"] < 1.0


def test_aggregate_weights_scenario_families_equally():
    """PROTOCOL section 6.1: no family may dominate.

    Pooling runs lets the single-fault families -- in which nothing needs
    managing -- outvote the compound family and dilute a real effect.
    """
    from uuv_mode_aware_navigation.analysis import aggregate_outcome

    rows = []
    # Three flat runs of one family, one run of a family where P is much better.
    for seed in range(3):
        for policy in ("proposed", "fixed"):
            rows.append({
                "scenario": "flat", "policy": policy, "seed": seed,
                "completed": True, "rms_cross_track_m": 1.0,
                "safety_violations": 0, "elapsed_s": 100.0,
            })
    rows.append({
        "scenario": "hard", "policy": "proposed", "seed": 0,
        "completed": True, "rms_cross_track_m": 1.0,
        "safety_violations": 0, "elapsed_s": 100.0,
    })
    rows.append({
        "scenario": "hard", "policy": "fixed", "seed": 0,
        "completed": False, "rms_cross_track_m": 5.0,
        "safety_violations": 4, "elapsed_s": 100.0,
    })

    j = aggregate_outcome(rows)
    # Equal family weighting: the hard family is half the score, so the gap must
    # be large. Pooled over runs it would be a quarter and much smaller.
    assert j["proposed"] < j["fixed"]
    assert j["fixed"] - j["proposed"] > 0.5, (
        f"the compound family was diluted: J={j}"
    )


@pytest.mark.gating
def test_the_failure_matrix_actually_spans_the_comparator_range(model):
    """A scenario discriminates only if the floor fails it and the tuned fixed
    policy survives nominal conditions.

    This criterion is stated in terms of C4 and C1 alone -- never in terms of
    which method wins -- so it cannot be tuned toward a favourable result.
    """
    nominal = Scenario("E1", DEVELOPMENT_SEED_ROOT + 401,
                       WaterProfile.constant(0.2))
    compound = Scenario("E7", DEVELOPMENT_SEED_ROOT + 401,
                        WaterProfile.ramp(0.20, 1.60, 20.0, 90.0),
                        compound_schedule())

    fixed_nominal = run_scenario(
        nominal, build_policies(model, nominal.schedule)["fixed"]
    ).outcome
    assert fixed_nominal.completed, (
        "the tuned fixed policy cannot fly the mission even in clear water "
        "with no faults; the mission is too hard to attribute anything"
    )

    policies = build_policies(model, compound.schedule)
    dr = run_scenario(compound, policies["dead_reckoning"]).outcome
    assert dr.coverage_fraction < 1.0, (
        "dead reckoning completes the compound scenario; nothing in the failure "
        "matrix is demanding enough for absolute aiding to matter"
    )


def test_acoustic_aiding_is_intermittent_not_continuous():
    """S5 is declared intermittent. It must actually be intermittent.

    A single-beacon range is a two-way travel-time measurement. Returning one on
    every simulation tick makes range-only aiding a 10 Hz positioning system that
    bounds error by itself in every scenario -- which silently removes the
    optical channel from the study and would have made "optical management does
    not matter" the study's conclusion, as an artefact.
    """
    suite = SensorSuite(schedule=FaultSchedule(), seed=DEVELOPMENT_SEED_ROOT)
    fixes = 0
    ticks = 600
    dt = 0.1
    for i in range(ticks):
        reading = suite.sample(
            i * dt, np.array([0.0, 0.0, -17.0]), np.zeros(3), np.zeros(3),
            3.0, WaterState(c=0.2),
        )
        if reading.acoustic_range_m is not None:
            fixes += 1
    rate_hz = fixes / (ticks * dt)
    assert 0.0 < rate_hz <= 1.0, (
        f"acoustic aiding arrives at {rate_hz:.2f} Hz; a single-beacon range "
        "cannot be interrogated faster than the acoustic round trip"
    )
    assert fixes > 0, "acoustic aiding never arrives at all"


def test_optical_aiding_matters_somewhere_in_the_failure_matrix(model):
    """The study is about optical and multi-modal aiding. If no condition exists
    in which removing optical aiding changes the outcome, there is nothing to
    manage and no result to report -- whichever method wins."""
    sc = Scenario("E8", DEVELOPMENT_SEED_ROOT + 501,
                  WaterProfile.ramp(0.20, 1.80, 15.0, 55.0),
                  coupled_turbidity_dvl_schedule())
    policies = build_policies(model, sc.schedule)
    with_optical = run_scenario(sc, policies["fixed"]).outcome
    without = run_scenario(sc, policies["dead_reckoning"]).outcome
    assert without.rms_cross_track_m > with_optical.rms_cross_track_m, (
        "removing absolute aiding entirely changes nothing; the scenario cannot "
        "attribute anything to sensing"
    )


# ---------------------------------------------------------------------------
# The static Pareto frontier
# ---------------------------------------------------------------------------
def test_pareto_frontier_excludes_dominated_points():
    from uuv_mode_aware_navigation.analysis import pareto_frontier
    pts = {
        "good_cheap": (1.0, 2.0),     # best on both -- must survive
        "worse_same": (2.0, 2.0),     # dominated by good_cheap
        "same_worse": (1.0, 1.0),     # dominated by good_cheap
        "costly_fast": (3.0, 5.0),    # not comparable -- must survive
    }
    front = set(pareto_frontier(pts))
    assert front == {"good_cheap", "costly_fast"}, front


def test_frontier_claim_is_falsifiable_not_definitional():
    """`beats_every_static_on_J` must be capable of being False.

    A comparison that no comparator can fail is not a comparison. Here a policy
    that is merely mediocre must come out False, so the claim carries content.
    """
    from uuv_mode_aware_navigation.analysis import frontier_report

    def run(policy, j_driver, alt, completed=True):
        return {
            "policy": policy, "scenario": "S", "seed": 0, "completed": completed,
            "rms_cross_track_m": j_driver, "safety_violations": 0,
            "elapsed_s": 100.0, "mean_altitude_m": alt, "path_length_m": 100.0,
        }

    sweep = [run("static_a", 1.0, 3.0), run("static_b", 2.0, 1.0)]
    policies = [run("strong", 0.5, 2.0), run("mediocre", 1.5, 2.0)]
    report = frontier_report(sweep, policies)
    assert report["policies"]["strong"]["beats_every_static_on_J"]
    assert not report["policies"]["mediocre"]["beats_every_static_on_J"]


def test_productivity_rewards_covering_ground_not_merely_moving():
    """Productivity must fall when the vehicle flies lower or slower.

    Both are real survey costs, and a measure blind to either would let a method
    buy navigation quality with mission value invisibly -- which is exactly how
    the single-baseline comparison went wrong.
    """
    from uuv_mode_aware_navigation.analysis import survey_productivity

    def run(policy, alt, elapsed):
        return {
            "policy": policy, "mean_altitude_m": alt,
            "path_length_m": 100.0, "elapsed_s": elapsed,
        }

    p = survey_productivity([
        run("nominal", 3.0, 200.0),
        run("low", 1.0, 200.0),
        run("slow", 3.0, 400.0),
    ])
    assert p["low"] < p["nominal"], "flying lower must cost productivity"
    assert p["slow"] < p["nominal"], "flying slower must cost productivity"
