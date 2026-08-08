#!/usr/bin/env python3
"""Run a Paper 2 development campaign and report paired statistics.

Development use only. Held-out execution is gated on a freeze record and is not
reachable from this script.

Usage::

    PYTHONPATH=. python3 scripts/run_campaign.py --seeds 20 --out results/dev.csv
"""

from __future__ import annotations

import argparse
import time
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uuv_mode_aware_navigation.analysis import (  # noqa: E402
    aggregate_outcome,
    oracle_recovery,
    oracle_recovery_report,
    paired_difference,
    summarise,
    write_csv,
)
from uuv_mode_aware_navigation.availability import (  # noqa: E402
    AvailabilityModel,
    AvailabilitySample,
)
from freeze import mark_held_out_spent, require_freeze  # noqa: E402
from uuv_mode_aware_navigation.campaign import (  # noqa: E402
    DEVELOPMENT_SEED_ROOT,
    HELDOUT_SEED_ROOT,
    HELDOUT_SEED_ROOTS,
    BASELINE_TERRAIN_GRADIENT,
    FEATURELESS_TERRAIN_GRADIENT,
    TerrainProfile,
    CurrentProfile,
    NoiseProfile,
    Scenario,
    WaterProfile,
    run_scenario,
    static_sweep,
)
from uuv_mode_aware_navigation.comparators import (  # noqa: E402
    FixedPolicy,
    build_policies,
)
from uuv_mode_aware_navigation.imaging import (  # noqa: E402
    OpticalFeedback,
    analyse_image,
    render_patch,
    seabed_texture,
)
from uuv_mode_aware_navigation.manager import DEFAULT_CANDIDATES  # noqa: E402
from uuv_mode_aware_navigation.optics import (  # noqa: E402
    CAMERA_OFFAXIS,
    CONFIGURATIONS,
    WaterState,
    channel_response,
)
from uuv_mode_aware_navigation.sensors import (  # noqa: E402
    FaultSchedule,
    acoustic_duty_cycle_schedule,
    compound_schedule,
    coupled_turbidity_dvl_schedule,
    dvl_loss_schedule,
    optical_loss_schedule,
    short_dvl_loss_schedule,
    surface_asset_loss_schedule,
    total_dvl_loss_schedule,
    unprepared_area_schedule,
)

ALTITUDES = (1.0, 1.5, 2.0, 2.5, 3.0, 4.0)
TURBIDITIES = (0.15, 0.35, 0.6, 0.9, 1.2, 1.6, 2.0)


#: Rates of change of beam attenuation used to generate trend-bearing training
#: samples, in units of c per identification window. Zero is included so the
#: steady case is represented; the non-zero values bracket the ramps the
#: scenarios impose (0.20 to 1.80 over 40 s is about 0.4 per 10 s window).
TREND_RATES = (-0.4, -0.15, 0.0, 0.15, 0.4)


def calibrate(seed: int) -> AvailabilityModel:
    """Fit the availability model on development data only.

    Training samples carry a *quality trend* as well as an instantaneous
    quality. Without them the trend coefficient would be fitted entirely on
    zeros and the feature would be inert -- present in the design matrix,
    weighted arbitrarily, and contributing nothing. Several axes of this study
    have failed exactly that way, so the training distribution has to contain
    the variation the feature is meant to explain.

    The trend is generated the way it arises: the water is evolving, so the
    quality observed one window ago differs from the quality observed now, and
    the label is whether the candidate configuration is available *after* the
    horizon rather than at the instant of observation. That is what makes the
    coefficient mean something -- it is fitted against the future, which is the
    question the manager is actually asking.
    """
    rng = np.random.default_rng(seed)
    samples = []
    for c in TURBIDITIES:
        for rate in TREND_RATES:
            # Conditions one window ago, now, and one decision horizon ahead.
            past = WaterState(c=max(c - rate, 0.02))
            water = WaterState(c=c)
            future = WaterState(c=max(c + rate, 0.02))
            for observed_alt in ALTITUDES:
                observed = channel_response(
                    water, observed_alt, CAMERA_OFFAXIS, rng=rng
                )
                previous = channel_response(
                    past, observed_alt, CAMERA_OFFAXIS, rng=rng
                )
                trend = observed.quality - previous.quality
                for candidate_alt in ALTITUDES:
                    for cfg in CONFIGURATIONS:
                        samples.append(
                            AvailabilitySample(
                                observed.quality, observed_alt, candidate_alt,
                                cfg.name,
                                channel_response(
                                    future, candidate_alt, cfg, rng=rng
                                ).available,
                                quality_trend=trend,
                            )
                        )
    return AvailabilityModel().fit(samples)


def calibrate_optical_feedback(seed: int) -> OpticalFeedback:
    """Fit the image-only quality estimator on development data.

    Uses a different seed root from the availability model so the two are not
    fitted on a single realisation. Textures and altitudes here are the fitting
    set; agreement is reported in the tests against conditions used for neither
    fitting nor feature selection.
    """
    rng = np.random.default_rng(seed)
    textures = [seabed_texture(seed=seed + 101 + i) for i in range(4)]
    features, quality = [], []
    for c in np.linspace(0.15, 2.2, 15):
        for altitude in ALTITUDES:
            for config in CONFIGURATIONS:
                for texture in textures:
                    water = WaterState(c=float(c))
                    frame = render_patch(water, altitude, config, texture, rng)
                    features.append(analyse_image(frame))
                    quality.append(
                        channel_response(water, altitude, config).quality
                    )
    return OpticalFeedback().fit(features, quality)


#: Weak residual flow used by every cell that is not about currents, so that the
#: current axis changes only the cells that declare it.
BASELINE_CURRENT = CurrentProfile.constant((0.02, -0.01, 0.0))


def scenario_family():
    """The failure matrix (PROTOCOL.md section 5).

    Each entry is (name, water profile, fault schedule, current profile).

    Cells E9--E12 exercise the current axis. They exist because the title claims
    adaptation to ocean currents, and a matrix whose every cell carried the same
    0.02 m/s residual flow could not support that claim whatever the outcome: a
    current that small next to a 0.25--0.50 m/s survey speed is a rounding error
    on the track. E10 and E11 raise it to a fraction of vehicle speed, E12 turns
    it so that estimating it is not a one-off calibration, and E9 removes the
    ability to observe it at all while it is strong.
    """
    return [
        ("E1_nominal", WaterProfile.constant(0.20), FaultSchedule()),
        ("E2_dvl_short", WaterProfile.constant(0.20), short_dvl_loss_schedule()),
        ("E3_dvl_long", WaterProfile.constant(0.20), dvl_loss_schedule()),
        ("E4_optical_graded", WaterProfile.ramp(0.20, 1.60, 20.0, 90.0), FaultSchedule()),
        ("E5_optical_loss", WaterProfile.constant(0.20), optical_loss_schedule()),
        ("E6_acoustic_intermittent", WaterProfile.constant(0.20),
         acoustic_duty_cycle_schedule()),
        ("E7_compound", WaterProfile.ramp(0.20, 1.60, 20.0, 90.0), compound_schedule()),
        # E8 pairs a turbidity high enough that the nominal camera configuration
        # cannot produce a fix at survey altitude with a loss of velocity aiding,
        # so an absolute fix is both needed and obtainable -- but only from a
        # different configuration. See coupled_turbidity_dvl_schedule.
        ("E8_turbid_dvl_loss", WaterProfile.ramp(0.20, 1.80, 15.0, 55.0),
         coupled_turbidity_dvl_schedule()),
        # --- current axis -------------------------------------------------
        # A strong flow the vehicle can no longer observe. Both DVL modes fail
        # together, so the current estimate is frozen at its last value while
        # its uncertainty grows and the compensation silently goes stale. This
        # is the cell where knowing *how well* the flow is known matters.
        ("E9_current_unobservable", WaterProfile.constant(0.20),
         total_dvl_loss_schedule(), CurrentProfile.constant((0.18, -0.10, 0.0))),
        # A steady moderate flow with everything working: the easy case, and the
        # control that shows compensation works before any fault is added.
        ("E10_current_steady", WaterProfile.constant(0.20), FaultSchedule(),
         CurrentProfile.constant((0.12, -0.06, 0.0))),
        # A flow that strengthens mid-survey to a quarter of the vehicle's
        # maximum water speed, while turbidity also rises.
        ("E11_current_building", WaterProfile.ramp(0.20, 1.40, 20.0, 90.0),
         FaultSchedule(),
         CurrentProfile.ramp((0.05, 0.02, 0.0), (0.22, -0.14, 0.0), 25.0, 100.0)),
        # A veering flow. A constant current is a bias and anything that
        # estimates a bias removes it; a rotating one keeps the estimate
        # perpetually behind the truth, which is what tests tracking rather than
        # convergence.
        ("E12_current_rotating", WaterProfile.constant(0.20),
         short_dvl_loss_schedule(),
         CurrentProfile.rotating(speed_mps=0.15, period_s=240.0)),
        # --- acoustic-noise axis ------------------------------------------
        # A vessel passes overhead mid-survey and the water becomes acoustically
        # loud. Multipath outliers rise from roughly 1% to 25% of interrogations,
        # so the acoustic channel does not fail outright -- it starts lying
        # occasionally, by 15-20 m, in one direction. That is a different problem
        # from an outage and needs a different response.
        ("E13_acoustic_noise", WaterProfile.constant(0.20), FaultSchedule(),
         BASELINE_CURRENT, NoiseProfile.ramp(40.0, 70.0, 30.0, 90.0)),
        # Loud throughout, with velocity aiding also lost: the vehicle needs
        # absolute fixes and the ones it can get are contaminated.
        ("E14_noisy_dvl_loss", WaterProfile.constant(0.20),
         dvl_loss_schedule(), BASELINE_CURRENT, NoiseProfile.constant(68.0)),
        # Turbid and loud together: optical degraded, acoustic contaminated.
        # Neither modality is clean and the vehicle must weigh two bad options.
        ("E15_turbid_and_noisy", WaterProfile.ramp(0.20, 1.60, 20.0, 90.0),
         FaultSchedule(), BASELINE_CURRENT, NoiseProfile.constant(65.0)),
        # E16 and E17 exist because terrain-relative navigation was added to the
        # action space, and a technique with no failure mode is not a decision.
        #
        # E16 -- a sediment plain under clear water. Terrain matching returns
        # nothing at all: with no relief there is no correlation peak, and every
        # position on a flat seabed predicts the same depth. Optical aiding is
        # fine. A vehicle fixed on terrain matching fails here and one that can
        # switch does not, which is what stops the sweep selecting a terrain
        # configuration and keeping it.
        ("E16_featureless_plain", WaterProfile.constant(0.20), FaultSchedule(),
         BASELINE_CURRENT, NoiseProfile.constant(40.0),
         TerrainProfile.constant(FEATURELESS_TERRAIN_GRADIENT)),
        # E17 -- the mirror of E8, and the case terrain matching was added for.
        # Turbidity past the optical limit and both DVL modes gone, over a
        # seabed with relief. Optical cannot recover at any altitude and there
        # is no velocity reference, but the echo sounder is still working and
        # the terrain is still there. A capability change is needed and is
        # available, from a modality the first version of this study did not
        # model. If the method cannot win here it cannot win anywhere.
        ("E17_terrain_recoverable", WaterProfile.ramp(0.20, 1.80, 15.0, 45.0),
         coupled_turbidity_dvl_schedule(), BASELINE_CURRENT,
         NoiseProfile.constant(40.0),
         TerrainProfile.constant(0.22)),
        # E18 -- the support vessel leaves station while the water turns turbid.
        # USBL is the best technique in the action space and it depends on
        # something that is not part of the vehicle at all. Geometry cannot
        # predict its loss, so a manager that reasons only from geometry
        # re-selects a technique nothing is answering. Terrain matching is the
        # replacement, and the seabed here has relief for it to work with.
        ("E18_vessel_departs", WaterProfile.ramp(0.20, 1.70, 20.0, 60.0),
         surface_asset_loss_schedule(), BASELINE_CURRENT,
         NoiseProfile.constant(40.0),
         TerrainProfile.constant(0.18)),
        # E19 -- no acoustic infrastructure at all, because none was deployed.
        # This is the autonomy case the paper argues from: every acoustic
        # technique needs something a third party put in the water, and a
        # vehicle working an unprepared area has none of them. With velocity
        # aiding lost and the water turning turbid, the seabed is the only
        # absolute reference left.
        # No prior survey either: an area nobody has mapped well enough to lay
        # transponders in is an area nobody has mapped well enough to navigate
        # by terrain. The relief is there; the chart is not.
        ("E19_unprepared_area", WaterProfile.ramp(0.20, 1.75, 20.0, 60.0),
         unprepared_area_schedule(), BASELINE_CURRENT,
         NoiseProfile.constant(40.0),
         TerrainProfile.constant(0.20), False),
    ]


#: The primary metric the hindsight oracle is optimised against. Declared here,
#: once, so the oracle cannot be quietly re-optimised per figure.
ORACLE_OBJECTIVE = "rms_cross_track_m"


def _load_rows(path: Path) -> list[dict]:
    """Read a campaign CSV back with the types the analysis functions expect."""
    import csv
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key, value in list(row.items()):
            if key in ("scenario", "policy"):
                continue
            if value in ("", "None"):
                row[key] = float("nan")
            elif value in ("True", "False"):
                row[key] = value == "True"
            else:
                try:
                    row[key] = float(value)
                except ValueError:
                    pass
    return rows


def _install_hindsight_oracle(rows: list[dict], sweep: list[dict]) -> list[dict]:
    """Replace the clairvoyant oracle rows with a genuine per-seed ceiling.

    For each (scenario, seed) the oracle's outcome is the better of:

    * the best static configuration for that exact run, chosen with hindsight;
    * the clairvoyant dynamic manager, which sees the true water profile and
      fault schedule over its projection horizon.

    Taking the better of the two is what makes ``C5`` an upper bound rather than
    just another policy holding privileged information. The previous
    construction -- a hand-written heuristic given the fault schedule -- was
    beaten by the proposed manager in the compound scenario, which did not
    indicate that the method was excellent. It indicated that the comparator was
    not a bound.
    """
    best_static: dict[tuple, dict] = {}
    for row in sweep:
        key = (row["scenario"], row["seed"])
        incumbent = best_static.get(key)
        if incumbent is None or row[ORACLE_OBJECTIVE] < incumbent[ORACLE_OBJECTIVE]:
            best_static[key] = row

    out = []
    replaced = 0
    for row in rows:
        if row["policy"] != "oracle":
            out.append(row)
            continue
        key = (row["scenario"], row["seed"])
        candidate = best_static.get(key)
        if candidate is not None and candidate[ORACLE_OBJECTIVE] < row[ORACLE_OBJECTIVE]:
            row = {**candidate, "policy": "oracle"}
            replaced += 1
        out.append(row)
    print(f"  hindsight oracle improved on the clairvoyant manager in "
          f"{replaced} of {len(best_static)} runs")
    return out


# ---------------------------------------------------------------------------
# Parallel execution
#
# Every run is a pure function of (scenario, policy specification): the sensor
# suite is re-seeded from the scenario seed, the estimator starts fresh, and no
# run reads state written by another. Distributing them therefore cannot change
# any result, only the order in which they arrive -- and both helpers sort their
# output before returning so even that is not observable.
#
# Policies are rebuilt inside the worker rather than sent to it. A policy object
# carries fitted models and, for the oracle, the scenario's fault schedule;
# rebuilding from the specification keeps the parent's objects out of the
# pickling path entirely.
# ---------------------------------------------------------------------------
def _sweep_one(task):
    scenario, config_name, feedback = task
    from uuv_mode_aware_navigation.comparators import FixedPolicy

    config = next(c for c in DEFAULT_CANDIDATES if c.name == config_name)
    result = run_scenario(
        scenario, FixedPolicy(config, name=config_name), optical_feedback=feedback
    )
    result.policy = config_name
    return result.to_row()


def _parallel_sweep(scenarios, feedback, jobs):
    from multiprocessing import Pool

    tasks = [
        (scenario, config.name, feedback)
        for scenario in scenarios
        for config in DEFAULT_CANDIDATES
    ]
    # Report progress. Phase 1 previously printed nothing between its opening
    # line and its result, so a run that had done 20% and one that had done 95%
    # looked identical for six hours, and the only way to tell a slow sweep from
    # a hung one was to inspect worker CPU time. ``imap`` streams completions in
    # exchange for giving up ``map``'s batching; rows are sorted below anyway,
    # so ordering is unaffected.
    total = len(tasks)
    rows = []
    start = time.monotonic()
    with Pool(processes=jobs) as pool:
        for i, row in enumerate(pool.imap_unordered(_sweep_one, tasks, chunksize=4), 1):
            rows.append(row)
            if i % 500 == 0 or i == total:
                elapsed = time.monotonic() - start
                rate = i / max(elapsed, 1e-9)
                remaining = (total - i) / rate if rate > 0 else float("nan")
                print(
                    f"  phase 1: {i}/{total} runs "
                    f"({100.0 * i / total:.1f}%), "
                    f"{elapsed / 60.0:.0f} min elapsed, "
                    f"~{remaining / 60.0:.0f} min remaining",
                    flush=True,
                )
    return sorted(rows, key=lambda r: (r["scenario"], r["seed"], r["policy"]))


def _comparators_one(task):
    scenario, model, best_config, feedback = task
    from uuv_mode_aware_navigation.comparators import FixedPolicy

    policies = build_policies(model, scenario.schedule)
    policies["fixed"] = FixedPolicy(best_config)
    out = []
    for policy_name, policy in policies.items():
        result = run_scenario(scenario, policy, optical_feedback=feedback)
        result.policy = policy_name
        out.append(result.to_row())
    return out


def _parallel_comparators(scenarios, model, best_config, feedback, jobs):
    from multiprocessing import Pool

    tasks = [(s, model, best_config, feedback) for s in scenarios]
    with Pool(processes=jobs) as pool:
        batches = pool.map(_comparators_one, tasks, chunksize=1)
    rows = [row for batch in batches for row in batch]
    return sorted(rows, key=lambda r: (r["scenario"], r["seed"], r["policy"]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=10)
    parser.add_argument("--out", type=str, default="results/development.csv")
    parser.add_argument("--root", type=int, default=DEVELOPMENT_SEED_ROOT)
    parser.add_argument("--reuse-sweep", action="store_true",
                        help="reuse an existing static_sweep.csv for the same scenarios")
    parser.add_argument(
        "--jobs", type=int, default=1,
        help="worker processes. Results are identical to a serial run: every "
             "run is seeded from its scenario's seed and shares no state with "
             "any other, so only the order in which they complete changes, and "
             "rows are re-sorted before they are written.",
    )
    parser.add_argument(
        "--analytic-quality", action="store_true",
        help="report the analytic quality index instead of estimating it from a "
             "rendered frame. Development convenience only: the reported campaign "
             "runs with optical feedback, because that is what the title claims.",
    )
    parser.add_argument(
        "--fixed-config", type=str, default=None,
        help="name of the configuration to use as the fixed baseline C1, "
             "skipping the sweep that would otherwise select it. Used for the "
             "held-out comparator run, where C1 is settled on development data "
             "beforehand (PROTOCOL S2.5). The sweep is then run separately to "
             "report what a hindsight selection on held-out data would have "
             "chosen, as a supplementary comparison.",
    )
    parser.add_argument(
        "--sweep-only", action="store_true",
        help="run the configuration sweep and stop. Produces the baseline "
             "identity and the per-seed oracle without re-running the "
             "comparators.",
    )
    parser.add_argument(
        "--held-out", action="store_true",
        help="execute the held-out seed block (PROTOCOL D4). Sets --root to the "
             "held-out root, requires a verified freeze record, and marks that "
             "record spent on success so a second execution is refused.",
    )
    args = parser.parse_args()

    # --- The held-out gate ----------------------------------------------
    # PROTOCOL D4 permits exactly one execution of the held-out block, after
    # the freeze. Both halves of that sentence are enforced here rather than
    # remembered: `require_freeze` refuses if the tree has moved since the
    # record was written or if the block has already been spent.
    #
    # The second branch catches the accident that would be indistinguishable
    # from cheating after the fact -- reaching the held-out seeds through
    # `--root` without going through the gate.
    if args.held_out:
        args.root = HELDOUT_SEED_ROOT
        record = require_freeze()
        print(f"held-out execution permitted against freeze record of "
              f"{record['frozen_at']} ({record['file_count']} files)")
        if args.reuse_sweep:
            raise SystemExit(
                "refusing to reuse a development sweep for the held-out block: "
                "the sweep selects C1, and reusing one computed on development "
                "seeds would carry a development choice into the held-out result."
            )
    elif args.root in HELDOUT_SEED_ROOTS:
        raise SystemExit(
            f"seed root {args.root} is a held-out block. Reaching one without "
            "--held-out would bypass the freeze gate. Every reserved root is "
            "refused here, including blocks already spent: a spent block that "
            "can be re-entered through an ordinary argument is a development "
            "block, and every number ever drawn from it would have to be "
            "reported as one."
        )

    model = calibrate(args.root + 1)
    feedback = None if args.analytic_quality else calibrate_optical_feedback(
        args.root + 2
    )
    if feedback is not None:
        print("optical feedback enabled: quality is estimated from rendered frames")
    scenarios = [
        Scenario(
            name=entry[0],
            seed=args.root + 1000 + k,
            water=entry[1],
            schedule=entry[2],
            current=entry[3] if len(entry) > 3 else BASELINE_CURRENT,
            noise=entry[4] if len(entry) > 4 else NoiseProfile.constant(40.0),
            terrain=entry[5] if len(entry) > 5 else TerrainProfile.constant(
                BASELINE_TERRAIN_GRADIENT
            ),
            prior_map=entry[6] if len(entry) > 6 else True,
        )
        for entry in scenario_family()
        for k in range(args.seeds)
    ]

    # --- Phase 1: the static sweep --------------------------------------
    # Every configuration in the manager's own action space, flown on every
    # scenario. Selects C1 and bounds C5. Runs before anything else so that
    # neither is chosen with knowledge of how the proposed method did.
    # A named baseline skips the sweep entirely. Everything the comparator
    # campaign needs is the configuration itself; the sweep exists to choose it
    # and to build the oracle, and both are deferred to a separate run.
    if args.fixed_config is not None:
        best_config = next(
            (c for c in DEFAULT_CANDIDATES if c.name == args.fixed_config), None
        )
        if best_config is None:
            raise SystemExit(
                f"no configuration named {args.fixed_config!r}. It must match a "
                "candidate name exactly, so that the baseline reported in the "
                "paper is the one that was flown."
            )
        print(f"fixed baseline supplied: {best_config.name}")
        print("sweep skipped; C1 was settled on development data (PROTOCOL S2.5)")
    else:
        best_config = None

    sweep = None
    sweep_path = Path(args.out).with_name("static_sweep.csv")
    if best_config is not None:
        # Nothing below runs: the sweep exists to choose the baseline and to
        # build the per-seed oracle, and the baseline is already chosen. The
        # oracle is therefore the clairvoyant policy rather than the hindsight
        # ceiling, and the caller is told so rather than left to infer it from
        # a number that looks like an oracle and is not.
        print("  no per-seed hindsight oracle in this run; the oracle rows are "
              "the clairvoyant policy only")
    elif args.reuse_sweep and sweep_path.exists():
        # The sweep is a pure function of the scenario list and the candidate
        # set, so reusing it across reruns of phases 2 and 3 is exact, not an
        # approximation. Any change to either invalidates it, which is why this
        # is opt-in rather than automatic.
        print(f"phase 1: reusing static sweep from {sweep_path}")
        sweep = _load_rows(sweep_path)
        if {r["scenario"] for r in sweep} != {s.name for s in scenarios}:
            raise SystemExit(
                "refusing to reuse a sweep computed for different scenarios"
            )
    else:
        print("phase 1: static configuration sweep "
              f"({len(DEFAULT_CANDIDATES)} configurations x {len(scenarios)} scenarios)")
        if args.jobs > 1:
            sweep = _parallel_sweep(scenarios, feedback, args.jobs)
        else:
            sweep = [r.to_row() for r in static_sweep(
                scenarios, optical_feedback=feedback
            )]
        write_csv(sweep, sweep_path)

    if best_config is None:
        sweep_scores = aggregate_outcome(sweep)
        best_config_name = min(sweep_scores, key=lambda k: sweep_scores[k])
        best_config = next(
            c for c in DEFAULT_CANDIDATES if c.name == best_config_name
        )
        print(f"  best fixed configuration (C1): {best_config_name}  "
              f"J={sweep_scores[best_config_name]:.3f}")
        worst = max(sweep_scores, key=lambda k: sweep_scores[k])
        print(f"  worst configuration in the sweep: {worst}  "
              f"J={sweep_scores[worst]:.3f}")
        print(f"  full table written to "
              f"{Path(args.out).with_name('static_sweep.csv')}")
        if args.sweep_only:
            print("\nsweep-only: stopping before the comparator campaign")
            return 0

    # --- Phase 2: the comparator campaign -------------------------------
    print("\nphase 2: comparator campaign")
    if args.jobs > 1:
        rows = _parallel_comparators(
            scenarios, model, best_config, feedback, args.jobs
        )
    else:
        rows = []
        for scenario in scenarios:
            policies = build_policies(model, scenario.schedule)
            policies["fixed"] = FixedPolicy(best_config)
            for policy_name, policy in policies.items():
                result = run_scenario(scenario, policy, optical_feedback=feedback)
                result.policy = policy_name
                rows.append(result.to_row())

    # --- Phase 3: the hindsight oracle ----------------------------------
    # Per scenario AND per seed, the best static configuration chosen with full
    # hindsight, taken together with the clairvoyant dynamic manager. This is a
    # ceiling by construction rather than by hope.
    if sweep is not None:
        rows = _install_hindsight_oracle(rows, sweep)

    path = write_csv(rows, args.out)
    print(f"\nwrote {len(rows)} comparator runs to {path}\n")

    # The block is spent once results exist on disk, not when the run starts: a
    # crash before this point leaves it unspent, because nothing was learned
    # from it. After this point a second execution is refused.
    if args.held_out:
        import hashlib
        digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        mark_held_out_spent(str(path), digest)
        print(f"held-out block marked spent; results sha256 {digest[:16]}...\n")

    means = summarise(rows, "rms_cross_track_m")
    coverage = summarise(rows, "coverage_fraction")
    failed = {
        p: 1.0 - v for p, v in summarise(
            [{**r, "completed_f": 1.0 if r["completed"] else 0.0} for r in rows],
            "completed_f",
        ).items()
    }
    aggregate = aggregate_outcome(rows)
    print(
        f"{'policy':<18}{'failed':>9}{'coverage':>10}"
        f"{'xtrack (m)':>12}{'aggregate J':>14}"
    )
    for policy in sorted(means, key=lambda p: aggregate[p]):
        print(
            f"{policy:<18}{failed[policy]:9.2f}{coverage[policy]:10.3f}"
            f"{means[policy]:12.3f}{aggregate[policy]:14.3f}"
        )

    # Design check, set independently of which method wins: a scenario can only
    # discriminate if the floor fails it and the tuned fixed policy survives the
    # nominal case. Scenarios that fail this test are reported, not dropped.
    print("\ndiscrimination check (does the scenario span the comparator range?):")
    for scenario in sorted({r["scenario"] for r in rows}):
        subset = [r for r in rows if r["scenario"] == scenario]
        f_dr = 1.0 - np.mean(
            [1.0 if r["completed"] else 0.0
             for r in subset if r["policy"] == "dead_reckoning"]
        )
        f_fixed = 1.0 - np.mean(
            [1.0 if r["completed"] else 0.0
             for r in subset if r["policy"] == "fixed"]
        )
        spans = f_dr > f_fixed
        print(
            f"  {scenario:<20} dead-reckoning fails {f_dr:.0%}, "
            f"fixed fails {f_fixed:.0%}  "
            f"{'-> discriminates' if spans else '-> FLAT (report as such)'}"
        )

    report = oracle_recovery_report(rows)
    print("\noracle recovery, per scenario (pooling this ratio is invalid):")
    for scenario, value in sorted(report["per_scenario"].items()):
        flag = "  <-- INVESTIGATE: beat the oracle" if value > 1.0 else ""
        print(f"  {scenario:<20}{value:8.2f}{flag}")
    for scenario in report["degenerate"]:
        print(f"  {scenario:<20}      -- degenerate bracket (no headroom)")
    if "mean" in report:
        print(f"  {'mean over non-degenerate':<20}{report['mean']:8.2f}")

    print("\npaired comparisons (direction annotated per metric):")
    for metric in ("rms_cross_track_m", "coverage_fraction"):
        for reference in ("fixed", "covariance_only", "ablation_a1"):
            try:
                print("  " + paired_difference(
                    rows, metric, "proposed", reference
                ).describe())
            except ValueError:
                pass

    print("\ncost of the improvement:")
    for metric in ("mean_altitude_m", "elapsed_s"):
        m = summarise(rows, metric)
        print(f"  {metric:<18} proposed={m['proposed']:.2f}  fixed={m['fixed']:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
