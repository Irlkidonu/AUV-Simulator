#!/usr/bin/env python3
"""Study 3 post-freeze exploratory evaluation, Part E1.

Executes exactly the design in ``STUDY3_FINAL_DEVELOPMENT_V6_DESIGN.json``: sixty
generated environment realizations, each shared by the three frozen policies,
at exploratory root 33,000,000.

This is **exploratory**. It is not held-out, not confirmatory, and cannot revise
the held-out result or the freeze decision. It applies no threshold and assigns
no verdict; analysis is separate.

Guards, in the order applied:

  1. every file allowlisted in the freeze manifest must still match its hash;
  2. the interpreter must be the pinned one and OpenCV must provide AKAZE;
  3. the root must be exactly 33,000,000 -- the held-out and development bands
     are refused explicitly.

Composes frozen primitives only. Nothing frozen is written to.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[1]
sys.path.insert(0, str(REPOSITORY / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FixedConfiguration, PolicyKind, deployment_informed_environment_configuration,
    generate_environment, load_environment_config, run_one)

EXPLORATORY_ROOT = 35_000_000
HELD_OUT_ROOT = 32_000_000
STAGE = "final_development_v6"
OUT = HERE / "redesign_results"
DESIGN = HERE / "STUDY3_FINAL_DEVELOPMENT_V6_DESIGN.json"
MANIFEST = HERE / "STUDY3_FREEZE_MANIFEST_V1.json"
ENVIRONMENT_CONFIG = HERE / "examples/moderate_severe_variable_environment.json"
COMPLETION_MARKER = HERE / "final_development_v6_result.json"

#: Inherited unchanged from run_final_validation_v5.py Part B.
HORIZON_S = 180.0
DT_S = 2.0
IMAGE_PERIOD_S = 4.0
REDESIGN_VERSION = 3

PINNED_INTERPRETER = "/usr/bin/python3"
PINNED_OPENCV = "4.6.0"

#: The locked universal baseline, as used by V5. Not derived from any result.
LOCKED = dict(optical_channel="lidar", altitude_m=5.0, speed_mps=0.5,
              acoustic_technique="usbl", fusion_mode="weight")


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=True).encode()).hexdigest()


def load_design():
    design = json.loads(DESIGN.read_text())
    if design["root"] != EXPLORATORY_ROOT:
        raise SystemExit("design names a different exploratory root")
    if design["not_held_out"] is not True:
        raise SystemExit("design does not declare itself non-held-out")
    return design


def frozen_files_unchanged():
    """Every allowlisted file must still match the freeze manifest."""
    manifest = json.loads(MANIFEST.read_text())
    drifted = []
    for files in manifest["files"].values():
        for relative, recorded in files.items():
            path = REPOSITORY / relative
            if not path.exists():
                drifted.append(f"{relative} (missing)")
            elif hashlib.sha256(path.read_bytes()).hexdigest() != recorded:
                drifted.append(relative)
    return drifted


def check_guards():
    failures = []
    drifted = frozen_files_unchanged()
    if drifted:
        failures.append(f"frozen files changed since the freeze: {drifted}")
    if sys.executable != PINNED_INTERPRETER:
        failures.append(f"interpreter {sys.executable} is not the pinned {PINNED_INTERPRETER}")
    import cv2  # noqa: PLC0415
    if not hasattr(cv2, "AKAZE_create"):
        failures.append(f"OpenCV {cv2.__version__} lacks AKAZE_create")
    if cv2.__version__ != PINNED_OPENCV:
        failures.append(f"OpenCV {cv2.__version__} is not the pinned {PINNED_OPENCV}")
    return failures


def build_plan():
    """Enumerate the design. Pure; touches no simulation state."""
    design = load_design()
    config = load_environment_config(ENVIRONMENT_CONFIG)
    if config.name != design["environment_config"]["name"]:
        raise SystemExit("environment config name does not match the design")
    seeds = [EXPLORATORY_ROOT + 1_000 + n for n in range(design["environment_seeds"])]
    plan, deployments = [], {}
    for seed in seeds:
        realization = generate_environment(config, seed, HORIZON_S, DT_S)
        deployment = asdict(deployment_informed_environment_configuration(
            FixedConfiguration(**LOCKED), realization))
        deployments[str(seed)] = deployment
        for policy, configuration_id, configuration in (
                ("deployment_fixed", "deployment_informed_fixed_155", deployment),
                ("reactive", "reactive_shared_fixed_155", dict(LOCKED)),
                ("predictive", "predictive_shared_fixed_155", dict(LOCKED))):
            plan.append({"stage": STAGE, "root": EXPLORATORY_ROOT, "part": "generated",
                         "family": config.name, "index": 0, "policy": policy,
                         "configuration_id": configuration_id,
                         "configuration": configuration, "environment_seed": seed})
    return design, plan, deployments, seeds


def execute(task):
    """Run one member and persist an immutable packet. Resumable, never rewritten."""
    root = task["root"]
    if root == HELD_OUT_ROOT:
        raise RuntimeError("the held-out root is forbidden to the exploratory runner")
    if root != EXPLORATORY_ROOT:
        raise RuntimeError("exploratory runner refuses a non-exploratory root")
    identity = {k: task[k] for k in ("stage", "root", "part", "family", "index",
                                     "policy", "configuration_id", "configuration",
                                     "environment_seed")}
    path = OUT / STAGE / (digest(identity)[:24] + ".json")
    if path.exists():
        stored = json.loads(path.read_text())
        supplied = stored.pop("packet_sha256", None)
        if stored["identity"] != identity or digest(stored) != supplied:
            raise RuntimeError(f"bad resume packet {path}")
        return stored["result"]
    realization = generate_environment(
        load_environment_config(ENVIRONMENT_CONFIG), task["environment_seed"],
        HORIZON_S, DT_S)
    run_result, trace = run_one(
        root, task["family"], task["index"], PolicyKind(task["policy"]),
        FixedConfiguration(**task["configuration"]),
        horizon_s=HORIZON_S, dt_s=DT_S, image_period_s=IMAGE_PERIOD_S,
        keep_trace=True, redesign_version=REDESIGN_VERSION,
        environment_realization=realization)
    result = asdict(run_result)
    result["causal_trace"] = trace
    result["environment_digest"] = realization.digest
    result["configuration_id"] = task["configuration_id"]
    result["part"] = "generated"
    packet = {"schema": "study3_final_development_v6_packet_v1",
              "identity": identity, "result": result}
    packet["packet_sha256"] = digest(packet)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(packet, sort_keys=True, indent=2, allow_nan=True) + "\n")
    os.replace(temporary, path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", action="store_true",
                        help="enumerate the plan and exit; executes nothing")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--workers", type=int, default=4)
    arguments = parser.parse_args()

    design, plan, deployments, seeds = build_plan()
    summary = {"schema": "study3_final_development_v6_summary_v1",
               "classification": "DEVELOPMENT",
               "not_held_out": True, "not_confirmatory": True,
               "stage": STAGE, "root": EXPLORATORY_ROOT,
               "held_out_accessed": False,
               "policies": design["policies"],
               "environment_seeds": len(seeds),
               "seed_first": seeds[0], "seed_last": seeds[-1],
               "planned_total": len(plan),
               "design_total": design["executions"]["total"],
               "environment_config": ENVIRONMENT_CONFIG.name,
               "locked_universal_baseline": dict(LOCKED),
               "execution_parameters": {"horizon_s": HORIZON_S, "dt_s": DT_S,
                                        "image_period_s": IMAGE_PERIOD_S,
                                        "redesign_version": REDESIGN_VERSION},
               "plan_digest": digest([{k: t[k] for k in
                                       ("environment_seed", "policy", "configuration_id")}
                                      for t in plan])}

    if arguments.plan or not arguments.execute:
        summary["executed"] = False
        summary["frozen_file_drift"] = frozen_files_unchanged()
        print(json.dumps(summary, sort_keys=True, indent=2))
        matches = summary["planned_total"] == summary["design_total"]
        print(f"\nplan matches the predeclared design: {matches}")
        return 0 if matches else 1

    failures = check_guards()
    if failures:
        print("exploratory execution REFUSED")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    started = time.perf_counter()
    if arguments.workers <= 1:
        rows = [execute(task) for task in plan]
    else:
        with ProcessPoolExecutor(max_workers=arguments.workers) as pool:
            rows = list(pool.map(execute, plan))
    summary["executed"] = True
    summary["elapsed_s"] = round(time.perf_counter() - started, 3)
    summary["executions"] = len(rows)
    summary["deployment_informed_configurations"] = deployments
    summary["frozen_file_drift_after"] = frozen_files_unchanged()
    summary["result_digest"] = digest(sorted(
        (r["policy"], r["environment_digest"], r["trace_digest"]) for r in rows))
    COMPLETION_MARKER.write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items()
                      if k != "deployment_informed_configurations"},
                     sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
