#!/usr/bin/env python3
"""Study 3 corrected-controller one-shot held-out runner (root 36,000,000).

Executes exactly the design frozen in ``STUDY3_HELDOUT_V2_DESIGN.json``. It
makes no scientific choice: families, policies, seed counts, configurations and
execution parameters are read from frozen records.

``--plan`` enumerates without executing and **without generating any environment
realization at a held-out seed**. Part B deployment-informed configurations are
resolved inside ``execute`` for that reason, so preparation never touches the
reserved band.

Guards, in order:
  1. ``verify_lock_v2.py --require-authorization`` must exit 0;
  2. pinned interpreter and OpenCV with AKAZE;
  3. root exactly 36,000,000, enforced again inside ``execute``;
  4. no completed held-out output already present;
  5. the authorization must not already be marked executed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[1]
sys.path.insert(0, str(REPOSITORY / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FAMILIES, FixedConfiguration, PolicyKind, deployed_acoustic_services,
    deployment_informed_environment_configuration,
    deployment_informed_fixed_configuration, generate_environment,
    load_environment_config, run_one)

HELD_OUT_ROOT = 36_000_000
STAGE = "heldout_v2"
OUT = HERE / "redesign_results"
DESIGN = HERE / "STUDY3_HELDOUT_V2_DESIGN.json"
AUTHORIZATION = HERE / "STUDY3_HELDOUT_V2_AUTHORIZATION.json"
MANIFEST = HERE / "STUDY3_FREEZE_MANIFEST_V2.json"
FIXED_LOCK = OUT / "infrastructure_fixed_baseline_lock.json"
ENVIRONMENT_CONFIG = HERE / "examples/moderate_severe_variable_environment.json"
COMPLETION_MARKER = HERE / "heldout_v2_result.json"
VERIFY_LOCK = HERE / "verify_lock_v2.py"

#: Inherited unchanged from the frozen development runners. Not chosen here.
HORIZON_S = 180.0
DT_S = 2.0
IMAGE_PERIOD_S = 4.0
REDESIGN_VERSION = 3

PINNED_INTERPRETER = "/usr/bin/python3"
PINNED_OPENCV = "4.6.0"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=True).encode()).hexdigest()


def load_design():
    design = json.loads(DESIGN.read_text())
    if design["held_out_root"] != HELD_OUT_ROOT:
        raise SystemExit("frozen design names a different held-out root")
    if design["maximum_invocations"] != 1:
        raise SystemExit("frozen design is not one-shot")
    return design


def build_plan():
    """Enumerate the frozen design. Generates no held-out environment."""
    design = load_design()
    locked = json.loads(FIXED_LOCK.read_text())["configuration"]
    policies = design["policies"]
    plan = []

    scripted = design["part_a_scripted"]
    for families, seeds, group in (
            (scripted["primary_families"], scripted["seeds_per_primary_family_policy"], "primary"),
            (scripted["control_families"], scripted["seeds_per_control_family_policy"], "control")):
        for family in families:
            catalogue = deployed_acoustic_services(family, 0.0, HORIZON_S)
            deployment = asdict(deployment_informed_fixed_configuration(
                FixedConfiguration(**locked), catalogue))
            for index in range(seeds):
                for policy in policies:
                    configuration = deployment if policy == "deployment_fixed" else dict(locked)
                    plan.append({"stage": STAGE, "root": HELD_OUT_ROOT, "part": "scripted",
                                 "group": group, "family": family, "index": index,
                                 "policy": policy,
                                 "configuration_id": design["configuration_ids"][policy],
                                 "configuration": configuration, "environment_seed": None})

    generated = design["part_b_generated"]
    name = generated["environment_config_name"]
    for n in range(generated["environment_seeds"]):
        seed = HELD_OUT_ROOT + generated["seed_offset"] + n
        for policy in policies:
            # Deployment-informed configuration for Part B depends on the
            # realization and is therefore resolved in `execute`, so that
            # planning never generates an environment at a held-out seed.
            plan.append({"stage": STAGE, "root": HELD_OUT_ROOT, "part": "generated",
                         "group": "generated", "family": name, "index": 0,
                         "policy": policy,
                         "configuration_id": design["configuration_ids"][policy],
                         "configuration": None if policy == "deployment_fixed" else dict(locked),
                         "environment_seed": seed})
    return design, plan


def check_environment(failures):
    if sys.executable != PINNED_INTERPRETER:
        failures.append(f"interpreter {sys.executable} is not the pinned {PINNED_INTERPRETER}")
    import cv2  # noqa: PLC0415
    if not hasattr(cv2, "AKAZE_create"):
        failures.append(f"OpenCV {cv2.__version__} lacks AKAZE_create")
    if cv2.__version__ != PINNED_OPENCV:
        failures.append(f"OpenCV {cv2.__version__} is not the pinned {PINNED_OPENCV}")


def check_guards():
    failures = []
    result = subprocess.run([sys.executable, str(VERIFY_LOCK), "--require-authorization"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        failures.append("verify_lock_v2 --require-authorization did not pass:\n"
                        + result.stdout + result.stderr)
    check_environment(failures)
    if COMPLETION_MARKER.exists():
        failures.append("held-out v2 already completed; a second invocation is forbidden")
    if not AUTHORIZATION.exists():
        failures.append("no authorization record present")
        return failures
    authorization = json.loads(AUTHORIZATION.read_text())
    if not authorization.get("authorized"):
        failures.append("authorization record does not authorize execution")
    if authorization.get("executed"):
        failures.append("authorization already marks held-out v2 executed")
    if authorization.get("root") != HELD_OUT_ROOT:
        failures.append("authorization names a different root")
    return failures


def execute(task):
    """Run one member and persist an immutable packet. Resumable, never rewritten."""
    if task["root"] != HELD_OUT_ROOT:
        raise RuntimeError("held-out v2 runner refuses a non-held-out root")
    realization = None
    configuration = task["configuration"]
    if task["part"] == "generated":
        realization = generate_environment(
            load_environment_config(ENVIRONMENT_CONFIG), task["environment_seed"],
            HORIZON_S, DT_S)
        if configuration is None:
            locked = json.loads(FIXED_LOCK.read_text())["configuration"]
            configuration = asdict(deployment_informed_environment_configuration(
                FixedConfiguration(**locked), realization))
    identity = {k: task[k] for k in ("stage", "root", "part", "group", "family",
                                     "index", "policy", "configuration_id",
                                     "environment_seed")}
    identity["configuration"] = configuration
    path = OUT / STAGE / (digest(identity)[:24] + ".json")
    if path.exists():
        stored = json.loads(path.read_text())
        supplied = stored.pop("packet_sha256", None)
        if stored["identity"] != identity or digest(stored) != supplied:
            raise RuntimeError(f"bad resume packet {path}")
        return stored["result"]
    keep = task["part"] == "generated" or task["index"] == 0
    outcome = run_one(HELD_OUT_ROOT, task["family"], task["index"],
                      PolicyKind(task["policy"]), FixedConfiguration(**configuration),
                      horizon_s=HORIZON_S, dt_s=DT_S, image_period_s=IMAGE_PERIOD_S,
                      keep_trace=keep, redesign_version=REDESIGN_VERSION,
                      environment_realization=realization)
    if keep:
        run_result, trace = outcome
        result = asdict(run_result)
        result["causal_trace"] = trace
    else:
        result = asdict(outcome)
    if realization is not None:
        result["environment_digest"] = realization.digest
    result["configuration_id"] = task["configuration_id"]
    result["part"] = task["part"]
    result["group"] = task["group"]
    result["environment_seed"] = task["environment_seed"]
    packet = {"schema": "study3_heldout_v2_packet_v1", "identity": identity, "result": result}
    packet["packet_sha256"] = digest(packet)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(packet, sort_keys=True, indent=2, allow_nan=True) + "\n")
    os.replace(temporary, path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", action="store_true",
                        help="enumerate the frozen plan and exit; executes nothing")
    parser.add_argument("--execute", action="store_true",
                        help="spend the one-shot authorization")
    parser.add_argument("--workers", type=int, default=4)
    arguments = parser.parse_args()

    design, plan = build_plan()
    counts = {part: sum(1 for t in plan if t["part"] == part)
              for part in ("scripted", "generated")}
    summary = {"schema": "study3_heldout_v2_summary_v1", "stage": STAGE,
               "root": HELD_OUT_ROOT, "policies": design["policies"],
               "planned_scripted": counts["scripted"],
               "planned_generated": counts["generated"],
               "planned_total": len(plan),
               "design_scripted": design["executions"]["scripted"],
               "design_generated": design["executions"]["generated"],
               "design_total": design["executions"]["total"],
               "execution_parameters": {"horizon_s": HORIZON_S, "dt_s": DT_S,
                                        "image_period_s": IMAGE_PERIOD_S,
                                        "redesign_version": REDESIGN_VERSION,
                                        "provenance": "inherited from the frozen "
                                                      "development runners"},
               "plan_digest": digest([{k: t[k] for k in
                                       ("part", "family", "index", "policy",
                                        "configuration_id", "environment_seed")}
                                      for t in plan])}

    if arguments.plan or not arguments.execute:
        summary["executed"] = False
        print(json.dumps(summary, sort_keys=True, indent=2))
        matches = (summary["planned_scripted"] == summary["design_scripted"]
                   and summary["planned_generated"] == summary["design_generated"]
                   and summary["planned_total"] == summary["design_total"])
        print(f"\nplan matches frozen design: {matches}")
        print("no environment was generated at a held-out seed")
        return 0 if matches else 1

    failures = check_guards()
    if failures:
        print("held-out v2 execution REFUSED")
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
    summary["result_digest"] = digest(sorted(
        (r["part"], r["family"], r["policy"], r["index"],
         r.get("environment_seed"), r["trace_digest"]) for r in rows))
    COMPLETION_MARKER.write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    authorization = json.loads(AUTHORIZATION.read_text())
    authorization["executed"] = True
    authorization["execution_record"] = {
        "invocation": 1, "packets": len(rows),
        "result_digest": summary["result_digest"],
        "elapsed_s": summary["elapsed_s"]}
    AUTHORIZATION.write_text(json.dumps(authorization, sort_keys=True, indent=2) + "\n")
    print(json.dumps(summary, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
