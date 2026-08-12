#!/usr/bin/env python3
"""Study 3 one-shot held-out execution runner.

Executes exactly the design frozen in ``STUDY3_HELDOUT_DESIGN_V1.json``. It
makes no scientific choice of its own: every family, policy, seed count and
configuration is read from a frozen record, and the execution parameterisation
is inherited unchanged from the frozen development runners.

Guards, in the order they are applied:

  1. ``verify_lock.py --require-authorization`` must exit 0;
  2. the interpreter must be the pinned one and OpenCV must provide AKAZE;
  3. the root must be exactly 32,000,000;
  4. a completed held-out run must not already exist;
  5. the authorization must not already be marked executed.

``--plan`` enumerates the execution plan and exits without running anything and
without requiring authorization. That is the mechanism-test path.
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
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FixedConfiguration, PolicyKind, deployed_acoustic_services,
    deployment_informed_fixed_configuration, run_one)

HELD_OUT_ROOT = 32_000_000
STAGE = "heldout"
OUT = HERE / "redesign_results"
DESIGN = HERE / "STUDY3_HELDOUT_DESIGN_V1.json"
AUTHORIZATION = HERE / "STUDY3_HELDOUT_AUTHORIZATION_V1.json"
MANIFEST = HERE / "STUDY3_FREEZE_MANIFEST_V1.json"
FIXED_LOCK = OUT / "infrastructure_fixed_baseline_lock.json"
COMPLETION_MARKER = HERE / "heldout_result.json"
VERIFY_LOCK = HERE / "verify_lock.py"

#: Inherited unchanged from run_redesign_development.py and
#: run_final_validation_v5.py. Not a choice made here.
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
    """Enumerate the frozen design. Pure; touches no simulation."""
    design = load_design()
    locked = json.loads(FIXED_LOCK.read_text())["configuration"]
    policies = design["policies"]
    plan = []
    deployment_configurations = {}
    groups = ((design["primary_families"], design["seeds_per_primary_family_policy"], "primary"),
              (design["control_families"], design["seeds_per_control_family_policy"], "control"))
    for families, seeds, group in groups:
        for family in families:
            catalogue = deployed_acoustic_services(family, 0.0, HORIZON_S)
            deployment = asdict(deployment_informed_fixed_configuration(
                FixedConfiguration(**locked), catalogue))
            deployment_configurations[family] = deployment
            for index in range(seeds):
                for policy in policies:
                    if policy == "fixed":
                        configuration_id, configuration = "fixed_155", dict(locked)
                    elif policy == "deployment_fixed":
                        configuration_id, configuration = ("deployment_informed_fixed_155",
                                                           deployment)
                    elif policy == "reactive":
                        configuration_id, configuration = ("reactive_shared_fixed_155",
                                                           dict(locked))
                    else:
                        raise SystemExit(f"policy not in the frozen design: {policy}")
                    plan.append({"stage": STAGE, "root": HELD_OUT_ROOT, "group": group,
                                 "family": family, "index": index, "policy": policy,
                                 "configuration_id": configuration_id,
                                 "configuration": configuration})
    return design, plan, deployment_configurations


def check_environment(failures):
    if sys.executable != PINNED_INTERPRETER:
        failures.append(f"interpreter {sys.executable} is not the pinned {PINNED_INTERPRETER}")
    import cv2  # noqa: PLC0415
    if not hasattr(cv2, "AKAZE_create"):
        failures.append(f"OpenCV {cv2.__version__} lacks AKAZE_create")
    if cv2.__version__ != PINNED_OPENCV:
        failures.append(f"OpenCV {cv2.__version__} is not the pinned {PINNED_OPENCV}")


def check_guards():
    """Every precondition for spending the one-shot authorization."""
    failures = []
    result = subprocess.run([sys.executable, str(VERIFY_LOCK), "--require-authorization"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        failures.append("verify_lock --require-authorization did not pass:\n"
                        + result.stdout + result.stderr)
    check_environment(failures)
    if COMPLETION_MARKER.exists():
        failures.append("held-out already completed; a second invocation is forbidden")
    authorization = json.loads(AUTHORIZATION.read_text())
    if not authorization.get("authorized"):
        failures.append("authorization record does not authorize execution")
    if authorization.get("executed"):
        failures.append("authorization already marks held-out executed")
    if authorization.get("root") != HELD_OUT_ROOT:
        failures.append("authorization names a different root")
    return failures


def execute(task):
    """Run one member and persist an immutable packet. Resumable, never rewritten."""
    if task["root"] != HELD_OUT_ROOT:
        raise RuntimeError("held-out runner refuses a non-held-out root")
    identity = {k: task[k] for k in ("stage", "root", "family", "index", "policy",
                                     "configuration_id", "configuration")}
    path = OUT / STAGE / (digest(identity)[:24] + ".json")
    if path.exists():
        stored = json.loads(path.read_text())
        supplied = stored.pop("packet_sha256", None)
        if stored["identity"] != identity or digest(stored) != supplied:
            raise RuntimeError(f"bad resume packet {path}")
        return stored["result"]
    outcome = run_one(HELD_OUT_ROOT, task["family"], task["index"],
                      PolicyKind(task["policy"]),
                      FixedConfiguration(**task["configuration"]),
                      horizon_s=HORIZON_S, dt_s=DT_S, image_period_s=IMAGE_PERIOD_S,
                      keep_trace=(task["index"] == 0), redesign_version=REDESIGN_VERSION)
    if task["index"] == 0:
        run_result, trace = outcome
        result = asdict(run_result)
        result["causal_trace"] = trace
    else:
        result = asdict(outcome)
    result["configuration_id"] = task["configuration_id"]
    result["group"] = task["group"]
    packet = {"schema": "study3_heldout_packet_v1", "identity": identity, "result": result}
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

    design, plan, deployment = build_plan()
    summary = {"stage": STAGE, "root": HELD_OUT_ROOT,
               "policies": design["policies"],
               "primary_families": design["primary_families"],
               "control_families": design["control_families"],
               "seeds_per_primary_family_policy": design["seeds_per_primary_family_policy"],
               "seeds_per_control_family_policy": design["seeds_per_control_family_policy"],
               "planned_primary": sum(1 for t in plan if t["group"] == "primary"),
               "planned_control": sum(1 for t in plan if t["group"] == "control"),
               "planned_total": len(plan),
               "design_primary": design["executions"]["primary"],
               "design_control": design["executions"]["controls"],
               "design_total": design["executions"]["total"],
               "execution_parameters": {"horizon_s": HORIZON_S, "dt_s": DT_S,
                                        "image_period_s": IMAGE_PERIOD_S,
                                        "redesign_version": REDESIGN_VERSION,
                                        "provenance": "inherited from the frozen "
                                                      "development runners"},
               "plan_digest": digest([{k: t[k] for k in
                                       ("family", "index", "policy", "configuration_id")}
                                      for t in plan])}

    if arguments.plan or not arguments.execute:
        summary["executed"] = False
        print(json.dumps(summary, sort_keys=True, indent=2))
        matches = (summary["planned_primary"] == summary["design_primary"]
                   and summary["planned_control"] == summary["design_control"]
                   and summary["planned_total"] == summary["design_total"])
        print(f"\nplan matches frozen design: {matches}")
        return 0 if matches else 1

    failures = check_guards()
    if failures:
        print("held-out execution REFUSED")
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
    summary["deployment_informed_configurations"] = deployment
    summary["result_digest"] = digest(sorted(
        (r["family"], r["policy"], r["index"], r["trace_digest"]) for r in rows))
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
