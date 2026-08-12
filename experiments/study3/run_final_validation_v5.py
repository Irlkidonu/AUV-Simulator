#!/usr/bin/env python3
"""Study 3 final DEVELOPMENT validation V5.

Executes the campaign registered in ``STUDY3_FINAL_VALIDATION_V5_PROTOCOL.md``
and writes one immutable packet per run. It performs no analysis and applies no
threshold: the protocol's gating criteria are evaluated separately, once, by
``analyse_final_validation_v5.py``.

Composes already-tested primitives only. Held-out root 32,000,000 is refused.
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
sys.path.insert(0, str(HERE.parents[1] / "src/uuv_mode_aware_navigation"))

from uuv_mode_aware_navigation.study3 import (  # noqa: E402
    FAMILIES, FixedConfiguration, PolicyKind,
    deployed_acoustic_services, deployment_informed_fixed_configuration,
    deployment_informed_environment_configuration, generate_environment,
    load_environment_config, run_one)

SMOKE_ROOT = 31_899_000
VALIDATION_ROOT = 31_900_000
HELD_OUT_ROOT = 32_000_000
OUT = HERE / "redesign_results"
STAGE = "final_validation_v5"
ENVIRONMENT_CONFIG = HERE / "examples/moderate_severe_variable_environment.json"
POLICIES = ("fixed", "deployment_fixed", "reactive")
INDICES = 17
ENVIRONMENT_SEEDS = 12
HORIZON_S = 180.0
DT_S = 2.0
#: The locked universal baseline. Not derived from any campaign result.
LOCKED = dict(optical_channel="lidar", altitude_m=5.0, speed_mps=0.5,
              acoustic_technique="usbl", fusion_mode="weight")


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=True).encode()).hexdigest()


def _guard(root):
    if root == HELD_OUT_ROOT:
        raise SystemExit("held-out root is forbidden by this protocol")
    if not 31_000_000 <= root < 32_000_000:
        raise SystemExit("V5 requires a 31-million DEVELOPMENT root")


def execute(task):
    """Run one paired member and persist its packet. Resumable, never rewritten."""
    stage, root, part, family, index, policy, cid, config, seed = task
    _guard(root)
    identity = {"stage": stage, "root": root, "part": part, "family": family,
                "index": index, "policy": policy, "configuration_id": cid,
                "configuration": config, "environment_seed": seed}
    path = OUT / stage / (digest(identity)[:24] + ".json")
    if path.exists():
        stored = json.loads(path.read_text())
        supplied = stored.pop("packet_sha256", None)
        if stored["identity"] != identity or digest(stored) != supplied:
            raise RuntimeError(f"bad resume packet {path}")
        return stored["result"]

    fixed = FixedConfiguration(**config)
    if part == "generated":
        realization = generate_environment(
            load_environment_config(ENVIRONMENT_CONFIG), seed, HORIZON_S, DT_S)
        outcome = run_one(root, family, index, PolicyKind(policy), fixed,
                          horizon_s=HORIZON_S, dt_s=DT_S, image_period_s=4.0,
                          keep_trace=True, redesign_version=3,
                          environment_realization=realization)
        run_result, trace = outcome
        result = asdict(run_result)
        result["causal_trace"] = trace
        result["environment_digest"] = realization.digest
    else:
        # Trace retained on index zero only, matching prior scripted stages.
        keep = index == 0
        outcome = run_one(root, family, index, PolicyKind(policy), fixed,
                          horizon_s=HORIZON_S, dt_s=DT_S, image_period_s=4.0,
                          keep_trace=keep, redesign_version=3)
        if keep:
            run_result, trace = outcome
            result = asdict(run_result)
            result["causal_trace"] = trace
        else:
            result = asdict(outcome)
    result["configuration_id"] = cid
    result["part"] = part
    packet = {"schema": "study3_final_validation_v5_packet_v1",
              "identity": identity, "result": result}
    packet["packet_sha256"] = digest(packet)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(packet, sort_keys=True, indent=2,
                                    allow_nan=True) + "\n")
    os.replace(temporary, path)
    return result


def scripted_tasks(root, indices):
    """Part A: ten scripted families, three policies, paired indices."""
    tasks = []
    deployment_configurations = {}
    for family in FAMILIES:
        catalogue = deployed_acoustic_services(family, 0.0, HORIZON_S)
        deployment = asdict(deployment_informed_fixed_configuration(
            FixedConfiguration(**LOCKED), catalogue))
        deployment_configurations[family] = deployment
        for index in indices:
            tasks += [
                (STAGE, root, "scripted", family, index, "fixed",
                 "fixed_155", dict(LOCKED), None),
                (STAGE, root, "scripted", family, index, "deployment_fixed",
                 "deployment_informed_fixed_155", deployment, None),
                (STAGE, root, "scripted", family, index, "reactive",
                 "reactive_shared_fixed_155", dict(LOCKED), None)]
    return tasks, deployment_configurations


def generated_tasks(root, seeds):
    """Part B: one shared generated environment per seed, three policies."""
    config = load_environment_config(ENVIRONMENT_CONFIG)
    tasks = []
    for seed in seeds:
        realization = generate_environment(config, seed, HORIZON_S, DT_S)
        deployment = asdict(deployment_informed_environment_configuration(
            FixedConfiguration(**LOCKED), realization))
        tasks += [
            (STAGE, root, "generated", config.name, 0, "fixed",
             "fixed_155", dict(LOCKED), seed),
            (STAGE, root, "generated", config.name, 0, "deployment_fixed",
             "deployment_informed_fixed_155", deployment, seed),
            (STAGE, root, "generated", config.name, 0, "reactive",
             "reactive_shared_fixed_155", dict(LOCKED), seed)]
    return tasks


def run(tasks, workers):
    started = time.perf_counter()
    if workers <= 1:
        rows = [execute(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(execute, tasks))
    return rows, time.perf_counter() - started


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", action="store_true",
                        help="tiny plumbing check on the smoke root; not the campaign")
    parser.add_argument("--workers", type=int, default=4)
    arguments = parser.parse_args()

    if arguments.smoke:
        root = SMOKE_ROOT
        scripted, deployment = scripted_tasks(root, range(1))
        scripted = [t for t in scripted if t[3] == FAMILIES[0]]
        generated = generated_tasks(root, [31_899_100])
        tasks = scripted + generated
    else:
        root = VALIDATION_ROOT
        scripted, deployment = scripted_tasks(root, range(INDICES))
        generated = generated_tasks(root, [root + 1_000 + n
                                           for n in range(ENVIRONMENT_SEEDS)])
        tasks = scripted + generated
    _guard(root)

    rows, elapsed = run(tasks, arguments.workers)
    summary = {"schema": "study3_final_validation_v5_summary_v1",
               "stage": STAGE, "root": root, "smoke": bool(arguments.smoke),
               "executions": len(rows), "elapsed_s": round(elapsed, 3),
               "scripted_executions": len(scripted),
               "generated_executions": len(generated),
               "policies": list(POLICIES),
               "locked_universal_baseline": dict(LOCKED),
               "deployment_informed_configurations": deployment,
               "environment_config": str(ENVIRONMENT_CONFIG.name),
               "held_out_accessed": False,
               "result_digest": digest(sorted(
                   (r["family"], r["policy"], r["index"],
                    r.get("environment_digest"), r["trace_digest"]) for r in rows))}
    name = f"{STAGE}{'_smoke' if arguments.smoke else ''}_summary.json"
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    print(json.dumps(summary, sort_keys=True, indent=2))


if __name__ == "__main__":
    main()
