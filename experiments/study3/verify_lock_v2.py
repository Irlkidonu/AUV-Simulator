#!/usr/bin/env python3
"""Study 3 `verify-lock`, required by STUDY3_PROTOCOL.md section 10.

Section 10: "`verify-lock` must pass before execution and the runner must reject
unregistered roots, unknown policies, changed family counts, duplicate attempts
and held-out execution without a separate authorization record."

Read-only. Exits non-zero on any violation. This is the gate a held-out runner
must call and honour before touching root 32,000,000.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[1]
HELD_OUT_ROOT = 36_000_000

MANIFEST = HERE / "STUDY3_FREEZE_MANIFEST_V2.json"
DESIGN = HERE / "STUDY3_HELDOUT_V2_DESIGN.json"
DECISION = HERE / "STUDY3_FREEZE_DECISION_V1.md"
AUTHORIZATION = HERE / "STUDY3_HELDOUT_V2_AUTHORIZATION.json"
REGISTRY = HERE / "STUDY3_SEED_REGISTRY.json"
FORBIDDEN_OUTPUTS = ("heldout_v2_result.json", "campaign_complete.json",
                     "heldout_v2_attempt.json")


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check(condition, message, failures):
    if not condition:
        failures.append(message)
    return condition


def main(require_authorization=False):
    failures: list[str] = []

    # 1. The freeze manifest exists and every allowlisted file still matches.
    if not check(MANIFEST.exists(), "freeze manifest missing", failures):
        print("\n".join(failures))
        return 1
    manifest = json.loads(MANIFEST.read_text())
    drifted = []
    for group, files in manifest["files"].items():
        for relative, digest in files.items():
            path = REPOSITORY / relative
            if not path.exists():
                drifted.append(f"{relative} (missing)")
            elif _sha(path) != digest:
                drifted.append(relative)
    check(not drifted, f"frozen files changed since the freeze: {drifted}", failures)
    check(manifest["file_count"] == sum(len(v) for v in manifest["files"].values()),
          "manifest file_count inconsistent", failures)
    check(not manifest["missing_from_allowlist"],
          "allowlist incomplete", failures)

    # 2. Environment is AKAZE-capable; P5-v4 cannot run without it.
    import cv2  # noqa: PLC0415
    check(hasattr(cv2, "AKAZE_create"),
          f"OpenCV {cv2.__version__} lacks AKAZE_create; P5-v4 cannot execute", failures)

    # 3. Decision and design records exist and are self-consistent.
    check(DECISION.exists(), "researcher freeze decision missing", failures)
    if check(DESIGN.exists(), "held-out design missing", failures):
        design = json.loads(DESIGN.read_text())
        policies = design["policies"]
        executions = design["executions"]
        a = design["part_a_scripted"]
        scripted = (len(a["primary_families"]) * a["seeds_per_primary_family_policy"]
                    + len(a["control_families"]) * a["seeds_per_control_family_policy"]) * len(policies)
        generated = design["part_b_generated"]["environment_seeds"] * len(policies)
        check(scripted == executions["scripted"], "scripted execution count inconsistent", failures)
        check(generated == executions["generated"], "generated execution count inconsistent", failures)
        check(scripted + generated == executions["total"], "total execution count inconsistent", failures)
        check(design["held_out_root"] == HELD_OUT_ROOT, "unexpected held-out root", failures)
        check(design["maximum_invocations"] == 1, "held-out is one-shot", failures)
        check(design["executed"] is False, "design already marks held-out executed", failures)
        known = {"fixed", "deployment_fixed", "reactive", "robust_fusion", "predictive"}
        check(set(policies) <= known, f"unknown policy in design: {set(policies) - known}", failures)

    # 4. Seed separation: the held-out root is reserved and never a development root.
    registry = json.loads(REGISTRY.read_text())
    development = {r for values in registry["development_attempt_roots"].values() for r in values}
    development |= set(registry["development"].values())
    check(HELD_OUT_ROOT not in development, "held-out root appears as a development root", failures)
    check(all(31_000_000 <= r < 32_000_000 for r in development),
          "a development root lies outside the 31-million band", failures)
    # The v2 band must be disjoint from every band ever used, including the
    # original held-out block, which is preserved and not re-run.
    used_bands = {20, 22, 31, 32, 33, 34, 35, 40}
    check(HELD_OUT_ROOT // 1_000_000 not in used_bands,
          f"held-out v2 root {HELD_OUT_ROOT} lies in a used band", failures)
    check(HELD_OUT_ROOT != 32_000_000,
          "held-out v2 must not reuse the original held-out root", failures)
    original = HERE / "heldout_result.json"
    check(original.exists(),
          "the original held-out record is missing; it must be preserved", failures)

    # 5. No held-out output may already exist.
    existing = [name for name in FORBIDDEN_OUTPUTS if (HERE / name).exists()]
    check(not existing, f"held-out output already present: {existing}", failures)

    # 6. Authorization is a SEPARATE record. Absence is not a failure of the
    #    lock itself; it is a failure only when execution is being attempted.
    authorized = False
    if AUTHORIZATION.exists():
        authorization = json.loads(AUTHORIZATION.read_text())
        authorized = bool(authorization.get("authorized")) and \
            authorization.get("root") == HELD_OUT_ROOT and \
            authorization.get("executed") is False
    if require_authorization:
        check(authorized, "no valid separate held-out authorization record", failures)

    if failures:
        print("verify-lock FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("verify-lock PASS: frozen files match, environment is AKAZE-capable, "
          "design and seed separation are consistent, no held-out output exists")
    print(f"  authorization record present and valid: {authorized}")
    print(f"  held-out root {HELD_OUT_ROOT} remains unexecuted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main("--require-authorization" in sys.argv))
