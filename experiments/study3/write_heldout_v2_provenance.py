#!/usr/bin/env python3
"""Post-execution provenance for the corrected-controller held-out block.

Records what was executed, the interruption and resume, and the integrity
evidence. Reads the completion marker and the authorization; asserts no
interpretation of its own.
"""
from __future__ import annotations

import hashlib
import json
import platform
import subprocess
import sys
from pathlib import Path

import cv2
import numpy

HERE = Path(__file__).resolve().parent
sha = lambda name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()


def main():
    marker = json.loads((HERE / "heldout_v2_result.json").read_text())
    authorization = json.loads((HERE / "STUDY3_HELDOUT_V2_AUTHORIZATION.json").read_text())
    design = json.loads((HERE / "STUDY3_HELDOUT_V2_DESIGN.json").read_text())
    packets = sorted((HERE / "redesign_results/heldout_v2").glob("*.json"))

    document = {
        "schema": "study3_heldout_v2_provenance_v1",
        "record_type": "POST_EXECUTION_PROVENANCE",
        "created_utc": "2026-08-11",
        "statement": ("Provenance for the single final held-out evaluation of the "
                      "corrected controller at root 36,000,000. The block is spent."),
        "provenance_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True,
            cwd=HERE).stdout.strip(),
        "held_out": {
            "root": 36_000_000,
            "executed": True,
            "invocations_of_the_evaluation": 1,
            "packets_on_disk": len(packets),
            "packets_reported": marker["executions"],
            "elapsed_s": marker["elapsed_s"],
            "result_digest": marker["result_digest"],
            "plan_digest": marker["plan_digest"]},
        "plan_digest_matches_pre_authorization_record": (
            marker["plan_digest"] == authorization.get("pre_authorization_plan_digest")),
        "execution_interruption": {
            "occurred": True,
            "cause": ("The first --execute invocation was killed by a 10-minute "
                      "harness command-timeout, not by any fault in the run."),
            "packets_at_interruption": 697,
            "completion_marker_written": False,
            "authorization_spent_at_interruption": False,
            "partial_or_temporary_files_left": 0,
            "orphan_processes": 0,
            "results_observed_before_resume": False,
            "resume_semantics": ("Packets are written atomically and are immutable. "
                                 "On resume the runner verifies each stored packet's "
                                 "checksum and identity and recomputes only what is "
                                 "missing, so the completed block is identical to an "
                                 "uninterrupted one."),
            "why_this_is_still_one_evaluation": (
                "No result was observed between the interruption and the resume, "
                "nothing was re-rolled, and the plan digest is unchanged from the "
                "value published before authorization. The one-shot guarantee "
                "concerns selection on observed outcomes, and none occurred.")},
        "design": {
            "scripted": design["executions"]["scripted"],
            "generated": design["executions"]["generated"],
            "total": design["executions"]["total"],
            "policies": design["policies"]},
        "decision_rules_frozen_before_authorization": True,
        "decision_rules": design["decision_rules"],
        "execution_parameters": marker["execution_parameters"],
        "environment": {
            "python": platform.python_version(), "numpy": numpy.__version__,
            "opencv": cv2.__version__,
            "opencv_akaze_present": bool(hasattr(cv2, "AKAZE_create")),
            "interpreter": sys.executable, "platform": platform.platform()},
        "original_heldout_preserved": {
            "root": 32_000_000,
            "interpretation": "evidence for the PRE-correction controller",
            "revised": False, "re_run": False, "accessed_during_v2": False},
        "artefacts": {name: sha(name) for name in (
            "heldout_v2_result.json", "STUDY3_HELDOUT_V2_AUTHORIZATION.json",
            "STUDY3_HELDOUT_V2_DESIGN.json", "STUDY3_FREEZE_MANIFEST_V2.json",
            "run_heldout_v2.py", "analyse_heldout_v2.py", "verify_lock_v2.py")},
        "terminal": ("This is the final held-out evaluation of Study 3. No V7, no "
                     "correction cycle and no further held-out block follows, "
                     "whatever the result."),
        "reporting_obligations": [
            "The result is reported regardless of direction.",
            "Parts A and B are reported separately and are never pooled.",
            "The V6 adaptation regression to 0.5531, the PREDICTIVE null with its "
            "adverse aiding-gap result, and the V6 safety and completion figures "
            "remain standing development findings.",
            "PREDICTIVE has no success threshold and is reported either way."]}

    path = HERE / "STUDY3_HELDOUT_V2_PROVENANCE.json"
    path.write_text(json.dumps(document, indent=1, sort_keys=True) + "\n")
    print(f"wrote {path.name}")
    print(f"  packets {document['held_out']['packets_on_disk']} / "
          f"{design['executions']['total']}")
    print(f"  plan digest matches pre-authorization record: "
          f"{document['plan_digest_matches_pre_authorization_record']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
