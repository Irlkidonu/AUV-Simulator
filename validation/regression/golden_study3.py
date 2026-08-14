#!/usr/bin/env python3
"""Study-3 golden rows: an engineering regression fixture, not scientific evidence.

Captured at M0, before any physics work, so that every later milestone can prove
the frozen numerical path still produces identical results. Re-run with
``--check`` to compare against the captured baseline.

Seed root 39,000,000 is an *engineering* band. It appears in no scientific
registry, no freeze manifest and no reported result, and it is deliberately
outside every band the study used (20.x, 22.x, 31.x, 32M, 33.x, 34.x, 35.x, 36M).
These rows exist only to detect regression and may be regenerated freely.

Read-only with respect to the simulator: it imports and calls ``run_one`` and
writes nothing outside this directory.

    PYTHONPATH=src/uuv_mode_aware_navigation python3 baselines/M0/golden_study3.py
    PYTHONPATH=src/uuv_mode_aware_navigation python3 baselines/M0/golden_study3.py --check
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

from uuv_mode_aware_navigation.study3 import (
    FixedConfiguration, PolicyKind, run_one,
)

HERE = Path(__file__).resolve().parent
GOLDEN = HERE / "golden_study3.json"

ROOT = 39_000_000
HORIZON_S = 120.0
REDESIGN_VERSION = 3

#: The locked Study-3 configuration (fixed_155), as used by the held-out design.
FIXED = FixedConfiguration(optical_channel="lidar", altitude_m=5.0, speed_mps=0.5,
                           acoustic_technique="usbl", fusion_mode="weight")

FAMILIES = ("S3_NOMINAL", "S3_OPTICAL_GRADUAL", "S3_DVL_GRADUAL",
            "S3_ACOUSTIC_GEOMETRY_ASYNC", "S3_RECOVERY", "S3_NO_RECOVERY")
INDICES = (0, 1)
POLICIES = (PolicyKind.FIXED, PolicyKind.DEPLOYMENT_FIXED,
            PolicyKind.REACTIVE, PolicyKind.PREDICTIVE)

#: Wall-clock fields are excluded: they are not deterministic and are not results.
VOLATILE = {"runtime_s", "mean_policy_runtime_ms", "mode_telemetry"}


def _canonical(value):
    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        if math.isinf(value):
            return "inf" if value > 0 else "-inf"
        return repr(value)          # exact round-trip, no formatting loss
    if isinstance(value, (list, tuple)):
        return [_canonical(v) for v in value]
    if isinstance(value, dict):
        return {k: _canonical(v) for k, v in sorted(value.items())}
    return value


def rows():
    out = []
    for family in FAMILIES:
        for index in INDICES:
            for kind in POLICIES:
                result = run_one(ROOT, family, index, kind, FIXED,
                                 horizon_s=HORIZON_S,
                                 redesign_version=REDESIGN_VERSION)
                record = {k: _canonical(v) for k, v in asdict(result).items()
                          if k not in VOLATILE}
                out.append(record)
    return out


def digest(records):
    blob = json.dumps(records, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="compare against the captured baseline")
    args = parser.parse_args(argv)

    records = rows()
    current = digest(records)

    if args.check:
        if not GOLDEN.exists():
            print("golden file missing; capture it first")
            return 2
        stored = json.loads(GOLDEN.read_text())
        if stored["digest"] == current and stored["rows"] == records:
            print(f"GOLDEN ROWS: PASS  ({len(records)} rows, digest {current})")
            return 0
        print(f"GOLDEN ROWS: FAIL\n  expected {stored['digest']}\n  actual   {current}")
        for was, now in zip(stored["rows"], records):
            if was != now:
                changed = [k for k in now if was.get(k) != now.get(k)]
                print(f"  {now['family']}/{now['index']}/{now['policy']}: {changed}")
        return 1

    GOLDEN.write_text(json.dumps(
        {"root": ROOT, "horizon_s": HORIZON_S,
         "redesign_version": REDESIGN_VERSION,
         "fixed_configuration": asdict(FIXED),
         "excluded_volatile_fields": sorted(VOLATILE),
         "row_count": len(records), "digest": current, "rows": records},
        indent=2, sort_keys=True) + "\n")
    print(f"captured {len(records)} rows -> {GOLDEN}")
    print(f"digest {current}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
