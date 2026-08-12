"""Headless discovery and validation CLI; campaign execution stays explicit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .benchmarks import load_benchmark


def _root(): return Path(__file__).resolve().parents[3]


def main(argv=None):
    parser=argparse.ArgumentParser(prog="auv-sim")
    sub=parser.add_subparsers(dest="command",required=True)
    sub.add_parser("list-benchmarks")
    validate=sub.add_parser("validate-benchmark"); validate.add_argument("path",type=Path)
    sub.add_parser("platform-status")
    platform_config=sub.add_parser("validate-platform-config")
    platform_config.add_argument("path",type=Path)
    args=parser.parse_args(argv)
    root=_root()
    if args.command=="list-benchmarks":
        for path in sorted((root/"benchmarks").glob("*.json")):
            try:
                identity=load_benchmark(path); print(f"{identity.benchmark}\t{path.name}")
            except (ValueError,KeyError):
                continue
        return 0
    if args.command=="validate-benchmark":
        identity=load_benchmark(args.path)
        print(json.dumps(identity.__dict__,indent=2,sort_keys=True))
        return 0
    if args.command=="validate-platform-config":
        configuration=json.loads(args.path.read_text())
        required={"identifier","data_class","optical_frontend","capability_filter",
                  "capability_prediction_horizon_s","fixed_lag_s","active_recovery",
                  "energy_policy_objective","trn_enabled","held_out"}
        if set(configuration)!=required:
            raise ValueError("platform configuration has missing or unknown fields")
        if configuration["held_out"]:
            raise ValueError("development CLI refuses held-out configuration")
        if configuration["energy_policy_objective"]:
            raise ValueError("energy-aware policy research is outside platform-v2 scope")
        if configuration["trn_enabled"]:
            raise ValueError("real TRN is deferred")
        print(json.dumps(configuration,indent=2,sort_keys=True))
        return 0
    if args.command=="platform-status":
        print("study2_legacy: frozen")
        print("P5-v1: FAIL (immutable)")
        print("P5-v2: terminal execution FAIL; no scientific result")
        print("P5-v3: feasibility FAIL (immutable)")
        print("P5-v4: development feasibility PASS; turbid availability unresolved")
        print("P6-v1: FAIL (immutable)")
        print("P6-v2: FAIL; promising but incomplete/deferred")
        print("held-out evaluation: not executed")
        return 0
    return 2


if __name__=="__main__": raise SystemExit(main())
