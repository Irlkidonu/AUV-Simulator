#!/usr/bin/env python3
"""Build/verify a candidate DEVELOPMENT lock; never authorizes held-out."""
from __future__ import annotations
import argparse,hashlib,json,platform,sys
from pathlib import Path
import cv2,numpy as np

HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
LOCK=HERE/"DEVELOPMENT_CANDIDATE_LOCK.json"
ALLOW=(
 "experiments/study3/STUDY3_PROTOCOL.md","experiments/study3/STUDY3_DESIGN.json",
 "experiments/study3/STUDY3_SEED_REGISTRY.json","experiments/study3/run_development.py",
 "experiments/study3/analyse_confirmation.py",
 "src/uuv_mode_aware_navigation/uuv_mode_aware_navigation/localization/optical_v4.py",
 "src/uuv_mode_aware_navigation/uuv_mode_aware_navigation/study3/policies.py",
 "src/uuv_mode_aware_navigation/uuv_mode_aware_navigation/study3/scenarios.py",
 "src/uuv_mode_aware_navigation/uuv_mode_aware_navigation/study3/simulation.py",
 "src/uuv_mode_aware_navigation/uuv_mode_aware_navigation/platform_v2.py",
 "src/uuv_mode_aware_navigation/uuv_mode_aware_navigation/capability/prediction.py",
 "experiments/study3/development_results/fixed_v2_summary.json",
 "experiments/study3/development_results/adaptive_v6_summary.json",
 "experiments/study3/development_results/confirmation_v6_summary.json",
 "experiments/study3/development_results/confirmation_v6_analysis.json")
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def record():return {"schema":"study3_development_candidate_lock_v1","held_out_authorized":False,
 "forbidden_root":32_000_000,"files":{p:sha(REPO/p) for p in ALLOW},
 "environment":{"python":sys.version.split()[0],"numpy":np.__version__,"opencv":cv2.__version__,
 "platform":platform.platform()}}
def main():
 p=argparse.ArgumentParser();p.add_argument("command",choices=("build","verify"));a=p.parse_args()
 if a.command=="build":LOCK.write_text(json.dumps(record(),sort_keys=True,indent=2)+"\n")
 if not LOCK.exists():raise SystemExit("candidate lock missing")
 expected=json.loads(LOCK.read_text());actual=record()
 if expected!=actual:raise SystemExit("DEVELOPMENT LOCK VERIFY FAIL")
 print("DEVELOPMENT LOCK VERIFY PASS (held-out remains unauthorized)")
if __name__=="__main__":main()
