#!/usr/bin/env python3
"""Analyze only the valid root-31,820,000 Study 3 DEVELOPMENT confirmation."""
from __future__ import annotations
import json
from collections import Counter
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
PACKETS=HERE/"redesign_results"/"infrastructure_confirmation"
OUT=HERE/"redesign_results"/"infrastructure_confirmation_analysis.json"
ROOT=31_820_000
POLICIES=("fixed","robust_fusion","reactive","predictive")
PRIMARY=("S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
         "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
         "S3_COMPOUND_DVL_ACOUSTIC")
METRICS=("completed","safety_violation","rmse_transition_m","overall_rmse_m",
         "peak_error_m","unaided_time_s","longest_unaided_gap_s",
         "survey_coverage_fraction","mission_duration_s","unnecessary_interventions",
         "mode_switches","optical_fixes","acoustic_fixes")

def load():
 rows=[]
 for path in PACKETS.glob("*.json"):
  packet=json.loads(path.read_text())
  if packet["identity"]["root"]==ROOT:rows.append(packet["result"])
 assert len(rows)==680
 assert {(r["family"],r["index"],r["policy"]) for r in rows}=={
  (f,i,p) for f in sorted({r["family"] for r in rows}) for i in range(17) for p in POLICIES}
 return rows

def means(rows,families,policy):
 a=[r for r in rows if r["family"] in families and r["policy"]==policy]
 return {m:float(np.mean([r[m] for r in a])) for m in METRICS}

def paired(rows,families,a,b):
 lookup={(r["family"],r["index"],r["policy"]):r for r in rows}
 rng=np.random.default_rng(31_820_000);out={}
 for metric in METRICS:
  by={f:np.array([float(lookup[f,i,a][metric])-float(lookup[f,i,b][metric])
                  for i in range(17)]) for f in families}
  estimate=float(np.mean([x.mean() for x in by.values()]))
  boot=np.empty(10_000)
  for k in range(10_000):
   boot[k]=np.mean([x[rng.integers(0,len(x),len(x))].mean() for x in by.values()])
  out[metric]={"difference":estimate,"ci95":[float(x) for x in np.quantile(boot,[.025,.975])]}
 return out

def main():
 rows=load();families=sorted({r["family"] for r in rows})
 family={f:{p:means(rows,(f,),p) for p in POLICIES} for f in families}
 modes={}
 for f in families:
  modes[f]={}
  for p in POLICIES:
   a=[r for r in rows if r["family"]==f and r["policy"]==p]
   rec=Counter()
   for r in a:rec.update(dict(r["recovery_action_counts"]))
   modes[f][p]={
    "optical_channels":sorted({v for r in a for v in r["optical_channels_used"]}),
    "acoustic_techniques":sorted({v for r in a for v in r["acoustic_techniques_used"]}),
    "fusion_modes":sorted({v for r in a for v in r["fusion_modes_used"]}),
    "mission_actions":sorted({v for r in a for v in r["mission_actions_used"]}),
    "recovery_action_counts":dict(sorted(rec.items()))}
 result={"schema":"study3_infrastructure_confirmation_analysis_v1","root":ROOT,
  "executions":len(rows),"primary_families":PRIMARY,"family":family,"modes":modes,
  "aggregate_primary":{p:means(rows,PRIMARY,p) for p in POLICIES},
  "aggregate_all":{p:means(rows,families,p) for p in POLICIES},
  "contrasts":{
   "reactive_minus_fixed_primary":paired(rows,PRIMARY,"reactive","fixed"),
   "reactive_minus_robust_primary":paired(rows,PRIMARY,"reactive","robust_fusion"),
   "predictive_minus_reactive_primary":paired(rows,PRIMARY,"predictive","reactive")},
  "fixed_safety_current_all":{"events":sum(r["safety_violation"] for r in rows if r["policy"]=="fixed"),"runs":170},
  "fixed_safety_preserved_selection":{"events":6,"runs":170,"root":31_800_000}}
 OUT.write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=True)+"\n")
 print(OUT)
if __name__=="__main__":main()
