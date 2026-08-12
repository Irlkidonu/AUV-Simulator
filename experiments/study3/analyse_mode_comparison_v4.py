#!/usr/bin/env python3
"""Immutable paired analysis for final corrected Study-3 DEVELOPMENT V4."""
import hashlib,json,math
from collections import Counter,defaultdict
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent; DATA=HERE/"redesign_results/mode_comparison_v4"
OUT=HERE/"redesign_results/mode_comparison_v4_analysis.json"
FAMILIES=("S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
 "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
 "S3_COMPOUND_DVL_ACOUSTIC","S3_NOMINAL","S3_SUDDEN","S3_NO_RECOVERY")
PRIMARY=FAMILIES[:7]; POLICIES=("fixed","robust_fusion","reactive","predictive")
METRICS=("completed","safety_violation","surfaced_for_gps","gps_reacquired",
 "overall_rmse_m","rmse_transition_m","peak_error_m","unaided_time_s",
 "longest_unaided_gap_s","survey_coverage_fraction","mission_duration_s",
 "physical_interventions","unnecessary_interventions","mode_switches")
LOWER={"safety_violation","overall_rmse_m","rmse_transition_m","peak_error_m",
 "unaided_time_s","longest_unaided_gap_s","physical_interventions",
 "unnecessary_interventions","mode_switches"}

def digest(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=True).encode()).hexdigest()
rows=[]
for path in sorted(DATA.glob("*.json")):
 p=json.loads(path.read_text());stored=p.pop("packet_sha256")
 if digest(p)!=stored:raise RuntimeError(f"checksum failure {path}")
 ident=p["identity"]
 if ident["root"]!=31_870_000 or ident["stage"]!="mode_comparison_v4":raise RuntimeError(f"identity failure {path}")
 r=p["result"]|{"family":ident["family"],"index":ident["index"],"policy":ident["policy"]};rows.append(r)
keys={(r["family"],r["index"],r["policy"]) for r in rows}
expected={(f,i,p) for f in FAMILIES for i in range(17) for p in POLICIES}
if keys!=expected or len(rows)!=680:raise RuntimeError("incomplete or duplicate campaign")

def mean(rows,key):return float(np.mean([float(r[key]) for r in rows]))
def aggregate(families):
 return {p:{m:mean([r for r in rows if r["policy"]==p and r["family"] in families],m) for m in METRICS} for p in POLICIES}
def paired(a,b,families,metric):
 return np.array([next(r for r in rows if r["family"]==f and r["index"]==i and r["policy"]==a)[metric]-
                  next(r for r in rows if r["family"]==f and r["index"]==i and r["policy"]==b)[metric]
                  for f in families for i in range(17)],float)
def contrast(a,b,families):
 rng=np.random.default_rng(31_870_000);out={}
 for metric in METRICS:
  by=[paired(a,b,(f,),metric) for f in families];d=np.concatenate(by);boots=[]
  for _ in range(10000):boots.append(np.mean(np.concatenate([x[rng.integers(0,17,17)] for x in by])))
  favorable=-d if metric in LOWER else d
  out[metric]={"difference":float(np.mean(d)),"ci95":[float(x) for x in np.quantile(boots,[.025,.975])],
   "standardized_paired_effect":float(np.mean(d)/(np.std(d,ddof=1) or math.inf)),
   "wins":int(np.sum(favorable>1e-12)),"ties":int(np.sum(abs(d)<=1e-12)),"losses":int(np.sum(favorable< -1e-12))}
 return out
def family_table():
 result={}
 for f in FAMILIES:
  result[f]={}
  for p in POLICIES:
   rr=[r for r in rows if r["family"]==f and r["policy"]==p]
   result[f][p]={m:mean(rr,m) for m in METRICS}
 return result
def telemetry(policy,families=FAMILIES):
 rr=[r for r in rows if r["policy"]==policy and r["family"] in families];mode=Counter();vel=Counter();absolute=Counter();directed=Counter();terminal=relative=fallback=0
 for r in rr:
  t=r["mode_telemetry"] or {}
  mode.update(dict(t.get("mode_dwell_s",[])));vel.update(dict(t.get("velocity_source_s",[])));absolute.update(dict(t.get("absolute_source_s",[])));directed.update(dict(t.get("mode_transitions_directed",[])))
  terminal+=t.get("first_terminal_entry") is not None;relative+=t.get("first_relative_entry") is not None;fallback+=t.get("first_fallback_entry") is not None
 total=sum(mode.values()) or 1
 return {"mode_occupancy_fraction":{k:v/total for k,v in sorted(mode.items())},"velocity_source_s":dict(sorted(vel.items())),
  "absolute_source_s":dict(sorted(absolute.items())),"directed_transitions":dict(sorted(directed.items())),
  "runs_with_relative":relative,"runs_with_fallback":fallback,"runs_with_terminal":terminal}

record={"schema":"study3_mode_comparison_v4_analysis_v1","root":31_870_000,"packets_verified":len(rows),
 "bootstrap_resamples":10000,"aggregate_primary":aggregate(PRIMARY),"aggregate_all":aggregate(FAMILIES),
 "families":family_table(),"contrasts":{"reactive_minus_fixed_primary":contrast("reactive","fixed",PRIMARY),
 "reactive_minus_fixed_all":contrast("reactive","fixed",FAMILIES),"robust_minus_fixed_primary":contrast("robust_fusion","fixed",PRIMARY),
 "predictive_minus_reactive_primary":contrast("predictive","reactive",PRIMARY)},
 "family_contrasts_reactive_minus_fixed":{f:contrast("reactive","fixed",(f,)) for f in FAMILIES},
 "telemetry":{"reactive_primary":telemetry("reactive",PRIMARY),"reactive_all":telemetry("reactive")}}
OUT.write_text(json.dumps(record,indent=2,sort_keys=True,allow_nan=True)+"\n")
print(json.dumps({"output":str(OUT),"packets":len(rows),"sha256":hashlib.sha256(OUT.read_bytes()).hexdigest()},indent=2))
