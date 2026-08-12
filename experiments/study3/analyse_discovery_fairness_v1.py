#!/usr/bin/env python3
"""Registered paired analysis for bounded acoustic-discovery fairness DEVELOPMENT."""
import hashlib,json,math
from collections import Counter
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
DATA=HERE/"redesign_results/discovery_fairness_v1"
OUT=HERE/"redesign_results/discovery_fairness_v1_analysis.json"
FAMILIES=("S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
 "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
 "S3_COMPOUND_DVL_ACOUSTIC","S3_NOMINAL","S3_SUDDEN","S3_NO_RECOVERY")
PRIMARY=FAMILIES[:7]
POLICIES=("fixed","deployment_fixed","reactive")
METRICS=("completed","safety_violation","surfaced_for_gps","gps_reacquired",
 "overall_rmse_m","rmse_transition_m","peak_error_m","unaided_time_s",
 "longest_unaided_gap_s","survey_coverage_fraction","mission_duration_s",
 "physical_interventions","unnecessary_interventions","mode_switches",
 "optical_fixes","acoustic_fixes")
LOWER={"safety_violation","overall_rmse_m","rmse_transition_m","peak_error_m",
 "unaided_time_s","longest_unaided_gap_s","physical_interventions",
 "unnecessary_interventions","mode_switches"}

def digest(x):
 return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=True).encode()).hexdigest()

rows=[]
for path in sorted(DATA.glob("*.json")):
 packet=json.loads(path.read_text());stored=packet.pop("packet_sha256",None)
 if digest(packet)!=stored:raise RuntimeError(f"checksum failure {path}")
 identity=packet["identity"]
 if identity["root"]!=31_880_000 or identity["stage"]!="discovery_fairness_v1":
  raise RuntimeError(f"identity failure {path}")
 rows.append(packet["result"]|{"family":identity["family"],"index":identity["index"],
                              "policy":identity["policy"]})
expected={(f,i,p) for f in FAMILIES for i in range(17) for p in POLICIES}
actual={(r["family"],r["index"],r["policy"]) for r in rows}
if len(rows)!=510 or actual!=expected:raise RuntimeError("incomplete or duplicate campaign")

def mean(rr,key):return float(np.mean([float(r[key]) for r in rr]))
def aggregate(families):
 return {p:{m:mean([r for r in rows if r["policy"]==p and r["family"] in families],m)
            for m in METRICS} for p in POLICIES}
def paired(a,b,families,metric):
 return np.array([next(r for r in rows if r["family"]==f and r["index"]==i and r["policy"]==a)[metric]-
                  next(r for r in rows if r["family"]==f and r["index"]==i and r["policy"]==b)[metric]
                  for f in families for i in range(17)],float)
def contrast(a,b,families,seed):
 rng=np.random.default_rng(seed);out={}
 for metric in METRICS:
  blocks=[paired(a,b,(f,),metric) for f in families];delta=np.concatenate(blocks);boots=[]
  for _ in range(10000):
   boots.append(np.mean(np.concatenate([x[rng.integers(0,17,17)] for x in blocks])))
  favorable=-delta if metric in LOWER else delta
  out[metric]={"difference":float(np.mean(delta)),
   "ci95":[float(x) for x in np.quantile(boots,[.025,.975])],
   "standardized_paired_effect":float(np.mean(delta)/(np.std(delta,ddof=1) or math.inf)),
   "wins":int(np.sum(favorable>1e-12)),"ties":int(np.sum(abs(delta)<=1e-12)),
   "losses":int(np.sum(favorable< -1e-12))}
 return out
def family_table():
 return {f:{p:{m:mean([r for r in rows if r["family"]==f and r["policy"]==p],m)
                   for m in METRICS} for p in POLICIES} for f in FAMILIES}
def telemetry(policy):
 mode=Counter();absolute=Counter();velocity=Counter();directed=Counter()
 for r in rows:
  if r["policy"]!=policy:continue
  t=r.get("mode_telemetry") or {}
  mode.update(dict(t.get("mode_dwell_s",[])));absolute.update(dict(t.get("absolute_source_s",[])))
  velocity.update(dict(t.get("velocity_source_s",[])));directed.update(dict(t.get("mode_transitions_directed",[])))
 total=sum(mode.values()) or 1
 return {"mode_occupancy_fraction":{k:v/total for k,v in sorted(mode.items())},
  "absolute_source_s":dict(sorted(absolute.items())),"velocity_source_s":dict(sorted(velocity.items())),
  "directed_transitions":dict(sorted(directed.items()))}

record={"schema":"study3_discovery_fairness_v1_analysis","root":31_880_000,
 "packets_verified":len(rows),"bootstrap_resamples":10000,
 "aggregate_primary":aggregate(PRIMARY),"aggregate_all":aggregate(FAMILIES),
 "families":family_table(),"contrasts":{
  "reactive_minus_fixed_primary":contrast("reactive","fixed",PRIMARY,31_880_001),
  "reactive_minus_fixed_all":contrast("reactive","fixed",FAMILIES,31_880_002),
  "reactive_minus_deployment_fixed_primary":contrast("reactive","deployment_fixed",PRIMARY,31_880_003),
  "reactive_minus_deployment_fixed_all":contrast("reactive","deployment_fixed",FAMILIES,31_880_004)},
 "telemetry":{"reactive":telemetry("reactive")}}
OUT.write_text(json.dumps(record,indent=2,sort_keys=True,allow_nan=True)+"\n")
print(json.dumps({"output":str(OUT),"packets":len(rows),
 "sha256":hashlib.sha256(OUT.read_bytes()).hexdigest()},indent=2))
