#!/usr/bin/env python3
"""Literature-grounded Study 3 redesign DEVELOPMENT runner (never held-out)."""
from __future__ import annotations
import argparse,hashlib,itertools,json,os,sys,time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

HERE=Path(__file__).resolve().parent
# ``HERE`` is <repository>/experiments/study3, so the importable package is
# below HERE.parents[1].  The former parents[2] only worked when callers
# supplied an undocumented PYTHONPATH and broke the documented direct launch.
PKG=HERE.parents[1]/"src/uuv_mode_aware_navigation"
sys.path.insert(0,str(PKG))
from uuv_mode_aware_navigation.study3 import (FAMILIES,PRIMARY,FixedConfiguration,
    PolicyKind,deployed_acoustic_services,deployment_informed_fixed_configuration,run_one)

ROOTS={"calibration":31_700_000,"fixed":31_710_000,"adaptive":31_720_000,
       "confirmation":31_730_000,"adaptive_v2":31_742_000,
       "confirmation_v2":31_744_000,"confirmation_v3":31_745_000,
       "predictor_confirmation":31_770_000,
       "infrastructure_fixed":31_800_000,
       "infrastructure_smoke":31_819_000,
       "infrastructure_confirmation":31_820_000,
       "recovery_focused":31_830_000,
       "recovery_confirmation":31_840_000,
       "mode_comparison_v3":31_850_000,
       "mode_comparison_v4":31_870_000,
       "discovery_fairness_v1":31_880_000}
OUT=HERE/"redesign_results"
FIXED_LOCK=OUT/"fixed_baseline_lock.json"
def digest(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=True).encode()).hexdigest()
def execute(task):
 stage,root,family,index,policy,cid,config=task
 if not 31_000_000<=root<32_000_000:raise RuntimeError("redesign DEVELOPMENT root outside 31 million range")
 identity={"stage":stage,"root":root,"family":family,"index":index,"policy":policy,"configuration_id":cid,"configuration":config}
 key=digest(identity)[:24];path=OUT/stage/(key+".json")
 if path.exists():
  p=json.loads(path.read_text());stored=p.pop("packet_sha256",None)
  if p["identity"]!=identity or digest(p)!=stored:raise RuntimeError(f"bad resume packet {path}")
  return p["result"]
 capture_trace=((stage.startswith("confirmation") or stage in {"predictor_confirmation",
                "infrastructure_confirmation","mode_comparison_v4","discovery_fairness_v1"}) and
                (index==0 or stage=="infrastructure_confirmation"))
 version=3 if (stage.startswith("infrastructure_") or stage.startswith("recovery_")
               or stage in {"mode_comparison_v4","discovery_fairness_v1"}) else 2
 outcome=run_one(root,family,index,PolicyKind(policy),FixedConfiguration(**config),
                 horizon_s=180.,dt_s=2.,image_period_s=4.,
                 keep_trace=capture_trace,redesign_version=version)
 if capture_trace:
  run_result,trace=outcome;result=asdict(run_result);result["causal_trace"]=trace
 else:
  result=asdict(outcome)
 result["configuration_id"]=cid
 packet={"schema":"study3_redesign_development_packet_v1","identity":identity,"result":result};packet["packet_sha256"]=digest(packet)
 path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix(f".{os.getpid()}.tmp")
 tmp.write_text(json.dumps(packet,sort_keys=True,indent=2,allow_nan=True)+"\n");os.replace(tmp,path);return result
def run(tasks,workers):
 t=time.perf_counter()
 with ProcessPoolExecutor(max_workers=workers) as pool:r=list(pool.map(execute,tasks,chunksize=1))
 return r,time.perf_counter()-t
def configs():
 vals=itertools.product(("camera_coaxial","camera_offaxis","lidar"),(1.,3.,5.),(.25,.5,.75),
                        ("single_beacon","lbl","usbl"),("gate","weight"))
 return {f"fixed_{i:03d}":asdict(FixedConfiguration(*v)) for i,v in enumerate(vals)}
def infrastructure_comparison_configs():
 lock=json.loads((OUT/"infrastructure_fixed_baseline_lock.json").read_text())
 if lock["configuration_id"]!="fixed_155":raise RuntimeError("infrastructure FIXED lock is not fixed_155")
 fixed=lock["configuration"]
 selected=json.loads((OUT/"adaptive_v2_summary.json").read_text())
 shared_keys=("optical_channel","altitude_m","speed_mps","acoustic_technique","fusion_mode")
 reactive={**selected["reactive_configuration"],**{k:fixed[k] for k in shared_keys}}
 predictive={**selected["predictive_configuration"],**{k:fixed[k] for k in shared_keys}}
 expected={k:fixed[k] for k in shared_keys}
 for name,configuration in (("FIXED",fixed),("REACTIVE",reactive),("PREDICTIVE",predictive)):
  actual={k:configuration[k] for k in shared_keys}
  if actual!=expected:raise RuntimeError(f"{name} does not start from locked fixed_155: {actual}")
 return fixed,reactive,predictive

def infrastructure_tasks(stage,root,indices,families=FAMILIES):
 fixed,reactive,predictive=infrastructure_comparison_configs();tasks=[]
 for f in families:
  for i in indices:
   tasks += [(stage,root,f,i,"fixed","fixed_155",fixed),
             (stage,root,f,i,"robust_fusion","robust_fusion_fixed_155",fixed),
             (stage,root,f,i,"reactive","reactive_shared_fixed_155",reactive),
             (stage,root,f,i,"predictive","predictive_shared_fixed_155",predictive)]
 return tasks,(fixed,reactive,predictive)

def discovery_fairness_tasks(root,indices,families=FAMILIES):
 fixed,reactive,_=infrastructure_comparison_configs();tasks=[];deployment_configs={}
 for family in families:
  # The catalogue is the service declared deployed at launch. It is legitimate
  # pre-mission information; no later schedule or physical-state value enters.
  catalogue=deployed_acoustic_services(family,0.0,180.0)
  deployment=asdict(deployment_informed_fixed_configuration(
      FixedConfiguration(**fixed),catalogue))
  deployment_configs[family]=deployment
  for i in indices:
   tasks += [("discovery_fairness_v1",root,family,i,"fixed","fixed_155",fixed),
             ("discovery_fairness_v1",root,family,i,"deployment_fixed",
              "deployment_informed_fixed_155",deployment),
             ("discovery_fairness_v1",root,family,i,"reactive",
              "reactive_shared_fixed_155",reactive)]
 return tasks,(fixed,reactive,deployment_configs)
def rank(rows):
 by={}
 for r in rows:by.setdefault(r["configuration_id"],[]).append(r)
 return [cid for _,cid in sorted(((sum(x["safety_violation"] for x in a)/len(a),
   -sum(x["completed"] for x in a)/len(a),sum(x["unaided_time_s"] for x in a)/len(a),
   sum(x["rmse_transition_m"] for x in a)/len(a),sum(x["mission_duration_s"] for x in a)/len(a),cid),cid) for cid,a in by.items())]
def summary(stage,root,rows,elapsed,extra=None):
 d={"schema":"study3_redesign_summary_v1","stage":stage,"root":root,"executions":len(rows),
    "wall_runtime_s":elapsed,"result_digest":digest(rows),"created_utc":time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime())}
 if extra:d.update(extra)
 p=OUT/f"{stage}_summary.json";p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(".tmp");tmp.write_text(json.dumps(d,sort_keys=True,indent=2)+"\n");os.replace(tmp,p);print(json.dumps(d,indent=2,sort_keys=True))
def main():
 ap=argparse.ArgumentParser();ap.add_argument("stage",choices=ROOTS);ap.add_argument("--workers",type=int,default=4);a=ap.parse_args();root=ROOTS[a.stage]
 default=asdict(FixedConfiguration())
 if a.stage=="calibration":
  tasks=[("calibration",root,f,i,p.value,"default",default) for f in FAMILIES for i in range(8) for p in PolicyKind]
  rows,e=run(tasks,a.workers);summary(a.stage,root,rows,e)
 elif a.stage in {"fixed","infrastructure_fixed"}:
  c=configs();rows=[];elapsed=0.
  prefix=("infrastructure_fixed" if a.stage=="infrastructure_fixed" else "fixed")
  t=[(prefix+"_s1",root,f,0,"fixed",cid,v) for cid,v in c.items() for f in FAMILIES];r,e=run(t,a.workers);rows+=r;elapsed+=e;top18=rank(r)[:18]
  t=[(prefix+"_s2",root,f,i,"fixed",cid,c[cid]) for cid in top18 for i in range(1,5) for f in FAMILIES];r2,e=run(t,a.workers);rows+=r2;elapsed+=e
  # Successive halving ranks only candidates advanced from the previous
  # stage. Mixing one-seed eliminated candidates with five-seed survivors can
  # promote a candidate that never qualified for the next stage.
  top4=rank([x for x in rows if x["configuration_id"] in top18])[:4]
  t=[(prefix+"_s3",root,f,i,"fixed",cid,c[cid]) for cid in top4 for i in range(5,17) for f in FAMILIES];r3,e=run(t,a.workers);rows+=r3;elapsed+=e;baseline=rank(rows)[0]
  baseline=rank([x for x in rows if x["configuration_id"] in top4])[0]
  summary(a.stage,root,rows,elapsed,{"top18":top18,"top4":top4,
    "strongest_fixed_baseline_id":baseline,"strongest_fixed_baseline":c[baseline]})
 elif a.stage in {"adaptive","adaptive_v2"}:
  baseline=json.loads(FIXED_LOCK.read_text())["configuration"]
  # Each candidate gives P5 at least two observations per recovery episode
  # while respecting the survey-derived low-altitude duty bound.
  control={
   "two_frame_48s":{**baseline,"recovery_dwell_s":8.,"recovery_cooldown_s":48.,"minimum_action_hold_s":8.,"recovery_altitude_floor_m":2.},
   "three_frame_64s":{**baseline,"recovery_dwell_s":12.,"recovery_cooldown_s":64.,"minimum_action_hold_s":8.,"recovery_altitude_floor_m":2.},
   "two_frame_stable":{**baseline,"recovery_dwell_s":8.,"recovery_cooldown_s":56.,"minimum_action_hold_s":12.,"recovery_altitude_floor_m":2.}}
  tasks=[(a.stage+"_control",root,f,i,p.value,cid,v)
         for cid,v in control.items() for p in (PolicyKind.REACTIVE,PolicyKind.PREDICTIVE)
         for f in PRIMARY for i in range(6)]
  control_rows,e1=run(tasks,a.workers);selected_control=rank(control_rows)[0]
  shared=control[selected_control]
  forecasts={
   "short_confirmed":{**shared,"prediction_horizon_s":8.,"trend_confirmation_frames":4,"minimum_cumulative_quality_decline":.18},
   "balanced":{**shared,"prediction_horizon_s":12.,"trend_confirmation_frames":3,"minimum_cumulative_quality_decline":.18},
   "long_confirmed":{**shared,"prediction_horizon_s":18.,"trend_confirmation_frames":4,"minimum_cumulative_quality_decline":.20},
   "early_strict":{**shared,"prediction_horizon_s":15.,"trend_confirmation_frames":3,"minimum_cumulative_quality_decline":.22}}
  tasks=[(a.stage+"_forecast",root+1_000,f,i,"predictive",cid,v)
         for cid,v in forecasts.items() for f in PRIMARY for i in range(6)]
  forecast_rows,e2=run(tasks,a.workers);selected_forecast=rank(forecast_rows)[0]
  rows=control_rows+forecast_rows
  summary(a.stage,root,rows,e1+e2,{"selected_shared_controller":selected_control,
    "reactive_configuration":shared,"selected_predictive_forecast":selected_forecast,
    "predictive_configuration":forecasts[selected_forecast]})
 elif a.stage in {"infrastructure_smoke","infrastructure_confirmation","mode_comparison_v3","mode_comparison_v4"}:
  indices=(range(1) if a.stage=="infrastructure_smoke" else range(17))
  families=((FAMILIES[0],) if a.stage=="infrastructure_smoke" else FAMILIES)
  tasks,(fixed,reactive,predictive)=infrastructure_tasks(a.stage,root,indices,families)
  rows,e=run(tasks,a.workers)
  summary(a.stage,root,rows,e,{"fixed_configuration":fixed,
      "reactive_configuration":reactive,"predictive_configuration":predictive,
      "primary_comparison":"reactive_minus_fixed paired by family and index",
      "scientific_evidence":a.stage in {"infrastructure_confirmation","mode_comparison_v3","mode_comparison_v4"}})
 elif a.stage=="discovery_fairness_v1":
  tasks,(fixed,reactive,deployment_configs)=discovery_fairness_tasks(root,range(17))
  rows,e=run(tasks,a.workers)
  summary(a.stage,root,rows,e,{"fixed_configuration":fixed,
      "reactive_configuration":reactive,
      "deployment_informed_configurations":deployment_configs,
      "comparisons":["reactive_minus_fixed","reactive_minus_deployment_fixed"],
      "scientific_evidence":"bounded_development_fairness_correction"})
 elif a.stage=="recovery_focused":
  families=("S3_RECOVERY","S3_OPTICAL_GRADUAL","S3_NO_RECOVERY",
            "S3_ACOUSTIC_GEOMETRY_ASYNC","S3_COMPOUND_OPTICAL_ACOUSTIC")
  fixed,reactive,_=infrastructure_comparison_configs();tasks=[]
  for f in families:
   for i in range(15):
    tasks += [(a.stage,root,f,i,"fixed","fixed_155",fixed),
              (a.stage,root,f,i,"reactive","reactive_ab_correction",reactive)]
  rows,e=run(tasks,a.workers);summary(a.stage,root,rows,e,{"families":families,
    "fixed_configuration":fixed,"reactive_configuration":reactive,
    "scientific_evidence":"mechanism_specific_development_gate"})
 elif a.stage=="recovery_confirmation":
  tasks,(fixed,reactive,predictive)=infrastructure_tasks(a.stage,root,range(17))
  rows,e=run(tasks,a.workers);summary(a.stage,root,rows,e,{"fixed_configuration":fixed,
    "reactive_configuration":reactive,"predictive_configuration":predictive,
    "primary_comparison":"reactive_minus_fixed paired by family and index",
    "scientific_evidence":True})
 else:
  fixed=json.loads(FIXED_LOCK.read_text())["configuration"]
  adaptive_name=("adaptive_v2_summary.json" if a.stage in {"confirmation_v2","confirmation_v3","predictor_confirmation"}
                 else "adaptive_summary.json")
  adaptive_summary=json.loads((OUT/adaptive_name).read_text())
  reactive=adaptive_summary["reactive_configuration"];predictive=adaptive_summary["predictive_configuration"]
  tasks=[]
  for f in FAMILIES:
   for i in range(15):
    tasks += [(a.stage,root,f,i,"fixed","fixed",fixed),
              (a.stage,root,f,i,"robust_fusion","robust",fixed),
              (a.stage,root,f,i,"reactive","reactive",reactive),
              (a.stage,root,f,i,"predictive","predictive",predictive)]
  rows,e=run(tasks,a.workers);summary(a.stage,root,rows,e)
if __name__=="__main__":main()
