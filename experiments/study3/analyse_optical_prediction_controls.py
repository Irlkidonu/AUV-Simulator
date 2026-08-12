#!/usr/bin/env python3
"""Classify P5 diagnostics and select an evidence forecaster on DEVELOPMENT controls."""
from __future__ import annotations
import argparse,itertools,json,math
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;DATA=HERE/"predictor_development"

def margins(o):
 kp=max(min(o["keypoints_a"],o["keypoints_b"]),1);sigma=o["sigma_m"]
 return np.clip([(o["inliers"]-12)/36,(o["inliers"]/kp-.05)/.75,
  (o["inlier_fraction"]-.5)/.5,(2-o["reprojection_px"])/2 if math.isfinite(o["reprojection_px"]) else 0,
  (.5-o["ambiguity_ratio"])/.5 if math.isfinite(o["ambiguity_ratio"]) else 0,
  (.1-sigma)/.1 if math.isfinite(sigma) else 0],0,1)

def trace(rows,window,horizon,floor,quorum,min_slope):
 hist=[];out=[]
 for row in rows:
  hist.append((row["time_s"],margins(row["observable"])));hist=hist[-window:];warning=False;ttl=math.inf
  if len(hist)>=window and row["observable"]["available"]:
   slopes=[]
   for j in range(6):
    slopes.append(np.median([(hist[b][1][j]-hist[a][1][j])/(hist[b][0]-hist[a][0])
                             for a in range(window) for b in range(a+1,window)]))
   score=float(np.median(hist[-1][1]));slope=float(np.median(slopes))
   ttl=((score-floor)/-slope if slope < -min_slope and score>floor else
        0. if score<=floor else math.inf)
   warning=ttl<=horizon and sum(s < -min_slope for s in slopes)>=quorum
  out.append({"time_s":row["time_s"],"warning":warning,"time_to_loss_s":ttl,
              "available":row["observable"]["available"],"level":row["evaluator"]["degradation_level"]})
 return out

def evaluate(members,cfg):
 by={}
 for member in members:by.setdefault(member["control"],[]).append(trace(member["rows"],*cfg))
 const_member=float(np.mean([any(x["warning"] for x in z) for z in by["constant_texture"]]))
 const_frame=float(np.mean([x["warning"] for z in by["constant_texture"] for x in z]))
 warned=[];leads=[]
 for z in by["true_degradation"]:
  loss=next((x["time_s"] for x in z if not x["available"]),math.inf)
  times=[x["time_s"] for x in z if x["warning"] and x["time_s"]<loss]
  warned.append(bool(times));leads.extend([loss-times[0]] if times else [])
 sudden=float(np.mean([any(x["warning"] and x["time_s"]<84 for x in z) for z in by["sudden_loss"]]))
 recovery=float(np.mean([not any(x["warning"] for x in z[-8:]) for z in by["recovery"]]))
 return {"constant_false_member_rate":const_member,"constant_false_frame_rate":const_frame,
         "true_warning_rate":float(np.mean(warned)),"median_lead_s":float(np.median(leads)) if leads else 0.,
         "sudden_pre_loss_false_rate":sudden,"recovery_clear_rate":recovery}

def classification(members):
 controls={}
 for m in members:controls.setdefault(m["control"],[]).append(m)
 names=("quality","available","inliers","support_density","consensus","reprojection_margin","ambiguity_margin","uncertainty_margin")
 def f(o):
  kp=max(min(o["keypoints_a"],o["keypoints_b"]),1);s=o["sigma_m"]
  return (o["quality"],float(o["available"]),math.log1p(o["inliers"]),o["inliers"]/kp,
   o["inlier_fraction"],max(0,1-o["reprojection_px"]/2) if math.isfinite(o["reprojection_px"]) else 0,
   max(0,1-o["ambiguity_ratio"]/.5) if math.isfinite(o["ambiguity_ratio"]) else 0,
   math.exp(-s/.1) if math.isfinite(s) else 0)
 constant=np.array([f(r["observable"]) for m in controls["constant_texture"] for r in m["rows"]])
 degraded=[r for m in controls["true_degradation"] for r in m["rows"]]
 values=np.array([f(r["observable"]) for r in degraded]);level=np.array([r["evaluator"]["degradation_level"] for r in degraded])
 out={}
 for j,name in enumerate(names):
  corr=float(np.corrcoef(values[:,j],level)[0,1]) if np.std(values[:,j]) else 0.
  out[name]={"constant_mean":float(np.mean(constant[:,j])),"constant_sd":float(np.std(constant[:,j])),
             "degradation_correlation":corr}
 return out

def main():
 p=argparse.ArgumentParser();p.add_argument("stage",choices=("diagnosis","tuning","confirmation"));a=p.parse_args()
 data=json.loads((DATA/(a.stage+"_controls.json")).read_text());members=data["members"]
 configs=list(itertools.product((4,5,6),(8.,12.,16.),(.20,.30,.40),(2,3,4),(.001,.003,.005)))
 results=[{"configuration":{"window":c[0],"horizon_s":c[1],"score_floor":c[2],"decline_quorum":c[3],"minimum_slope":c[4]},
           "metrics":evaluate(members,c)} for c in configs]
 results.sort(key=lambda x:(x["metrics"]["constant_false_member_rate"],
                            x["metrics"]["sudden_pre_loss_false_rate"],
                            -x["metrics"]["true_warning_rate"],-x["metrics"]["median_lead_s"]))
 out={"schema":"study3_optical_prediction_control_analysis_v1","stage":a.stage,
      "feature_classification":classification(members),"selected":results[0],"top10":results[:10]}
 path=DATA/(a.stage+"_analysis.json");path.write_text(json.dumps(out,sort_keys=True,indent=2)+"\n")
 print(json.dumps(out,indent=2))
if __name__=="__main__":main()
