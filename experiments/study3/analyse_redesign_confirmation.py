#!/usr/bin/env python3
"""Deterministic paired analysis for redesigned Study 3 DEVELOPMENT evidence."""
from __future__ import annotations

import argparse,hashlib,json,math
from collections import defaultdict
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
PRIMARY=("S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
         "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
         "S3_COMPOUND_DVL_ACOUSTIC")
FAMILIES=PRIMARY+("S3_NOMINAL","S3_SUDDEN","S3_NO_RECOVERY")
POLICIES=("fixed","robust_fusion","reactive","predictive")

def digest(x):
    return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=True).encode()).hexdigest()

def load(packets):
    rows=[]
    for path in sorted(packets.glob("*.json")):
        packet=json.loads(path.read_text());stored=packet.pop("packet_sha256",None)
        if digest(packet)!=stored:raise RuntimeError(f"checksum mismatch: {path}")
        row=packet["result"];row["root"]=packet["identity"]["root"]
        rows.append(row)
    expected={(f,i,p) for f in FAMILIES for i in range(15) for p in POLICIES}
    actual={(r["family"],r["index"],r["policy"]) for r in rows}
    if len(rows)!=600 or actual!=expected:raise RuntimeError(f"incomplete confirmation: {len(rows)}/600")
    return rows

def means(rows):
    keys=("completed","survey_coverage_fraction","rmse_transition_m","peak_error_m",
          "unaided_time_s","longest_unaided_gap_s","capability_preserved",
          "unnecessary_interventions","preemptive_actions","mission_duration_s",
          "safety_violation","mean_policy_runtime_ms","optical_forecast_episodes",
          "true_optical_forecast_episodes","false_optical_forecast_episodes")
    out={k:float(np.mean([r[k] for r in rows])) for k in keys}
    finite=[r["recovery_time_s"] for r in rows if math.isfinite(r["recovery_time_s"])]
    out["recovery_time_s_finite_mean"]=float(np.mean(finite)) if finite else None
    out["recovery_observed_fraction"]=len(finite)/len(rows)
    leads=[r["optical_prediction_lead_s"] for r in rows if math.isfinite(r.get("optical_prediction_lead_s",math.nan))]
    out["optical_prediction_lead_s_finite_mean"]=float(np.mean(leads)) if leads else None
    out["optical_prediction_lead_observed_fraction"]=len(leads)/len(rows)
    return out

def paired(rows,candidate,reference,metric,direction):
    table={(r["family"],r["index"],r["policy"]):r for r in rows}
    return {f:np.array([direction*(float(table[f,i,candidate][metric])-float(table[f,i,reference][metric]))
                        for i in range(15)]) for f in PRIMARY}

def stratified_ci(by_family,seed=31730000,n_boot=10000):
    rng=np.random.default_rng(seed);families=list(by_family)
    observed=float(np.mean([np.mean(by_family[f]) for f in families]));boot=np.empty(n_boot)
    for b in range(n_boot):
        sampled=rng.choice(families,len(families),replace=True)
        boot[b]=np.mean([rng.choice(by_family[f],len(by_family[f]),replace=True).mean() for f in sampled])
    return observed,[float(x) for x in np.quantile(boot,[.025,.975])]

def sign_p(by_family,seed,n=20000):
    values=np.concatenate(list(by_family.values()));obs=abs(values.mean())
    rng=np.random.default_rng(seed);extreme=0
    for _ in range(n):
        extreme+=abs(np.mean(values*rng.choice((-1.,1.),len(values))))>=obs-1e-15
    return (extreme+1)/(n+1)

def holm(raw):
    ordered=sorted(raw,key=raw.get);adjusted={};running=0.
    for rank,key in enumerate(ordered):
        running=max(running,min(1.,raw[key]*(len(raw)-rank)));adjusted[key]=running
    return adjusted

def causal(rows):
    traces={(r["family"],r["policy"]):r.get("causal_trace") for r in rows if r["index"]==0}
    evidence=[]
    for family in PRIMARY:
        pred=traces[family,"predictive"] or [];react=traces[family,"reactive"] or []
        action_index=next((i for i,x in enumerate(pred) if x[6]["preemptive"]),None)
        if action_index is None:
            evidence.append({"family":family,"preemptive_action":False});continue
        before=pred[max(0,action_index-1)];after=pred[min(len(pred)-1,action_index+2)]
        evidence.append({"family":family,"preemptive_action":True,
          "forecast_at_action":list(pred[action_index][5]),"action_time_s":pred[action_index][0],
          "action":pred[action_index][6],"altitude_before_m":before[7],"altitude_after_m":after[7],
          "quality_before":before[1],"quality_after":after[1],
          "error_after_predictive_m":after[8],"error_after_reactive_m":react[min(len(react)-1,action_index+2)][8]})
    return evidence

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--stage",default="confirmation_v2",
        choices=("confirmation","confirmation_v2","confirmation_v3","predictor_confirmation"));args=parser.parse_args()
    packets=HERE/"redesign_results"/args.stage
    out_path=HERE/"redesign_results"/(args.stage+"_analysis.json")
    rows=load(packets);root=rows[0]["root"];family={}
    for f in FAMILIES:
        family[f]={p:means([r for r in rows if r["family"]==f and r["policy"]==p]) for p in POLICIES}
    aggregate={p:means([r for r in rows if r["family"] in PRIMARY and r["policy"]==p]) for p in POLICIES}
    comparisons={};raw={}
    metrics={"completed":1,"survey_coverage_fraction":1,"rmse_transition_m":-1,
             "peak_error_m":-1,"unaided_time_s":-1,"longest_unaided_gap_s":-1,
             "capability_preserved":1,"unnecessary_interventions":-1,
             "mission_duration_s":-1,"safety_violation":-1}
    for candidate,reference in (("robust_fusion","fixed"),("reactive","robust_fusion"),
                                ("predictive","reactive"),("predictive","fixed")):
        label=f"{candidate}_vs_{reference}";comparisons[label]={}
        for j,(metric,direction) in enumerate(metrics.items()):
            by=paired(rows,candidate,reference,metric,direction);effect,ci=stratified_ci(by,31730000+j)
            comparisons[label][metric]={"benefit_positive_effect":effect,"ci95":ci,
                "family_effects":{f:float(v.mean()) for f,v in by.items()}}
            if label=="predictive_vs_reactive" and metric in {"completed","rmse_transition_m","unaided_time_s","safety_violation"}:
                raw[metric]=sign_p(by,31730100+j)
    adjusted=holm(raw)
    for metric,p in raw.items():
        comparisons["predictive_vs_reactive"][metric].update({"raw_p":p,"holm_p":adjusted[metric]})
    unaided=paired(rows,"predictive","reactive","unaided_time_s",-1)
    sd=float(np.std(np.concatenate(list(unaided.values())),ddof=1))
    proposed=max(30,int(math.ceil(((1.96+.84)*sd/2.)**2)))
    result={"schema":"study3_redesign_development_confirmation_analysis_v1",
      "root":root,"executions":len(rows),"family_metrics":family,
      "primary_family_equal_metrics":aggregate,"comparisons":comparisons,
      "causal_trace_evidence":causal(rows),"paired_unaided_effect_sd_s":sd,
      "proposed_heldout_members_per_primary_family":proposed,
      "packet_set_digest":digest([{k:v for k,v in r.items() if k!="causal_trace"} for r in rows])}
    tmp=out_path.with_suffix(".tmp");tmp.write_text(json.dumps(result,sort_keys=True,indent=2,allow_nan=True)+"\n");tmp.replace(out_path)
    print(json.dumps({"executions":600,"aggregate":aggregate,"comparisons":comparisons,
      "causal":result["causal_trace_evidence"],"paired_unaided_sd":sd,
      "proposed_n":proposed},indent=2,allow_nan=True))

if __name__=="__main__":main()
