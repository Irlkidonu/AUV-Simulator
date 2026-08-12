#!/usr/bin/env python3
"""Deterministic family-stratified Study 3 development statistics."""
from __future__ import annotations
import argparse,glob,json,math,statistics
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
PRIMARY=("S3_OPTICAL_GRADUAL","S3_DVL_GRADUAL","S3_ACOUSTIC_GEOMETRY_ASYNC",
 "S3_INFRASTRUCTURE_WARNING","S3_RECOVERY","S3_COMPOUND_OPTICAL_ACOUSTIC",
 "S3_COMPOUND_DVL_ACOUSTIC")
METRICS=("completed","safety_violation","rmse_transition_m","unaided_time_s")


def main():
 p=argparse.ArgumentParser();p.add_argument("--iteration",type=int,default=6);a=p.parse_args()
 stage=f"confirmation_v{a.iteration}" if a.iteration>1 else "confirmation"
 pairs={}
 for path in glob.glob(str(HERE/"development_results"/stage/"*.json")):
  packet=json.loads(Path(path).read_text());r=packet["result"]
  pairs.setdefault((r["family"],r["index"]),{})[r["policy"]]=r
 if len(pairs)!=150 or any(set(v)!={"fixed","reactive","predictive"} for v in pairs.values()):
  raise SystemExit("incomplete or unpaired confirmation corpus")
 rng=np.random.default_rng(31_499_006);analysis={"stage":stage,"pairs":len(pairs),"bootstrap_resamples":10000,"metrics":{},"families":{}}
 for family in tuple(PRIMARY)+("S3_NOMINAL","S3_SUDDEN","S3_NO_RECOVERY"):
  rows=[v for (f,_),v in sorted(pairs.items()) if f==family]
  analysis["families"][family]={}
  for metric in METRICS+("unnecessary_interventions",):
   d=np.array([float(x["predictive"][metric])-float(x["reactive"][metric]) for x in rows])
   analysis["families"][family][metric]={"mean_difference":float(d.mean()),
     "median_difference":float(np.median(d)),"paired_sd":float(d.std(ddof=1))}
 for metric in METRICS:
  arrays=[np.array([float(pairs[(f,i)]["predictive"][metric])-float(pairs[(f,i)]["reactive"][metric]) for i in range(15)]) for f in PRIMARY]
  observed=float(np.mean([x.mean() for x in arrays]));boots=np.empty(10000)
  for b in range(len(boots)):boots[b]=np.mean([x[rng.integers(0,len(x),len(x))].mean() for x in arrays])
  pooled=np.concatenate(arrays);sd=float(pooled.std(ddof=1));effect=observed/sd if sd>0 else 0.
  null=np.empty(10000)
  for b in range(len(null)):
   null[b]=float(np.mean([np.mean(x*rng.choice((-1.,1.),len(x))) for x in arrays]))
  favorable_tail=(np.sum(null>=observed) if metric=="completed" else np.sum(null<=observed))
  p_one_sided=float((1+favorable_tail)/(1+len(null)))
  lower_is_better=metric!="completed"
  wins=int(np.sum(pooled<0)) if lower_is_better else int(np.sum(pooled>0))
  losses=int(np.sum(pooled>0)) if lower_is_better else int(np.sum(pooled<0))
  analysis["metrics"][metric]={"family_equal_mean_difference":observed,
    "bootstrap_95_ci":[float(x) for x in np.quantile(boots,[.025,.975])],
    "standardized_paired_effect":effect,"one_sided_permutation_p":p_one_sided,"wins":wins,
    "ties":int(np.sum(pooled==0)),"losses":losses}
 gap=np.concatenate([np.array([float(pairs[(f,i)]["predictive"]["unaided_time_s"])-float(pairs[(f,i)]["reactive"]["unaided_time_s"]) for i in range(15)]) for f in PRIMARY])
 sd=float(gap.std(ddof=1));required=math.ceil((1.96+0.84)**2*sd**2/2.0**2) if sd else 1
 nominal=analysis["families"]["S3_NOMINAL"]
 analysis["power"]={"observed_paired_sd_unaided_s":sd,"seeds_for_2s_effect":required,
   "proposed_primary_seeds_per_family":max(30,required),"proposed_control_seeds_per_family":20}
 analysis["nominal_margin"]={"unnecessary_intervention_difference":nominal["unnecessary_interventions"]["mean_difference"],
   "margin":1.0,"passes":nominal["unnecessary_interventions"]["mean_difference"]<=1.0}
 ordered=sorted(METRICS,key=lambda m:analysis["metrics"][m]["one_sided_permutation_p"])
 running=0.0
 for rank,metric in enumerate(ordered):
  adjusted=min(1.0,(len(METRICS)-rank)*analysis["metrics"][metric]["one_sided_permutation_p"])
  running=max(running,adjusted);analysis["metrics"][metric]["holm_adjusted_p"]=running
 superiority=any(analysis["metrics"][m]["holm_adjusted_p"]<.05 for m in METRICS)
 analysis["predictive_mechanism_supported_in_development"]=bool(
   superiority and analysis["nominal_margin"]["passes"])
 out=HERE/"development_results"/f"{stage}_analysis.json";out.write_text(json.dumps(analysis,sort_keys=True,indent=2)+"\n")
 print(json.dumps(analysis,sort_keys=True,indent=2))
if __name__=="__main__":main()
