#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json,math,time
from pathlib import Path
import numpy as np
from uuv_mode_aware_navigation.recovery import ActiveRecoveryPlanner,RecoveryAction,RecoveryState

IDENTIFIER="platform_v2_active_recovery_v1";ROOT=22_240_000;POLICIES=("fixed","reactive","predictive")
def seed(label,i):return int.from_bytes(hashlib.sha256(f"{ROOT}:{label}:{i}".encode()).digest()[:8],"little")
def attenuation(x):return .12+.40/(1+math.exp(-(x-8)/1.2))
def simulate(policy,index):
 rng=np.random.default_rng(seed("paired",index));planner=ActiveRecoveryPlanner();dt=.1;x=y=0.;est_x=est_y=0.;alt=3.;cov=.04;bias=float(rng.normal(.018,.004));prev_q=None;unavailable=0.;max_error=0.;max_cov=cov;transitions=0;safety=0;first_loss=None;first_action=None;recovered=None
 for step in range(600):
  t=step*dt;c=attenuation(x);truth_q=math.exp(-2*c*alt);q=float(np.clip(truth_q+rng.normal(0,.008),0,1));trend=0 if prev_q is None else (q-prev_q)/dt;prev_q=q;available=q>=.25
  if not available:
   unavailable+=dt
   if first_loss is None:first_loss=t
  elif first_loss is not None and recovered is None:recovered=t
  state=RecoveryState(q,trend if policy=="predictive" else 0,alt,.5,0,1,True,1,cov,0)
  action=RecoveryAction.CONTINUE if policy=="fixed" else planner.decide(state).action
  if action is RecoveryAction.LOWER_ALTITUDE:
   if first_action is None:first_action=t
   old=alt;alt=max(1.,alt-.25*dt);transitions+=int(old>1 and alt==1)
  if alt<.95:safety+=1
  # Closed-loop guidance uses the estimated cross-track state.
  cmd_y=float(np.clip(-.8*est_y,-.2,.2));x+=.5*dt;y+=cmd_y*dt
  est_x+=.5*dt+bias*dt;est_y+=cmd_y*dt+rng.normal(0,.002)
  if available:
   est_x=x+rng.normal(0,.1);est_y=y+rng.normal(0,.1);cov=1/(1/max(cov,1e-9)+1/.01)
  else:cov+=.06*dt
  error=math.hypot(est_x-x,est_y-y);max_error=max(max_error,error);max_cov=max(max_cov,cov)
  if x>=20:break
 return {"policy":policy,"seed_index":index,"completed":x>=20 and safety==0,"elapsed_s":t+dt,"aiding_loss_s":unavailable,"recovery_latency_s":None if first_loss is None or recovered is None else recovered-first_loss,"maximum_position_error_m":max_error,"maximum_covariance_m2":max_cov,"altitude_transitions":transitions,"safety_violations":safety,"first_loss_s":first_loss,"first_action_s":first_action,"acted_before_loss":first_action is not None and (first_loss is None or first_action<first_loss)}
def summary(rows):
 return {"completion_rate":sum(r["completed"] for r in rows)/len(rows),"median_aiding_loss_s":float(np.median([r["aiding_loss_s"] for r in rows])),"median_maximum_position_error_m":float(np.median([r["maximum_position_error_m"] for r in rows])),"median_maximum_covariance_m2":float(np.median([r["maximum_covariance_m2"] for r in rows])),"acted_before_loss_rate":sum(r["acted_before_loss"] for r in rows)/len(rows),"altitude_action_rate":sum(r["first_action_s"] is not None for r in rows)/len(rows),"safety_violations":sum(r["safety_violations"] for r in rows)}
def run():
 raw={p:[simulate(p,i) for i in range(30)] for p in POLICIES};s={p:summary(v) for p,v in raw.items()};c={"predictive_completion_at_least_0_90":s["predictive"]["completion_rate"]>=.9,"predictive_zero_safety_violations":s["predictive"]["safety_violations"]==0,"predictive_aiding_loss_below_reactive_and_fixed":s["predictive"]["median_aiding_loss_s"]<s["reactive"]["median_aiding_loss_s"] and s["predictive"]["median_aiding_loss_s"]<s["fixed"]["median_aiding_loss_s"],"predictive_position_error_below_fixed":s["predictive"]["median_maximum_position_error_m"]<s["fixed"]["median_maximum_position_error_m"],"predictive_pre_loss_action_at_least_0_80":s["predictive"]["acted_before_loss_rate"]>=.8,"reactive_action_reached_at_least_0_80":s["reactive"]["altitude_action_rate"]>=.8,"deterministic":raw=={p:[simulate(p,i) for i in range(30)] for p in POLICIES}}
 return {"identifier":IDENTIFIER,"seed_root":ROOT,"status":"DEVELOPMENT PASS" if all(c.values()) else "FAIL","criteria":c,"summaries":s,"raw":raw}
def main():
 p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);a=p.parse_args();started=time.time();r=run();r["wall_time_s"]=time.time()-started;a.output.write_text(json.dumps(r,indent=2,sort_keys=True)+"\n");print(json.dumps({k:v for k,v in r.items() if k!="raw"},indent=2));return 0 if r["status"]=="DEVELOPMENT PASS" else 2
if __name__=="__main__":raise SystemExit(main())
