#!/usr/bin/env python3
"""Corrected adaptation analysis for the five-seed generated-environment pilot.

Analysis only: controller, generator, thresholds and pilot realizations are not
modified. Deterministic policy members are replayed to recover observation-side
evidence that was not retained as a pilot packet.
"""
from __future__ import annotations
from dataclasses import asdict
import hashlib,json,math,sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/"src/uuv_mode_aware_navigation"))
from uuv_mode_aware_navigation.study3 import (FixedConfiguration,PolicyKind,
    Study3Policy,deployment_informed_environment_configuration,
    generate_environment,load_environment_config,run_one)

ROOT=31_892_000
SEEDS=tuple(range(31_892_000,31_892_005))
HORIZON_S=180.;DT_S=2.;PERSISTENCE_SAMPLES=3
CONFIG_PATH=HERE/"examples/moderate_severe_variable_environment.json"
OUT=HERE/"redesign_results/generated_environment_pilot_v1_corrected_analysis.json"
BOUNDARY=.35

def physical_acceptable_modes(state):
    """Predeclared dominance objective; no ordering within absolute modes."""
    response=dict(state.service_response_probability);absolute=set()
    acoustic_ok=state.acoustic_noise_db<=65.
    if ("lbl" in state.deployed_acoustic_services and acoustic_ok and
            response.get("lbl",state.acoustic_response_probability)>=.5 and
            state.lbl_geometry_scale>=.35):absolute.add("lbl_aided")
    if ("usbl" in state.deployed_acoustic_services and acoustic_ok and
            response.get("usbl",state.acoustic_response_probability)>=.5):
        absolute.add("usbl_aided")
    optical=state.turbidity<=.35
    bottom=state.dvl_lock_probability>=.5
    water=state.dvl_water_track_probability>=.5
    if optical:absolute.add("optical_dvl" if bottom else "optical_no_bottom_lock")
    if absolute:return frozenset(absolute)
    if bottom or water:return frozenset({"relative_dead_reckoning"})
    return frozenset({"terminal_degraded"})

def observable_support(record,mode):
    observation=record["observation"];belief=record["belief"]
    services={x.name for x in observation.acoustic.service_evidence
              if x.responding and x.gives_position}
    if mode=="lbl_aided":return "lbl" in services
    if mode=="usbl_aided":return "usbl" in services
    if mode=="optical_dvl":
        return observation.optical.available and belief["optical"]>=BOUNDARY and observation.dvl.bottom_lock
    if mode=="optical_no_bottom_lock":
        return observation.optical.available and belief["optical"]>=BOUNDARY and not observation.dvl.bottom_lock
    if mode=="relative_dead_reckoning":
        absolute_observed=bool(services or
            (observation.optical.available and belief["optical"]>=BOUNDARY))
        return (not absolute_observed and belief["velocity"]>=BOUNDARY and
                (observation.dvl.bottom_lock or observation.dvl.water_track))
    if mode=="terminal_degraded":return record["action"].mission_action=="surface_for_gps"
    return False

def episodes(values,limit):
    result=[];start=0
    for end in range(1,limit+1):
        if end==limit or values[end]!=values[start]:
            if start>0 and end-start>=PERSISTENCE_SAMPLES:
                result.append((start,end,values[start]))
            start=end
    return result

def analyse_policy(kind,environment,index,fixed):
    class Recorder(Study3Policy):
        records=[]
        def step(self,observation):
            action,output=super().step(observation)
            self.__class__.records.append({"observation":observation,"action":action,
                "belief":dict(output.belief.usable_probability),
                "mode_reason":self.last_mode_decision.reason})
            return action,output
    result,trace=run_one(ROOT,environment.config.name,index,kind,fixed,
        horizon_s=HORIZON_S,dt_s=DT_S,image_period_s=4.,keep_trace=True,
        redesign_version=3,environment_realization=environment,policy_factory=Recorder)
    records=Recorder.records
    acceptable=[]
    for step,row in enumerate(trace):
        state=environment.physical_state(step,altitude_m=-row[7],position_xy=(0.,0.))
        acceptable.append(physical_acceptable_modes(state))
    terminal=next((i for i,r in enumerate(records)
                   if r["action"].mission_action=="surface_for_gps"),None)
    # Include the commitment sample so a terminal episode may match; exclude
    # every sample after commitment under terminate-after-GPS semantics.
    limit=len(records) if terminal is None else terminal+1
    detail=[]
    for start,end,modes in episodes(acceptable,limit):
        ambiguous=len(modes)>1
        adequate=next((j for j in range(start,end)
            if records[j]["action"].navigation_mode in modes and
               observable_support(records[j],records[j]["action"].navigation_mode)),None)
        preferred=next(iter(modes)) if not ambiguous else None
        exact=(next((j for j in range(start,end)
            if records[j]["action"].navigation_mode==preferred and
               observable_support(records[j],preferred)),None)
               if preferred is not None else None)
        detail.append({"start_s":start*DT_S,"end_s":end*DT_S,
            "duration_s":(end-start)*DT_S,"acceptable_modes":sorted(modes),
            "ambiguous":ambiguous,"adequate_match":adequate is not None,
            "adequate_delay_s":None if adequate is None else (adequate-start)*DT_S,
            "exact_preferred_evaluable":preferred is not None,
            "exact_preferred_match":None if preferred is None else exact is not None,
            "exact_preferred_delay_s":None if exact is None else (exact-start)*DT_S,
            "selected_at_start":records[start]["action"].navigation_mode,
            "support_at_start":observable_support(records[start],records[start]["action"].navigation_mode),
            "terminal_commit_s":None if terminal is None else terminal*DT_S})
    return result,detail,terminal

def summarize(rows):
    episodes_flat=[e for row in rows for e in row["episodes"]]
    exact=[e for e in episodes_flat if e["exact_preferred_evaluable"]]
    adequate_delays=[e["adequate_delay_s"] for e in episodes_flat if e["adequate_match"]]
    exact_delays=[e["exact_preferred_delay_s"] for e in exact if e["exact_preferred_match"]]
    return {"evaluable_post_launch_episodes":len(episodes_flat),
        "ambiguous_episodes":sum(e["ambiguous"] for e in episodes_flat),
        "adequate_viable_matches":sum(e["adequate_match"] for e in episodes_flat),
        "adequate_viable_rate":sum(e["adequate_match"] for e in episodes_flat)/len(episodes_flat),
        "adequate_delay_mean_s":float(np.mean(adequate_delays)) if adequate_delays else math.nan,
        "adequate_delay_median_s":float(np.median(adequate_delays)) if adequate_delays else math.nan,
        "adequate_delay_max_s":max(adequate_delays,default=math.nan),
        "unique_preferred_episodes":len(exact),
        "exact_preferred_matches":sum(bool(e["exact_preferred_match"]) for e in exact),
        "exact_preferred_rate":sum(bool(e["exact_preferred_match"]) for e in exact)/len(exact),
        "exact_delay_mean_s":float(np.mean(exact_delays)) if exact_delays else math.nan,
        "exact_delay_median_s":float(np.median(exact_delays)) if exact_delays else math.nan,
        "exact_delay_max_s":max(exact_delays,default=math.nan)}

config=load_environment_config(CONFIG_PATH);policy_rows={p:[] for p in ("reactive","predictive")}
environment_digests=[]
for index,seed in enumerate(SEEDS):
    environment=generate_environment(config,seed,HORIZON_S,DT_S)
    environment_digests.append({"seed":seed,"digest":environment.digest})
    fixed=deployment_informed_environment_configuration(FixedConfiguration(
        optical_channel="lidar",altitude_m=5.,speed_mps=.5,
        acoustic_technique="usbl",fusion_mode="weight"),environment)
    for name in policy_rows:
        result,detail,terminal=analyse_policy(PolicyKind(name),environment,index,fixed)
        policy_rows[name].append({"seed":seed,"trace_digest":result.trace_digest,
            "terminal_commit_s":None if terminal is None else terminal*DT_S,
            "episodes":detail})

record={"schema":"study3_generated_environment_pilot_corrected_adaptation_v1",
 "analysis_only":True,"controller_commit":"7030aa4eeb1228f447ea7729b27113aad0ecccbd",
 "root":ROOT,"seeds":SEEDS,"environment_digests":environment_digests,
 "definitions":{"persistence":"unchanged physical acceptable-mode set for >=3 samples (6 s)",
  "dominance_objective":"use any viable absolute aid; otherwise relative DVL/IMU; otherwise terminal",
  "simultaneous_absolute_modes":"unordered ambiguous acceptable set",
  "match":"selected mode belongs to acceptable set and has contemporaneous observable support",
  "post_terminal":"commitment sample included; later samples excluded"},
 "policies":{name:{"summary":summarize(rows),"seeds":rows} for name,rows in policy_rows.items()}}
canonical=json.dumps(record,sort_keys=True,separators=(",",":"),allow_nan=True).encode()
record["analysis_sha256"]=hashlib.sha256(canonical).hexdigest()
OUT.parent.mkdir(parents=True,exist_ok=True)
OUT.write_text(json.dumps(record,indent=2,sort_keys=True,allow_nan=True)+"\n")
print(json.dumps({"output":str(OUT),"analysis_sha256":record["analysis_sha256"],
 "summaries":{k:v["summary"] for k,v in record["policies"].items()}},indent=2,sort_keys=True))
