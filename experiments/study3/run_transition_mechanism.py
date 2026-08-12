#!/usr/bin/env python3
"""Run one DEVELOPMENT transition mechanism case; never a campaign/held-out runner."""
from __future__ import annotations
import argparse,json,sys
from dataclasses import asdict
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/"src/uuv_mode_aware_navigation"))
from uuv_mode_aware_navigation.study3 import (FixedConfiguration,PolicyKind,
    deployment_informed_transition_configuration,load_transition_scenario,run_one,
    standard_transition_scenarios)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scenario",type=Path,help="transition JSON file")
    source.add_argument("--standard",choices=tuple(standard_transition_scenarios()))
    parser.add_argument("--policy",choices=("deployment_fixed","reactive","predictive"),default="reactive")
    parser.add_argument("--root",type=int,default=31_890_000)
    parser.add_argument("--index",type=int,default=0)
    args=parser.parse_args()
    if not 31_000_000<=args.root<32_000_000:
        raise SystemExit("transition mechanisms require a 31-million DEVELOPMENT root")
    scenario=(load_transition_scenario(args.scenario) if args.scenario else
              standard_transition_scenarios()[args.standard])
    locked=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                              acoustic_technique="usbl",fusion_mode="weight")
    fixed=deployment_informed_transition_configuration(locked,scenario)
    result,trace=run_one(args.root,scenario.name,args.index,PolicyKind(args.policy),fixed,
        horizon_s=scenario.horizon_s,dt_s=2.,image_period_s=4.,keep_trace=True,
        redesign_version=3,transition_scenario=scenario)
    changes=[];previous=None
    for row in trace:
        action=row[6];key=(action["navigation_mode"],action["acoustic_technique"],
                           action["optical_channel"],action["mission_action"])
        if key!=previous:changes.append({"time_s":row[0],"configuration":key});previous=key
    print(json.dumps({"scenario":scenario.name,"policy":args.policy,
        "fixed_launch_configuration":asdict(fixed),"mode_configuration_changes":changes,
        "result":asdict(result)},indent=2,sort_keys=True,allow_nan=True))

if __name__=="__main__":main()
