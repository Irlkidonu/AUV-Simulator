#!/usr/bin/env python3
"""Run one seeded generated-environment DEVELOPMENT mechanism; never a campaign."""
from __future__ import annotations
import argparse,json,sys
from dataclasses import asdict
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/"src/uuv_mode_aware_navigation"))
from uuv_mode_aware_navigation.study3 import (FixedConfiguration,PolicyKind,
    deployment_informed_environment_configuration,generate_environment,
    load_environment_config,run_one)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,required=True)
    parser.add_argument("--environment-seed",type=int,required=True)
    parser.add_argument("--policy",choices=("deployment_fixed","reactive","predictive"),default="reactive")
    parser.add_argument("--root",type=int,default=31_891_000)
    parser.add_argument("--index",type=int,default=0)
    parser.add_argument("--horizon-s",type=float,default=180.)
    parser.add_argument("--dt-s",type=float,default=2.)
    args=parser.parse_args()
    if not 31_000_000<=args.root<32_000_000:
        raise SystemExit("generated mechanisms require a 31-million DEVELOPMENT root")
    config=load_environment_config(args.config)
    environment=generate_environment(config,args.environment_seed,args.horizon_s,args.dt_s)
    locked=FixedConfiguration(optical_channel="lidar",altitude_m=5.,speed_mps=.5,
                              acoustic_technique="usbl",fusion_mode="weight")
    fixed=deployment_informed_environment_configuration(locked,environment)
    result,trace=run_one(args.root,config.name,args.index,PolicyKind(args.policy),fixed,
        horizon_s=args.horizon_s,dt_s=args.dt_s,image_period_s=4.,keep_trace=True,
        redesign_version=3,environment_realization=environment)
    changes=[];previous=None
    for row in trace:
        action=row[6];key=(action["navigation_mode"],action["acoustic_technique"],
                           action["optical_channel"],action["mission_action"])
        if key!=previous:changes.append({"time_s":row[0],"configuration":key});previous=key
    print(json.dumps({"environment_config":config.name,"environment_seed":args.environment_seed,
        "environment_digest":environment.digest,"policy":args.policy,
        "fixed_launch_configuration":asdict(fixed),"mode_configuration_changes":changes,
        "result":asdict(result)},indent=2,sort_keys=True,allow_nan=True))

if __name__=="__main__":main()
