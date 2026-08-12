#!/usr/bin/env python3
"""Development-only closed-loop platform-v2 integration exercise."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from uuv_mode_aware_navigation.estimator import GRAVITY
from uuv_mode_aware_navigation.localization import P5V4CapabilityAdapter
from uuv_mode_aware_navigation.platform_v2 import (AcousticSignal, DVLSignal,
                                                    PlatformStepInput,
                                                    PlatformV2Coordinator)

IDENTIFIER="p2v2_system_integration_v1"
SEED_ROOT=22_300_000
FAMILIES=("optical","dvl","acoustic","asynchronous","compound")


def seed(family,index):
    return int.from_bytes(hashlib.sha256(f"{SEED_ROOT}:{family}:{index}".encode()).digest()[:8],"little")


def optical_packet(quality,rng):
    usable=quality>=.25
    sigma=.012+.025*(1-quality)+float(rng.uniform(0,.003))
    result={"localization_success":usable,"inliers":int(18+50*quality),
            "inlier_fraction":.55+.35*quality,"median_reprojection_px":1.3-.8*quality,
            "alternative_inliers":2,"estimated_scale":1.0,
            "covariance_eigenvalues_m2":[(.8*sigma)**2,sigma**2]}
    return P5V4CapabilityAdapter().observe(result,quality,0.0)


def run_one(family,index):
    rng=np.random.default_rng(seed(family,index));coordinator=PlatformV2Coordinator()
    altitude=3.0;speed=.5;rows=[];pending=[];last_quality=.8
    for step in range(1,81):
        t=step*.5
        optical_loss=family in {"optical","compound"} and 10<=t<32
        dvl_loss=family in {"dvl","compound"} and 16<=t<34
        infrastructure_loss=family in {"acoustic","compound"} and 18<=t<36
        quality=(.8 if not optical_loss else max(.08,.55-.035*(t-10)))
        quality=min(1.0,quality+.08*(3-altitude))
        trend=(quality-last_quality)/.5;last_quality=quality
        optical=optical_packet(quality,rng)
        lock_probability=.95 if not dvl_loss else max(.03,.5-.08*(t-16))
        dvl=DVLSignal(not dvl_loss,False,.1 if not dvl_loss else t-16,
                      lock_probability,-.08 if dvl_loss else .05)
        coordinator.estimator.predict(-GRAVITY,.5)
        if not dvl_loss: coordinator.estimator.update_velocity(np.array([.5,0,0]))
        if step%10==0:
            delay=1.2+float(rng.uniform(0,.8)) if family in {"asynchronous","compound"} else .3
            pending.append((t,t+delay,np.array([.5*t,0,-17.])+rng.normal(0,.03,3)))
        arrived=[packet for packet in pending if packet[1]<=t]
        packet=arrived[0] if arrived else None
        if packet: pending.remove(packet)
        infrastructure=not infrastructure_loss
        acoustic=AcousticSignal(
            bool(packet and infrastructure),packet[0] if packet else t,
            packet[1] if packet else t,packet[2] if packet and infrastructure else None,
            np.eye(2)*.01 if packet and infrastructure else None,
            2.0 if infrastructure else math.inf,infrastructure,5.0)
        output=coordinator.step(PlatformStepInput(
            t,.5,optical,trend,dvl,acoustic,.1,0.0,altitude,speed,0.002*speed))
        altitude=max(1.0,min(3.0,output.selected_altitude_m))
        speed=output.selected_speed_mps
        rows.append({"time_s":t,"phase_loss":bool(optical_loss or dvl_loss or infrastructure_loss),
                     "quality":quality,"altitude_m":altitude,"speed_mps":speed,
                     "optical_available":optical.available,"dvl_lock":dvl.bottom_lock,
                     "acoustic_infrastructure":infrastructure,
                     "acoustic_delay_s":packet[1]-packet[0] if packet else None,
                     "acoustic_update_accepted":output.acoustic_update_accepted,
                     "belief":dict(output.belief.usable_probability),
                     "mode":output.belief.point_mode.value,
                     "impending":sorted(output.forecast.impending),
                     "recovery":output.recovery.action.value,
                     "selected_mission_action":output.mission_action,
                     "covariance_trace":float(np.trace(coordinator.estimator.P[:3,:3]))})
    return {"family":family,"seed":seed(family,index),"rows":rows}


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,required=True);args=parser.parse_args()
    runs=[run_one(family,index) for family in FAMILIES for index in range(12)]
    criteria={
      "finite_covariance":all(math.isfinite(row["covariance_trace"]) for run in runs for row in run["rows"]),
      "delayed_acoustic_accepted":any(row["acoustic_update_accepted"] and row["acoustic_delay_s"]>.2 for run in runs for row in run["rows"]),
      "infrastructure_loss_blocks_acoustic":all(not row["acoustic_update_accepted"] for run in runs if run["family"] in {"acoustic","compound"} for row in run["rows"] if not row["acoustic_infrastructure"]),
      "optical_loss_reduces_belief":all(min(row["belief"]["optical"] for row in run["rows"] if row["phase_loss"])<.2 for run in runs if run["family"]=="optical"),
      "dvl_loss_reduces_belief":all(min(row["belief"]["velocity"] for row in run["rows"] if row["phase_loss"])<.2 for run in runs if run["family"]=="dvl"),
      "predictive_signal_precedes_or_coincides":all(any("optical" in row["impending"] for row in run["rows"] if row["time_s"]<=12) for run in runs if run["family"] in {"optical","compound"}),
      "active_recovery_exercised":all(any(row["recovery"]!="continue" for row in run["rows"]) for run in runs if run["family"] in {"optical","dvl","compound"}),
      "post_loss_recovery":all(run["rows"][-1]["belief"]["optical"]>.7 and run["rows"][-1]["belief"]["velocity"]>.7 for run in runs if run["family"] in {"optical","dvl","compound"}),
    }
    result={"identifier":IDENTIFIER,"seed_root":SEED_ROOT,"data_class":"development_only",
            "status":"DEVELOPMENT PASS" if all(criteria.values()) else "DEVELOPMENT FAIL",
            "criteria":criteria,"runs":runs}
    args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k!="runs"},indent=2,sort_keys=True))
    return 0 if result["status"]=="DEVELOPMENT PASS" else 2


if __name__=="__main__":raise SystemExit(main())
