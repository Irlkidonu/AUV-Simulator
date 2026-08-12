#!/usr/bin/env python3
"""Atomic, resumable Study 3 DEVELOPMENT orchestration.

The reserved held-out root is rejected unconditionally. Each run is written by
temporary-file replacement and an existing packet is accepted only after its
identity and checksum verify.
"""
from __future__ import annotations

import argparse,hashlib,itertools,json,os,sys,time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[2]/"src/uuv_mode_aware_navigation"
sys.path.insert(0,str(PACKAGE))
from uuv_mode_aware_navigation.study3 import FAMILIES,PRIMARY,FixedConfiguration,PolicyKind,run_one

ROOTS={"scenario":31_200_000,"fixed":31_100_000,"adaptive":31_300_000,
       "confirmation":31_400_000}
OUTPUT=HERE/"development_results"


def digest(payload):
    return hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()


def packet_path(stage,key):return OUTPUT/stage/(key+".json")


def execute(task):
    stage,root,family,index,kind,config_id,config=task
    identity={"stage":stage,"root":root,"family":family,"index":index,
              "policy":kind,"configuration_id":config_id,"configuration":config}
    key=digest(identity)[:24];path=packet_path(stage,key)
    if path.exists():
        packet=json.loads(path.read_text())
        stored=packet.pop("packet_sha256",None)
        if packet.get("identity")!=identity or digest(packet)!=stored:
            raise RuntimeError(f"invalid resume packet {path}")
        return packet["result"]
    fixed=FixedConfiguration(**config)
    result=asdict(run_one(root,family,index,PolicyKind(kind),fixed))
    result["configuration_id"]=config_id
    packet={"schema":"study3_development_packet_v1","identity":identity,"result":result}
    packet["packet_sha256"]=digest(packet)
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(packet,sort_keys=True,indent=2)+"\n")
    os.replace(temporary,path)
    return result


def run_tasks(tasks,workers):
    started=time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers) as pool:results=list(pool.map(execute,tasks,chunksize=1))
    return results,time.perf_counter()-started


def rank(results):
    by={}
    for r in results:by.setdefault(r["configuration_id"],[]).append(r)
    scored=[]
    for cid,rows in by.items():
        n=len(rows);score=(sum(x["safety_violation"] for x in rows)/n,
            -sum(x["completed"] for x in rows)/n,
            sum(x["unaided_time_s"] for x in rows)/n,
            sum(x["rmse_transition_m"] for x in rows)/n,
            sum(x["mission_duration_s"] for x in rows)/n,cid)
        scored.append((score,cid))
    return [cid for _,cid in sorted(scored)]


def configurations():
    values=itertools.product(("camera_coaxial","camera_offaxis","lidar"),(1.,3.,5.),
        (.25,.5,.75),("single_beacon","lbl","usbl"),("gate","weight"))
    return {f"fixed_{i:03d}":asdict(FixedConfiguration(*x)) for i,x in enumerate(values)}


def save_summary(stage,root,results,elapsed,extra=None):
    data={"schema":"study3_development_summary_v1","stage":stage,"root":root,
          "executions":len(results),"wall_runtime_s":elapsed,"result_digest":digest(results),
          "created_utc":time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime())}
    if extra:data.update(extra)
    path=OUTPUT/f"{stage}_summary.json";path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(".tmp");tmp.write_text(json.dumps(data,sort_keys=True,indent=2)+"\n");os.replace(tmp,path)
    return data


def main():
    p=argparse.ArgumentParser();p.add_argument("stage",choices=tuple(ROOTS));p.add_argument("--workers",type=int,default=4)
    p.add_argument("--iteration",type=int,default=1)
    args=p.parse_args();base_stage=args.stage
    if args.iteration<1:raise SystemExit("iteration must be positive")
    stage=base_stage if args.iteration==1 else f"{base_stage}_v{args.iteration}"
    root=ROOTS[base_stage]+100*(args.iteration-1)
    if root==32_000_000:raise SystemExit("reserved held-out root is forbidden")
    default=asdict(FixedConfiguration())
    if base_stage=="scenario":
        tasks=[(stage,root,f,i,k.value,"default",default) for f in FAMILIES for i in range(8) for k in PolicyKind]
        results,elapsed=run_tasks(tasks,args.workers);summary=save_summary(stage,root,results,elapsed)
    elif base_stage=="fixed":
        configs=configurations();all_results=[];times=0.
        tasks=[("fixed_stage1",root,f,0,"fixed",cid,c) for cid,c in configs.items() for f in FAMILIES]
        r,e=run_tasks(tasks,args.workers);all_results+=r;times+=e;top18=rank(r)[:18]
        tasks=[("fixed_stage2",root,f,i,"fixed",cid,configs[cid]) for cid in top18 for i in range(1,5) for f in FAMILIES]
        r2,e=run_tasks(tasks,args.workers);all_results+=r2;times+=e;top4=rank(r+r2)[:4]
        tasks=[("fixed_stage3",root,f,i,"fixed",cid,configs[cid]) for cid in top4 for i in range(5,17) for f in FAMILIES]
        r3,e=run_tasks(tasks,args.workers);all_results+=r3;times+=e;winner=rank(all_results)[0]
        summary=save_summary(stage,root,all_results,times,{"top18":top18,"top4":top4,
            "winner_id":winner,"winner":configs[winner],"ranking":rank(all_results)})
    else:
        fixed_iteration=min(args.iteration,2)  # mission geometry last changed in v2
        fixed_name=("fixed_summary.json" if fixed_iteration==1
                    else f"fixed_v{fixed_iteration}_summary.json")
        fixed_summary=json.loads((OUTPUT/fixed_name).read_text());winner=fixed_summary["winner"]
        if base_stage=="adaptive":
            # Four predeclared shared inference/recovery bundles. REACTIVE uses
            # the selected bundle too; only PREDICTIVE is permitted to consume
            # its forecast.
            bundles={
              "balanced":{**winner,"prediction_horizon_s":10.,"optical_quality_floor":.25,
                           "usable_probability_boundary":.35,"trend_confirmation_frames":3,
                           "minimum_cumulative_quality_decline":.18},
              "conservative":{**winner,"prediction_horizon_s":8.,"optical_quality_floor":.22,
                               "usable_probability_boundary":.30,"trend_confirmation_frames":3,
                               "minimum_cumulative_quality_decline":.22},
              "early":{**winner,"prediction_horizon_s":12.,"optical_quality_floor":.28,
                        "usable_probability_boundary":.40,"trend_confirmation_frames":3,
                        "minimum_cumulative_quality_decline":.15},
              "confirmed":{**winner,"prediction_horizon_s":10.,"optical_quality_floor":.25,
                            "usable_probability_boundary":.35,"trend_confirmation_frames":4,
                            "minimum_cumulative_quality_decline":.18}}
            tasks=[(stage,root,f,i,"predictive",bid,c) for bid,c in bundles.items() for f in PRIMARY for i in range(10)]
            tasks += [(stage,root,f,i,"reactive","shared_reference",winner) for f in PRIMARY for i in range(10)]
            results,elapsed=run_tasks(tasks,args.workers)
            # rank predictive bundles on the same lexicographic outcome ordering.
            predictive=[r for r in results if r["policy"]=="predictive"]
            candidates=rank(predictive)
            if args.iteration>=5:
                nominal=json.loads((OUTPUT/"adaptive_nominal_summary.json").read_text())
                eligible=set(nominal["eligible_bundles"])
                candidates=[x for x in candidates if x in eligible]
                if not candidates:raise RuntimeError("no adaptive bundle satisfies nominal margin")
            selected=candidates[0]
            summary=save_summary(stage,root,results,elapsed,{"selected_bundle":selected,"selected":bundles[selected]})
        else:
            adaptive_name=("adaptive_summary.json" if args.iteration==1
                           else f"adaptive_v{args.iteration}_summary.json")
            adaptive=json.loads((OUTPUT/adaptive_name).read_text());selected=adaptive["selected"]
            tasks=[]
            for f in FAMILIES:
                for i in range(15):
                    tasks.extend([(stage,root,f,i,"fixed","winner",winner),
                                  (stage,root,f,i,"reactive","selected",selected),
                                  (stage,root,f,i,"predictive","selected",selected)])
            results,elapsed=run_tasks(tasks,args.workers);summary=save_summary(stage,root,results,elapsed)
    print(json.dumps(summary,sort_keys=True,indent=2))


if __name__=="__main__":main()
