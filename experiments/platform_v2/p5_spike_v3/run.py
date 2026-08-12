#!/usr/bin/env python3
"""Corrected P5 execution: safely reject degenerate similarity transforms."""

from __future__ import annotations
import argparse,importlib.util,json,math,time
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
V2_PATH=HERE.parent/"p5_spike_v2"/"run.py"
SPEC=importlib.util.spec_from_file_location("p5_v2_frozen_dependency",V2_PATH)
V2=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(V2)

IDENTIFIER="p2v2_p5_spike_v3";SEED_ROOT=22_120_000;MINIMUM_SCALE=1e-8

class SafeCv2Proxy:
    def __init__(self,wrapped):self._wrapped=wrapped
    def __getattr__(self,name):return getattr(self._wrapped,name)
    def estimateAffinePartial2D(self,*args,**kwargs):
        transform,mask=self._wrapped.estimateAffinePartial2D(*args,**kwargs)
        if transform is None or mask is None:return None,None
        transform=np.asarray(transform,dtype=float)
        if transform.shape!=(2,3) or not np.all(np.isfinite(transform)):
            return None,None
        scale=math.hypot(float(transform[0,0]),float(transform[1,0]))
        if not math.isfinite(scale) or scale<=MINIMUM_SCALE:return None,None
        return transform,mask

V2.cv2=SafeCv2Proxy(V2.cv2);V2.IDENTIFIER=IDENTIFIER;V2.SEED_ROOT=SEED_ROOT

def build_manifest():return V2.build_manifest()
def run(manifest):return V2.run(manifest)

def main():
 p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);p.add_argument("--manifest",type=Path);p.add_argument("--prepare-manifest",action="store_true");a=p.parse_args()
 if a.prepare_manifest:a.output.write_text(json.dumps(build_manifest(),indent=2,sort_keys=True)+"\n");return 0
 if a.manifest is None:p.error("--manifest required")
 started=time.time();result=run(json.loads(a.manifest.read_text()));result["wall_time_s"]=time.time()-started;a.output.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n");print(json.dumps({k:v for k,v in result.items() if k!="raw"},indent=2,sort_keys=True));return 0 if result["status"]=="FEASIBILITY PASS" else 2
if __name__=="__main__":raise SystemExit(main())
