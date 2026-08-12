#!/usr/bin/env python3
"""Study 3 optical-prediction DEVELOPMENT controls; never held-out."""
from __future__ import annotations
import argparse,hashlib,json,math,sys,time
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;PKG=HERE.parents[2]/"src/uuv_mode_aware_navigation"
sys.path.insert(0,str(PKG))
from uuv_mode_aware_navigation.imaging import analyse_image
from uuv_mode_aware_navigation.localization import P5V4CapabilityAdapter,P5V4ImageLocalizer
from uuv_mode_aware_navigation.optics import LIDAR,WaterState
from uuv_mode_aware_navigation.rendering import CameraPose,GeoreferencedRenderer,WorldTexture

ROOTS={"diagnosis":31_760_100,"tuning":31_761_000,"confirmation":31_762_000}
CONTROLS=("constant_texture","true_degradation","sudden_loss","recovery")
OUT=HERE/"predictor_development"

def seed(root,control,index,stream):
 value=f"{root}:{control}:{index}:{stream}".encode()
 return int.from_bytes(hashlib.sha256(value).digest()[:8],"big")%(2**32)

def quality(image):
 f=analyse_image(image);absolute=np.clip(f.structure_absolute/.12,0,1);contrast=np.clip(f.structure_contrast/.12,0,1)
 return float(math.sqrt(absolute*contrast))

def turbidity(control,u):
 if control=="constant_texture":return 0.05
 if control=="true_degradation":return float(np.clip((u-.20)/.60,0,1))
 if control=="sudden_loss":return float(u>=.70)
 if control=="recovery":return float(np.clip((u-.15)/.30,0,1)-np.clip((u-.58)/.25,0,1))
 raise ValueError(control)

def run_member(root,control,index,steps=61):
 world=WorldTexture.generate(2048,.04,seed(root,control,index,"texture"))
 renderer=GeoreferencedRenderer(world,sensor_seed=seed(root,control,index,"camera"),add_sensor_noise=False)
 localizer=P5V4ImageLocalizer();adapter=P5V4CapabilityAdapter();rows=[]
 for step in range(steps):
  u=step/(steps-1);x=-7+14*u;y=.7*math.sin(4*math.pi*u+index*.3);yaw=.08*math.sin(2*math.pi*u)
  query_pose=CameraPose(x,y,3.,yaw);reference_pose=CameraPose(x-.06,y+.04,3.,yaw-.015)
  t=turbidity(control,u);query=renderer.render(query_pose,WaterState.from_turbidity(t),LIDAR)
  reference=renderer.render(reference_pose,WaterState.from_turbidity(0.),LIDAR)
  try:
   fix=localizer.localize(reference,query,reference_pose);record=fix.capability_record()
  except ValueError:
   record={"localization_success":False,"frontend_exception":"insufficient_knn_neighbors"}
  signal=adapter.observe(record,quality(query),0.)
  rows.append({"step":step,"time_s":step*2.,"observable":{
    "available":signal.available,"quality":signal.quality,"sigma_m":signal.sigma_m,
    "keypoints_a":int(record.get("keypoints_a",0)),"keypoints_b":int(record.get("keypoints_b",0)),
    "matches":int(record.get("matches",0)),"inliers":signal.inliers,
    "inlier_fraction":signal.inlier_fraction,"reprojection_px":signal.reprojection_px,
    "ambiguity_ratio":signal.ambiguity_ratio,"reason":signal.reason},
    "evaluator":{"degradation_level":t,"degrading":control in {"true_degradation","recovery"} and .15<u<.75,
                 "sudden":control=="sudden_loss" and u>=.70}})
 return {"control":control,"index":index,"rows":rows}

def main():
 p=argparse.ArgumentParser();p.add_argument("stage",choices=ROOTS);p.add_argument("--members",type=int,default=12);a=p.parse_args()
 root=ROOTS[a.stage]
 if not 31_000_000<=root<32_000_000:raise SystemExit("DEVELOPMENT root guard")
 path=OUT/(a.stage+"_controls.json")
 if path.exists():raise SystemExit(f"refusing overwrite: {path}")
 started=time.perf_counter();members=[run_member(root,c,i) for c in CONTROLS for i in range(a.members)]
 payload={"schema":"study3_optical_prediction_controls_v1","stage":a.stage,"root":root,
          "controls":list(CONTROLS),"members_per_control":a.members,"members":members,
          "wall_runtime_s":time.perf_counter()-started}
 payload["sha256"]=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":"),allow_nan=True).encode()).hexdigest()
 path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix(".tmp");tmp.write_text(json.dumps(payload,sort_keys=True,indent=2,allow_nan=True)+"\n");tmp.replace(path)
 print(json.dumps({k:payload[k] for k in ("stage","root","members_per_control","wall_runtime_s","sha256")},indent=2))

if __name__=="__main__":main()
