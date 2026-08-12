#!/usr/bin/env python3
"""Execute the frozen P5-v2 optical-localisation feasibility spike once."""

from __future__ import annotations

import argparse, hashlib, json, math, time
from pathlib import Path

import cv2
import numpy as np

from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS, WaterState
from uuv_mode_aware_navigation.rendering import CameraModel, CameraPose
from uuv_mode_aware_navigation.rendering.georeferenced import GeoreferencedRenderer, WorldTexture

IDENTIFIER="p2v2_p5_spike_v2"; SEED_ROOT=22_110_000
STRATA=("T","Y","S","P","W1","W2","R","F"); PAIRS_PER_STRATUM=50
WIDTH=HEIGHT=192; FOV=math.radians(60); CENTRE=np.array([(WIDTH-1)/2,(HEIGHT-1)/2])

def _seed(label,index=0):
    return int.from_bytes(hashlib.sha256(f"{SEED_ROOT}:{label}:{index}".encode()).digest()[:8],"little")

def _pose_dict(p): return {"x":p.x_m,"y":p.y_m,"altitude":p.altitude_m,"yaw":p.yaw_rad}
def _pose(d): return CameraPose(d["x"],d["y"],d["altitude"],d["yaw"])

def _pair(rng,stratum,index):
    a=CameraPose(*rng.uniform(-7,7,2),3.0,0.0); attenuation=.2; kind="normal"
    angle=float(rng.uniform(0,2*np.pi))
    if stratum=="T": distance=float(rng.uniform(.05,.50)); b=CameraPose(a.x_m+distance*math.cos(angle),a.y_m+distance*math.sin(angle),3,0)
    elif stratum=="Y":
        distance=float(rng.uniform(.05,.30)); dyaw=math.copysign(math.radians(float(rng.uniform(2,20))),rng.uniform(-1,1)); b=CameraPose(a.x_m+distance*math.cos(angle),a.y_m+distance*math.sin(angle),3,dyaw)
    elif stratum=="S":
        while True:
            ratio=float(rng.uniform(.75,1.35)); alt_b=float(rng.uniform(2,4)); alt_a=alt_b*ratio
            if 2<=alt_a<=4: break
        a=CameraPose(a.x_m,a.y_m,alt_a,float(rng.uniform(-math.radians(5),math.radians(5))))
        distance=float(rng.uniform(.05,.30)); b=CameraPose(a.x_m+distance*math.cos(angle),a.y_m+distance*math.sin(angle),alt_b,a.yaw_rad+float(rng.uniform(-math.radians(5),math.radians(5))))
    elif stratum=="P":
        overlap=float(rng.uniform(.35,.70)); footprint=2*3*math.tan(FOV/2); shift=(1-overlap)*footprint
        yaw=float(rng.uniform(-math.radians(10),math.radians(10))); b=CameraPose(a.x_m+shift,a.y_m,3,yaw)
    else:
        distance=float(rng.uniform(.05,.30)); yaw=float(rng.uniform(-math.radians(12),math.radians(12))); altitude=float(rng.uniform(2.5,3.5)); b=CameraPose(a.x_m+distance*math.cos(angle),a.y_m+distance*math.sin(angle),altitude,yaw)
        if stratum=="W1": attenuation=.6
        elif stratum=="W2": attenuation=1.2
        elif stratum=="R": kind="repeated"
        elif stratum=="F": kind="feature_poor"
    return {"id":f"{stratum}_{index:03d}","stratum":stratum,"kind":kind,"attenuation":attenuation,"a":_pose_dict(a),"b":_pose_dict(b),"negative":False}

def build_manifest():
    pairs=[]
    for stratum in STRATA:
        rng=np.random.default_rng(_seed(f"poses:{stratum}"))
        pairs.extend(_pair(rng,stratum,i) for i in range(PAIRS_PER_STRATUM))
    rng=np.random.default_rng(_seed("negative:nonoverlap"))
    for i in range(100):
        a=CameraPose(*rng.uniform(-7,7,2),3,0); angle=float(rng.uniform(0,2*np.pi)); b=CameraPose(a.x_m+8*math.cos(angle),a.y_m+8*math.sin(angle),3,0)
        pairs.append({"id":f"N_nonoverlap_{i:03d}","stratum":"N_nonoverlap","kind":"normal","attenuation":.2,"a":_pose_dict(a),"b":_pose_dict(b),"negative":True})
    rng=np.random.default_rng(_seed("negative:repeated"))
    for i in range(50):
        a=CameraPose(*rng.uniform(-7,7,2),3,0); b=CameraPose(a.x_m+6,a.y_m,3,0)
        pairs.append({"id":f"N_repeated_{i:03d}","stratum":"N_repeated","kind":"repeated","attenuation":.2,"a":_pose_dict(a),"b":_pose_dict(b),"negative":True})
    rng=np.random.default_rng(_seed("negative:feature"))
    for i in range(50):
        a=CameraPose(*rng.uniform(-7,7,2),3,0); b=CameraPose(*rng.uniform(-7,7,2),3,0)
        pairs.append({"id":f"N_feature_{i:03d}","stratum":"N_feature","kind":"feature_independent","attenuation":.2,"a":_pose_dict(a),"b":_pose_dict(b),"negative":True,"world_b_seed":_seed("independent_world",i)})
    return {"identifier":IDENTIFIER,"seed_root":SEED_ROOT,"pairs":pairs}

def _world(kind,seed):
    if kind=="normal": return WorldTexture.generate(1024,.04,_seed("world:normal"))
    axis=np.arange(1024); xx,yy=np.meshgrid(axis,axis,indexing="xy")
    rng=np.random.default_rng(seed)
    if kind=="repeated": pixels=.5+.22*np.sin(2*np.pi*xx/36)+.18*np.cos(2*np.pi*yy/48)+.05*np.sin(2*np.pi*(xx+yy)/19)
    else:
        pixels=.5+.015*np.sin(2*np.pi*xx/300)+.015*np.cos(2*np.pi*yy/270)+rng.normal(0,.004,(1024,1024))
    half=.5*1023*.04
    return WorldTexture(np.asarray(pixels,dtype=float),.04,-half,-half)

def _u8(image):
    lo,hi=np.quantile(image,(.01,.99))
    if hi<=lo:return np.zeros(image.shape,np.uint8)
    return np.clip((image-lo)*255/(hi-lo),0,255).astype(np.uint8)

def _truth_transform(a,b):
    sa=2*a.altitude_m*math.tan(FOV/2)/(WIDTH-1); sb=2*b.altitude_m*math.tan(FOV/2)/(WIDTH-1)
    theta=a.yaw_rad-b.yaw_rad; c,s=math.cos(theta),math.sin(theta); A=(sa/sb)*np.array([[c,-s],[s,c]])
    cb,sb_y=math.cos(b.yaw_rad),math.sin(b.yaw_rad); RbT=np.array([[cb,sb_y],[-sb_y,cb]])
    d=RbT@np.array([a.x_m-b.x_m,a.y_m-b.y_m])/(2*b.altitude_m*math.tan(FOV/2)/(WIDTH-1))
    t=d+CENTRE-A@CENTRE
    return A,t

def _centre_from_params(params,a):
    aa,bb,tx,ty=params; scale=math.hypot(aa,bb); theta=math.atan2(bb,aa); yaw_b=a.yaw_rad-theta
    sb=(2*a.altitude_m*math.tan(FOV/2)/(WIDTH-1))/scale
    A=np.array([[aa,-bb],[bb,aa]]); d=np.array([tx,ty])-CENTRE+A@CENTRE
    c,s=math.cos(yaw_b),math.sin(yaw_b); centre_b=np.array([a.x_m,a.y_m])-np.array([[c,-s],[s,c]])@(sb*d)
    return centre_b,yaw_b,sb

def estimate(first,second,a,b):
    started=time.perf_counter()
    detector=cv2.AKAZE_create(descriptor_type=cv2.AKAZE_DESCRIPTOR_MLDB,descriptor_size=0,descriptor_channels=3,threshold=1e-4,nOctaves=4,nOctaveLayers=4,diffusivity=cv2.KAZE_DIFF_PM_G2)
    ka,da=detector.detectAndCompute(_u8(first),None); kb,db=detector.detectAndCompute(_u8(second),None)
    row={"keypoints_a":len(ka),"keypoints_b":len(kb),"detection_success":len(ka)>=20 and len(kb)>=20}
    if da is None or db is None:return {**row,"match_success":False,"geometric_success":False,"localization_success":False,"runtime_ms":(time.perf_counter()-started)*1000}
    pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(da,db,k=2); good=[x for x,y in pairs if x.distance<.75*y.distance]
    row.update(matches=len(good),match_success=len(good)>=20)
    if len(good)<4:return {**row,"geometric_success":False,"localization_success":False,"runtime_ms":(time.perf_counter()-started)*1000}
    src=np.float64([ka[m.queryIdx].pt for m in good]); dst=np.float64([kb[m.trainIdx].pt for m in good])
    transform,mask=cv2.estimateAffinePartial2D(src,dst,method=cv2.RANSAC,ransacReprojThreshold=2,maxIters=2000,confidence=.995)
    if transform is None or mask is None:return {**row,"geometric_success":False,"localization_success":False,"runtime_ms":(time.perf_counter()-started)*1000}
    use=mask.ravel().astype(bool); x=src[use]; y=dst[use]; n=len(x); pred=x@transform[:,:2].T+transform[:,2]; residual=(y-pred); norms=np.linalg.norm(residual,axis=1)
    hull_a=cv2.contourArea(cv2.convexHull(x.astype(np.float32))) if n>=3 else 0; hull_b=cv2.contourArea(cv2.convexHull(y.astype(np.float32))) if n>=3 else 0
    def support(points):
        quadrants=[((points[:,0]>=CENTRE[0])|(points[:,1]>=CENTRE[1]))] # placeholder overwritten below
        counts=[]
        for sx,sy in ((-1,-1),(1,-1),(-1,1),(1,1)):
            counts.append(int(np.sum((sx*(points[:,0]-CENTRE[0])>=0)&(sy*(points[:,1]-CENTRE[1])>=0))))
        occupied=sum(c>=3 for c in counts); eig=float(np.min(np.linalg.eigvalsh(np.cov(points,rowvar=False)))) if n>=3 else 0
        return occupied,eig
    qa,eiga=support(x); qb,eigb=support(y); aa=float(transform[0,0]); bb=float(transform[1,0]); scale=math.hypot(aa,bb)
    J=np.zeros((2*n,4)); J[0::2]=np.column_stack((x[:,0],-x[:,1],np.ones(n),np.zeros(n))); J[1::2]=np.column_stack((x[:,1],x[:,0],np.zeros(n),np.ones(n)))
    dof=max(2*n-4,1); variance=float(np.sum(residual**2)/dof)
    try: covp=np.linalg.inv(J.T@J)*variance
    except np.linalg.LinAlgError: covp=np.full((4,4),np.nan)
    params=np.array([aa,bb,transform[0,2],transform[1,2]]); centre_b,yaw_b,sb_est=_centre_from_params(params,a)
    numeric=np.zeros((2,4)); eps=1e-5
    for j in range(4):
        shifted=params.copy(); shifted[j]+=eps; numeric[:,j]=(_centre_from_params(shifted,a)[0]-centre_b)/eps
    covxy=numeric@covp@numeric.T; eigcov=np.linalg.eigvalsh(covxy) if np.all(np.isfinite(covxy)) else np.array([np.nan,np.nan])
    geometric=(n>=20 and np.median(norms)<2 and hull_a/(WIDTH*HEIGHT)>=.15 and hull_b/(WIDTH*HEIGHT)>=.15 and qa>=3 and qb>=3 and eiga>=.04*min(WIDTH,HEIGHT)**2 and eigb>=.04*min(WIDTH,HEIGHT)**2 and .60<=scale<=1.67 and np.all(eigcov>0) and math.sqrt(float(np.max(eigcov)))<.10)
    yaw_error=abs(math.atan2(math.sin(yaw_b-b.yaw_rad),math.cos(yaw_b-b.yaw_rad))); true_sb=2*b.altitude_m*math.tan(FOV/2)/(WIDTH-1)
    row.update(inliers=n,median_reprojection_px=float(np.median(norms)) if n else None,hull_a=hull_a/(WIDTH*HEIGHT),hull_b=hull_b/(WIDTH*HEIGHT),quadrants_a=qa,quadrants_b=qb,scatter_eigen_a=eiga,scatter_eigen_b=eigb,estimated_scale=scale,geometric_success=bool(geometric),localization_success=bool(geometric),translation_error_m=float(np.linalg.norm(centre_b-[b.x_m,b.y_m])),yaw_error_deg=math.degrees(yaw_error),scale_error_fraction=abs(sb_est-true_sb)/true_sb,covariance_m2=covxy.tolist(),covariance_eigenvalues_m2=eigcov.tolist(),runtime_ms=(time.perf_counter()-started)*1000)
    if geometric:
        error=centre_b-np.array([b.x_m,b.y_m]); row["nees"]=float(error@np.linalg.solve(covxy,error)); row["ellipse_95_contains_truth"]=row["nees"]<=5.991464547
    return row

def _interval_zero(n): return [0.0,1-math.pow(.025,1/n)]
def _summary(rows):
    success=[r for r in rows if r["localization_success"]]; vals=lambda k:[r[k] for r in success]
    return {"total":len(rows),"detection_success_rate":sum(r["detection_success"] for r in rows)/len(rows),"match_success_rate":sum(r["match_success"] for r in rows)/len(rows),"geometric_success_rate":sum(r["geometric_success"] for r in rows)/len(rows),"localization_success_rate":len(success)/len(rows),"median_translation_error_m":float(np.median(vals("translation_error_m"))) if success else None,"p95_translation_error_m":float(np.percentile(vals("translation_error_m"),95)) if success else None,"median_yaw_error_deg":float(np.median(vals("yaw_error_deg"))) if success else None,"p95_yaw_error_deg":float(np.percentile(vals("yaw_error_deg"),95)) if success else None,"median_scale_error_fraction":float(np.median(vals("scale_error_fraction"))) if success else None,"p95_scale_error_fraction":float(np.percentile(vals("scale_error_fraction"),95)) if success else None,"ellipse_95_coverage":float(np.mean(vals("ellipse_95_contains_truth"))) if success else None,"median_runtime_ms":float(np.median([r["runtime_ms"] for r in rows])),"p95_runtime_ms":float(np.percentile([r["runtime_ms"] for r in rows],95))}

def run(manifest):
    if manifest!=build_manifest():raise RuntimeError("manifest differs from frozen generator")
    worlds={k:_world(k,_seed(f"world:{k}")) for k in ("normal","repeated","feature_poor")}; camera=CameraModel(WIDTH,HEIGHT,FOV); renderers={k:GeoreferencedRenderer(v,camera,_seed(f"sensor:{k}"),True) for k,v in worlds.items()}
    raw={}
    for pair in manifest["pairs"]:
        kind=pair["kind"]; base_kind="feature_poor" if kind=="feature_independent" else kind; a,b=_pose(pair["a"]),_pose(pair["b"]); water=WaterState(c=pair["attenuation"])
        first=renderers[base_kind].render(a,water,CAMERA_OFFAXIS)
        if kind=="feature_independent": second=GeoreferencedRenderer(_world("feature_poor",pair["world_b_seed"]),camera,pair["world_b_seed"]+1,True).render(b,water,CAMERA_OFFAXIS)
        else: second=renderers[base_kind].render(b,water,CAMERA_OFFAXIS)
        result=estimate(first,second,a,b); false=bool(result["localization_success"] and (pair["negative"] or result.get("translation_error_m",math.inf)>.5 or result.get("yaw_error_deg",math.inf)>5 or result.get("scale_error_fraction",math.inf)>.10)); result["false_fix"]=false; raw.setdefault(pair["stratum"],[]).append(result)
    summaries={k:_summary(v) for k,v in raw.items()}; positives=[r for k,v in raw.items() if not k.startswith("N_") for r in v]; negatives=[r for k,v in raw.items() if k.startswith("N_") for r in v]
    clear=[r for k in ("T","Y","S") for r in raw[k] if r["localization_success"]]; ys=[r for k in ("Y","S") for r in raw[k]]; s_success=[r for r in raw["S"] if r["localization_success"]]; y_success=[r for r in raw["Y"] if r["localization_success"]]
    rates=[sum(r["localization_success"] for k in ("T","Y","S") for r in raw[k])/150,summaries["W1"]["localization_success_rate"],summaries["W2"]["localization_success_rate"]]
    criteria={
      "T_success_at_least_0_90":summaries["T"]["localization_success_rate"]>=.90,
      "YS_success_at_least_0_80":sum(r["localization_success"] for r in ys)/len(ys)>=.80,
      "P_success_at_least_0_70":summaries["P"]["localization_success_rate"]>=.70,
      "clear_translation_error":bool(clear) and np.median([r["translation_error_m"] for r in clear])<.10 and np.percentile([r["translation_error_m"] for r in clear],95)<.25,
      "Y_yaw_error":bool(y_success) and np.median([r["yaw_error_deg"] for r in y_success])<1 and np.percentile([r["yaw_error_deg"] for r in y_success],95)<3,
      "S_scale_error":bool(s_success) and np.median([r["scale_error_fraction"] for r in s_success])<.02 and np.percentile([r["scale_error_fraction"] for r in s_success],95)<.05,
      "zero_negative_false_fixes":sum(r["false_fix"] for r in negatives)==0,
      "positive_false_fix_below_0_01_each":all(sum(r["false_fix"] for r in raw[k])/len(raw[k])<.01 for k in STRATA),
      "repeated_ambiguity_rejection_at_least_0_95":1-summaries["R"]["localization_success_rate"]>=.95,
      "feature_poor_rejection_at_least_0_95":1-summaries["F"]["localization_success_rate"]>=.95,
      "success_monotonic_with_attenuation":rates[0]>=rates[1]>=rates[2],
      "covariance_positive":all(np.all(np.asarray(r["covariance_eigenvalues_m2"])>0) for r in positives if r["localization_success"]),
      "clear_ellipse_coverage_0_85_to_0_99":bool(clear) and .85<=np.mean([r["ellipse_95_contains_truth"] for r in clear])<=.99,
      "runtime_median_below_50_ms":np.median([r["runtime_ms"] for r in positives+negatives])<50,
      "runtime_p95_below_100_ms":np.percentile([r["runtime_ms"] for r in positives+negatives],95)<100,
    }
    return {"identifier":IDENTIFIER,"seed_root":SEED_ROOT,"status":"FEASIBILITY PASS" if all(criteria.values()) else "FAIL","criteria":{k:bool(v) for k,v in criteria.items()},"summaries":summaries,"negative_false_fixes":sum(r["false_fix"] for r in negatives),"negative_false_fix_exact_95_interval":_interval_zero(len(negatives)) if not any(r["false_fix"] for r in negatives) else None,"attenuation_success_rates":rates,"raw":raw}

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);p.add_argument("--manifest",type=Path);p.add_argument("--prepare-manifest",action="store_true");a=p.parse_args()
    if a.prepare_manifest:a.output.write_text(json.dumps(build_manifest(),indent=2,sort_keys=True)+"\n");return 0
    if a.manifest is None:p.error("--manifest required")
    started=time.time();result=run(json.loads(a.manifest.read_text()));result["wall_time_s"]=time.time()-started;a.output.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n");print(json.dumps({k:v for k,v in result.items() if k!="raw"},indent=2,sort_keys=True));return 0 if result["status"]=="FEASIBILITY PASS" else 2
if __name__=="__main__":raise SystemExit(main())
