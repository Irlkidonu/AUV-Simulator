"""Platform interface for the selected P5-v4 optical front end.

The image algorithm remains frozen in its development evidence. This adapter
turns its observable output into a fail-closed runtime capability signal.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import cv2
import numpy as np

from ..rendering import CameraPose


@dataclass(frozen=True)
class P5V4Configuration:
    detector_threshold: float = 5e-5
    ratio_threshold: float = 0.80
    minimum_inliers: int = 12
    minimum_inlier_fraction: float = 0.50
    maximum_reprojection_px: float = 2.0
    maximum_alternative_support_ratio: float = 0.50
    minimum_scale: float = 0.60
    maximum_scale: float = 1.67
    covariance_inflation: float = 2.30
    maximum_sigma_m: float = 0.10


@dataclass(frozen=True)
class OpticalLocalizationSignal:
    available: bool
    quality: float
    sigma_m: float
    age_s: float
    inliers: int
    inlier_fraction: float
    reprojection_px: float
    ambiguity_ratio: float
    reason: str
    keypoints_a: int = 0
    keypoints_b: int = 0
    matches: int = 0


class P5V4CapabilityAdapter:
    """Validate P5-v4 output without access to truth or scenario commands."""

    def __init__(self, config: P5V4Configuration = P5V4Configuration()):
        self.config = config

    def observe(self, result: Mapping[str, object], quality: float, age_s: float = 0.0):
        c = self.config
        try:
            inliers = int(result.get("inliers", 0))
            fraction = float(result.get("inlier_fraction", 0.0))
            reprojection = float(result.get("median_reprojection_px", math.inf))
            alternative = int(result.get("alternative_inliers", inliers))
            scale = float(result.get("estimated_scale", math.nan))
            eigenvalues = np.asarray(result.get("covariance_eigenvalues_m2", []), dtype=float)
            sigma = (math.sqrt(float(np.max(eigenvalues))) if eigenvalues.shape == (2,)
                     and np.all(np.isfinite(eigenvalues)) and np.all(eigenvalues > 0)
                     else math.inf)
        except (TypeError, ValueError, OverflowError):
            inliers, fraction, reprojection, alternative, scale, sigma = 0, 0.0, math.inf, 0, math.nan, math.inf
        ambiguity = alternative / max(inliers, 1)
        checks = {
            "inliers": inliers >= c.minimum_inliers,
            "consensus": fraction >= c.minimum_inlier_fraction,
            "reprojection": reprojection < c.maximum_reprojection_px,
            "ambiguity": ambiguity < c.maximum_alternative_support_ratio,
            "scale": c.minimum_scale <= scale <= c.maximum_scale,
            "uncertainty": sigma < c.maximum_sigma_m,
            "frontend": bool(result.get("localization_success", False)),
        }
        failed = next((name for name, passed in checks.items() if not passed), None)
        available = failed is None and math.isfinite(age_s) and age_s >= 0
        return OpticalLocalizationSignal(
            available, min(max(float(quality), 0.0), 1.0), sigma, float(age_s),
            inliers, fraction, reprojection, ambiguity,
            "available" if available else f"rejected_{failed or 'invalid_age'}",
            int(result.get("keypoints_a",0)),int(result.get("keypoints_b",0)),
            int(result.get("matches",0)))


@dataclass(frozen=True)
class P5V4Fix:
    localization_success: bool
    estimated_pose: CameraPose | None
    covariance_m2: np.ndarray | None
    metrics: Mapping[str, object]

    def capability_record(self):
        record=dict(self.metrics)
        record["localization_success"]=self.localization_success
        record["covariance_eigenvalues_m2"]=(
            np.linalg.eigvalsh(self.covariance_m2).tolist()
            if self.covariance_m2 is not None else [])
        return record


class P5V4ImageLocalizer:
    """Selected P5-v4 image-to-fix implementation, without truth access."""

    def __init__(self, config=P5V4Configuration(), width_px=192, height_px=192,
                 horizontal_fov_rad=math.radians(60)):
        self.config=config;self.width=width_px;self.height=height_px
        self.fov=horizontal_fov_rad
        self.centre=np.array([(width_px-1)/2,(height_px-1)/2])

    @staticmethod
    def _u8(image):
        lo,hi=np.quantile(image,(.01,.99))
        if hi<=lo:return np.zeros(image.shape,np.uint8)
        return np.clip((image-lo)*255/(hi-lo),0,255).astype(np.uint8)

    def _support(self,points):
        span=np.ptp(points,axis=0) if len(points) else np.zeros(2)
        ij=np.clip(np.floor(points/(self.width/4)).astype(int),0,3)
        grid=len({(int(x),int(y)) for x,y in ij})
        centred=points-np.mean(points,axis=0)
        singular=np.linalg.svd(centred,compute_uv=False)
        condition=(float(singular[0]/singular[-1])
                   if len(singular)>1 and singular[-1]>1e-12 else math.inf)
        hull=(cv2.contourArea(cv2.convexHull(points.astype(np.float32)))
              if len(points)>=3 else 0.0)
        covariance=np.cov(points,rowvar=False) if len(points)>=3 else np.zeros((2,2))
        return {"span_x":float(span[0]),"span_y":float(span[1]),"grid4":grid,
                "point_condition":condition,"hull":float(hull/(self.width*self.height)),
                "scatter_eigen":float(np.min(np.linalg.eigvalsh(covariance)))}

    def _alternative_support(self,src,dst,primary_mask):
        remaining=~primary_mask
        if int(np.sum(remaining))<4:return 0
        transform,mask=cv2.estimateAffinePartial2D(
            src[remaining],dst[remaining],method=cv2.RANSAC,
            ransacReprojThreshold=2,maxIters=2000,confidence=.995)
        if transform is None or mask is None or not np.all(np.isfinite(transform)):return 0
        scale=math.hypot(float(transform[0,0]),float(transform[1,0]))
        return int(np.sum(mask)) if scale>1e-8 else 0

    def _pose_from_parameters(self,params,reference):
        aa,bb,tx,ty=params;scale=math.hypot(aa,bb)
        theta=math.atan2(bb,aa);yaw=reference.yaw_rad-theta
        metres_per_pixel=(2*reference.altitude_m*math.tan(self.fov/2)/(self.width-1))/scale
        transform=np.array([[aa,-bb],[bb,aa]])
        displacement=np.array([tx,ty])-self.centre+transform@self.centre
        c,s=math.cos(yaw),math.sin(yaw)
        centre=np.array([reference.x_m,reference.y_m])-np.array([[c,-s],[s,c]])@(metres_per_pixel*displacement)
        altitude=metres_per_pixel*(self.width-1)/(2*math.tan(self.fov/2))
        return CameraPose(float(centre[0]),float(centre[1]),float(altitude),float(yaw))

    def localize(self,reference_image,query_image,reference_pose):
        c=self.config
        detector=cv2.AKAZE_create(
            descriptor_type=cv2.AKAZE_DESCRIPTOR_MLDB,descriptor_size=0,
            descriptor_channels=3,threshold=c.detector_threshold,nOctaves=4,
            nOctaveLayers=4,diffusivity=cv2.KAZE_DIFF_PM_G2)
        ka,da=detector.detectAndCompute(self._u8(reference_image),None)
        kb,db=detector.detectAndCompute(self._u8(query_image),None)
        metrics={"keypoints_a":len(ka),"keypoints_b":len(kb),
                 "detection_success":len(ka)>=c.minimum_inliers and len(kb)>=c.minimum_inliers}
        if da is None or db is None:
            return P5V4Fix(False,None,None,{**metrics,"matches":0,"match_success":False})
        pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(da,db,k=2)
        good=[x for x,y in pairs if x.distance<c.ratio_threshold*y.distance]
        metrics.update(matches=len(good),match_success=len(good)>=c.minimum_inliers)
        if len(good)<4:return P5V4Fix(False,None,None,metrics)
        src=np.float64([ka[m.queryIdx].pt for m in good]);dst=np.float64([kb[m.trainIdx].pt for m in good])
        transform,mask=cv2.estimateAffinePartial2D(
            src,dst,method=cv2.RANSAC,ransacReprojThreshold=c.maximum_reprojection_px,
            maxIters=2000,confidence=.995)
        if transform is None or mask is None or not np.all(np.isfinite(transform)):
            return P5V4Fix(False,None,None,metrics)
        aa,bb=float(transform[0,0]),float(transform[1,0]);scale=math.hypot(aa,bb)
        if scale<=1e-8:return P5V4Fix(False,None,None,{**metrics,"degenerate_transform":True})
        use=mask.ravel().astype(bool);x,y=src[use],dst[use];n=len(x)
        residual=y-(x@transform[:,:2].T+transform[:,2]);norms=np.linalg.norm(residual,axis=1)
        support_a,support_b=self._support(x),self._support(y)
        alternative=self._alternative_support(src,dst,use)
        jacobian=np.zeros((2*n,4))
        jacobian[0::2]=np.column_stack((x[:,0],-x[:,1],np.ones(n),np.zeros(n)))
        jacobian[1::2]=np.column_stack((x[:,1],x[:,0],np.zeros(n),np.ones(n)))
        variance=max(float(np.sum(residual**2)/max(2*n-4,1)),.25)
        try:covariance_parameters=np.linalg.inv(jacobian.T@jacobian)*variance
        except np.linalg.LinAlgError:covariance_parameters=np.full((4,4),np.nan)
        params=np.array([aa,bb,transform[0,2],transform[1,2]])
        pose=self._pose_from_parameters(params,reference_pose)
        numerical=np.zeros((2,4))
        for j in range(4):
            shifted=params.copy();shifted[j]+=1e-5
            shifted_pose=self._pose_from_parameters(shifted,reference_pose)
            numerical[:,j]=(np.array([shifted_pose.x_m,shifted_pose.y_m])-
                            np.array([pose.x_m,pose.y_m]))/1e-5
        covariance=numerical@covariance_parameters@numerical.T*c.covariance_inflation
        eigen=(np.linalg.eigvalsh(covariance) if np.all(np.isfinite(covariance))
               else np.array([np.nan,np.nan]))
        sigma=math.sqrt(float(np.max(eigen))) if np.all(eigen>0) else math.inf
        fraction=n/len(good)
        accepted=(n>=c.minimum_inliers and fraction>=c.minimum_inlier_fraction
                  and float(np.median(norms))<c.maximum_reprojection_px
                  and alternative<c.maximum_alternative_support_ratio*n
                  and c.minimum_scale<=scale<=c.maximum_scale
                  and np.all(eigen>0) and sigma<c.maximum_sigma_m)
        metrics.update(inliers=n,inlier_fraction=float(fraction),
                       median_reprojection_px=float(np.median(norms)),
                       alternative_inliers=alternative,estimated_scale=scale,
                       **{f"{k}_a":v for k,v in support_a.items()},
                       **{f"{k}_b":v for k,v in support_b.items()},
                       geometric_success=bool(accepted))
        return P5V4Fix(bool(accepted),pose if accepted else None,
                       covariance if accepted else None,metrics)
