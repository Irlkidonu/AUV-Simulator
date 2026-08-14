"""Image-derived mapped ArUco localization with no vehicle-truth input."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class FiducialFix:
    position_world_m: np.ndarray
    covariance_m2: np.ndarray
    marker_count: int
    inlier_count: int
    reprojection_rmse_px: float
    condition_number: float
    optical_health: float
    marker_ids: tuple[int, ...]
    rotation_world_from_camera: np.ndarray


def camera_to_body_position(position_world_camera, rotation_world_from_camera,
                            translation_body_to_camera=(0.31,0.0,-0.06),
                            camera_pitch_rad=0.35):
    """Convert OpenCV optical-center position to the vehicle body origin."""
    c,s=math.cos(camera_pitch_rad),math.sin(camera_pitch_rad)
    rotation_body_from_cv=np.array([[0.,-s,c],[-1.,0.,0.],[0.,-c,-s]])
    rotation_world_from_body=np.asarray(rotation_world_from_camera)@rotation_body_from_cv.T
    return np.asarray(position_world_camera)-rotation_world_from_body@np.asarray(translation_body_to_camera,float)


def detector_image(image: np.ndarray, encoding: str) -> np.ndarray:
    """Convert a ROS camera array to the uint8 image consumed by ArUco."""
    values = np.asarray(image)
    if encoding == "32FC1":
        radiance = values.astype(float, copy=False)
        finite = radiance[np.isfinite(radiance)]
        if finite.size == 0:
            raise ValueError("radiance image has no finite pixels")
        lo, hi = np.percentile(finite, (1.0, 99.0))
        return np.clip((radiance-lo)/max(hi-lo, 1e-9)*255.0,
                       0.0, 255.0).astype(np.uint8)
    if values.dtype != np.uint8:
        raise ValueError(f"unsupported detector image dtype: {values.dtype}")
    if values.ndim not in (2, 3):
        raise ValueError(f"unsupported detector image shape: {values.shape}")
    return values


def camera_matrix(width: int, height: int, horizontal_fov_rad: float) -> np.ndarray:
    if width <= 0 or height <= 0 or not 0.0 < horizontal_fov_rad < math.pi:
        raise ValueError("invalid camera geometry")
    focal = width / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    return np.array([[focal, 0.0, width / 2.0],
                     [0.0, focal, height / 2.0],
                     [0.0, 0.0, 1.0]], dtype=float)


def marker_world_corners(center_xyz, size_m: float) -> np.ndarray:
    """Corners in OpenCV detector order for a horizontal upward-facing marker."""
    x, y, z = map(float, center_xyz)
    h = float(size_m) / 2.0
    return np.array([[x-h, y+h, z], [x+h, y+h, z],
                     [x+h, y-h, z], [x-h, y-h, z]], dtype=np.float64)


def detect_markers(image: np.ndarray):
    """Return unchanged ArUco detector corners and IDs for funnel accounting."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pixels = np.asarray(gray, dtype=np.uint8)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(pixels)
    else:
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(pixels, dictionary,
                                                   parameters=parameters)
    return corners, ids


def solve_mapped_fix(image: np.ndarray, marker_map: dict[int, np.ndarray],
                     intrinsic: np.ndarray, optical_health: float,
                     detection=None) -> FiducialFix | None:
    """Detect mapped DICT_4X4_50 markers and solve the camera world position."""
    corners, ids = detect_markers(image) if detection is None else detection
    if ids is None:
        return None
    object_points, image_points, used = [], [], []
    for corner, marker_id in zip(corners, ids.reshape(-1)):
        if int(marker_id) not in marker_map:
            continue
        object_points.extend(np.asarray(marker_map[int(marker_id)], dtype=float))
        image_points.extend(np.asarray(corner[0], dtype=float))
        used.append(int(marker_id))
    if len(used) < 2:
        return None
    obj = np.asarray(object_points, dtype=np.float64)
    img = np.asarray(image_points, dtype=np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj, img, intrinsic, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=3.0, confidence=0.999, iterationsCount=100)
    if not ok or inliers is None or len(inliers) < 8:
        return None
    projected, jacobian = cv2.projectPoints(obj, rvec, tvec, intrinsic, np.zeros(5))
    residual = img - projected.reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    rotation_world_to_camera, _ = cv2.Rodrigues(rvec)
    position = (-rotation_world_to_camera.T @ tvec).reshape(3)
    # Translation columns of the projection Jacobian provide a direct local
    # conditioning proxy. Scale covariance by reprojection residual and retain
    # a conservative floor for near-perfect synthetic renders.
    jt = np.asarray(jacobian[:, 3:6], dtype=float)
    normal = jt.T @ jt
    condition = float(np.linalg.cond(normal))
    variance_px2 = max(rmse**2, 0.25)
    covariance = variance_px2 * np.linalg.pinv(normal)
    covariance += np.eye(3) * 1e-4
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(covariance)):
        return None
    return FiducialFix(position, covariance, len(set(used)), len(inliers),
                       rmse, condition, float(np.clip(optical_health, 0.0, 1.0)),
                       tuple(sorted(set(used))),rotation_world_to_camera.T)
