#!/usr/bin/env python3
"""Evaluate whether rendered imagery supports classical relative localisation."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS, WaterState
from uuv_mode_aware_navigation.rendering import CameraModel, CameraPose
from uuv_mode_aware_navigation.rendering.georeferenced import (
    GeoreferencedRenderer,
    WorldTexture,
)


SEED_ROOT = 22_100_000
ATTENUATIONS = (0.2, 0.6, 1.2, 2.0)
MIN_INLIERS = 20
MAX_REPROJECTION_PX = 2.0


def _uint8(image: np.ndarray) -> np.ndarray:
    lo, hi = np.quantile(image, (0.01, 0.99))
    if hi <= lo:
        return np.zeros(image.shape, dtype=np.uint8)
    return np.clip((image - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)


def estimate_displacement(
    first: np.ndarray,
    second: np.ndarray,
    metres_per_image_pixel: float,
) -> dict:
    start = time.perf_counter()
    detector = cv2.AKAZE_create()
    key_a, des_a = detector.detectAndCompute(_uint8(first), None)
    key_b, des_b = detector.detectAndCompute(_uint8(second), None)
    if des_a is None or des_b is None:
        return {"accepted": False, "runtime_ms": (time.perf_counter() - start) * 1000.0}
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(des_a, des_b, k=2)
    good = [a for a, b in pairs if a.distance < 0.75 * b.distance]
    if len(good) < 4:
        return {"accepted": False, "runtime_ms": (time.perf_counter() - start) * 1000.0}
    src = np.float32([key_a[m.queryIdx].pt for m in good])
    dst = np.float32([key_b[m.trainIdx].pt for m in good])
    transform, mask = cv2.estimateAffinePartial2D(
        src, dst, method=cv2.RANSAC, ransacReprojThreshold=MAX_REPROJECTION_PX,
        maxIters=2000, confidence=0.995,
    )
    if transform is None or mask is None:
        return {"accepted": False, "runtime_ms": (time.perf_counter() - start) * 1000.0}
    inliers = mask.ravel().astype(bool)
    projected = src @ transform[:, :2].T + transform[:, 2]
    reprojection = np.linalg.norm(projected - dst, axis=1)
    inlier_count = int(inliers.sum())
    median_reprojection = float(np.median(reprojection[inliers])) if inlier_count else float("inf")
    accepted = inlier_count >= MIN_INLIERS and median_reprojection < MAX_REPROJECTION_PX
    return {
        "accepted": accepted,
        "dx_m": float(-transform[0, 2] * metres_per_image_pixel),
        "dy_m": float(-transform[1, 2] * metres_per_image_pixel),
        "inliers": inlier_count,
        "median_reprojection_px": median_reprojection,
        "runtime_ms": (time.perf_counter() - start) * 1000.0,
    }


def run(pairs_per_level: int = 50) -> dict:
    rng = np.random.default_rng(SEED_ROOT)
    camera = CameraModel(width_px=192, height_px=192)
    renderer = GeoreferencedRenderer(
        world=WorldTexture.generate(size_px=1024, metres_per_pixel=0.04),
        camera=camera,
        sensor_seed=SEED_ROOT + 1,
        add_sensor_noise=True,
    )
    altitude = 3.0
    footprint_m = 2.0 * altitude * np.tan(camera.horizontal_fov_rad / 2.0)
    metres_per_image_pixel = footprint_m / (camera.width_px - 1)
    levels = []
    clear_errors = []
    all_runtimes = []
    for attenuation in ATTENUATIONS:
        accepted = 0
        errors = []
        for _ in range(pairs_per_level):
            x, y = rng.uniform(-5.0, 5.0, size=2)
            dx, dy = rng.uniform(-0.35, 0.35, size=2)
            a = CameraPose(float(x), float(y), altitude)
            b = CameraPose(float(x + dx), float(y + dy), altitude)
            water = WaterState(c=attenuation)
            result = estimate_displacement(
                renderer.render(a, water, CAMERA_OFFAXIS),
                renderer.render(b, water, CAMERA_OFFAXIS),
                metres_per_image_pixel,
            )
            all_runtimes.append(result["runtime_ms"])
            if result["accepted"]:
                accepted += 1
                error = float(np.hypot(result["dx_m"] - dx, result["dy_m"] - dy))
                errors.append(error)
                if attenuation == ATTENUATIONS[0]:
                    clear_errors.append(error)
        levels.append({
            "attenuation_m_inv": attenuation,
            "accepted": accepted,
            "total": pairs_per_level,
            "recovery_rate": accepted / pairs_per_level,
            "median_error_m": float(np.median(errors)) if errors else None,
        })

    false_accepts = 0
    for _ in range(pairs_per_level):
        x, y = rng.uniform(-5.0, 5.0, size=2)
        # Separation exceeds the 3.46 m footprint, so there is no true overlap.
        a = CameraPose(float(x), float(y), altitude)
        b = CameraPose(float(x + 8.0), float(y + 8.0), altitude)
        result = estimate_displacement(
            renderer.render(a, WaterState(c=0.2), CAMERA_OFFAXIS),
            renderer.render(b, WaterState(c=0.2), CAMERA_OFFAXIS),
            metres_per_image_pixel,
        )
        false_accepts += int(result["accepted"])
        all_runtimes.append(result["runtime_ms"])

    rates = [level["recovery_rate"] for level in levels]
    criteria = {
        "A1_deterministic": np.array_equal(
            renderer.render(CameraPose(0.0, 0.0, altitude), WaterState(c=0.2), CAMERA_OFFAXIS),
            renderer.render(CameraPose(0.0, 0.0, altitude), WaterState(c=0.2), CAMERA_OFFAXIS),
        ),
        "A2_median_error_below_0_10_m": bool(clear_errors) and float(np.median(clear_errors)) < 0.10,
        "A3_clear_recovery_at_least_0_80": rates[0] >= 0.80,
        "A4_false_fix_below_0_05": false_accepts / pairs_per_level < 0.05,
        "A5_median_runtime_below_50_ms": float(np.median(all_runtimes)) < 50.0,
        "A6_recovery_monotonic": all(a >= b for a, b in zip(rates, rates[1:])),
    }
    return {
        "identifier": "p2v2_p5_spike_v1",
        "seed_root": SEED_ROOT,
        "pairs_per_level": pairs_per_level,
        "levels": levels,
        "false_fixes": false_accepts,
        "false_fix_total": pairs_per_level,
        "false_fix_rate": false_accepts / pairs_per_level,
        "median_runtime_ms": float(np.median(all_runtimes)),
        "criteria": criteria,
        "go": all(criteria.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairs", type=int, default=50)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.pairs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["go"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

