#!/usr/bin/env python3
"""P5-v4 development runner with direct geometry and ambiguity checks."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
V2_PATH = HERE.parent / "p5_spike_v2" / "run.py"
SPEC = importlib.util.spec_from_file_location("p5_v2_readonly_dependency", V2_PATH)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)

WIDTH = HEIGHT = 192
CENTRE = np.array([(WIDTH - 1) / 2, (HEIGHT - 1) / 2])


def configure(seed_root):
    BASE.SEED_ROOT = int(seed_root)
    BASE.IDENTIFIER = f"p2v2_p5_v4_dev_{seed_root}"


def build_manifest(seed_root):
    configure(seed_root)
    return BASE.build_manifest()


def _support(points):
    span = np.ptp(points, axis=0) if len(points) else np.zeros(2)
    ij = np.clip(np.floor(points / (WIDTH / 4)).astype(int), 0, 3)
    grid = len({(int(x), int(y)) for x, y in ij})
    centred = points - np.mean(points, axis=0)
    singular = np.linalg.svd(centred, compute_uv=False)
    condition = (float(singular[0] / singular[-1])
                 if len(singular) > 1 and singular[-1] > 1e-12 else math.inf)
    hull = (BASE.cv2.contourArea(BASE.cv2.convexHull(points.astype(np.float32)))
            if len(points) >= 3 else 0.0)
    covariance = np.cov(points, rowvar=False) if len(points) >= 3 else np.zeros((2, 2))
    return {
        "span_x": float(span[0]), "span_y": float(span[1]), "grid4": grid,
        "point_condition": condition, "hull": float(hull / (WIDTH * HEIGHT)),
        "scatter_eigen": float(np.min(np.linalg.eigvalsh(covariance))),
    }


def _alternative_support(src, dst, primary_mask):
    remaining = ~primary_mask
    if int(np.sum(remaining)) < 4:
        return 0
    transform, mask = BASE.cv2.estimateAffinePartial2D(
        src[remaining], dst[remaining], method=BASE.cv2.RANSAC,
        ransacReprojThreshold=2, maxIters=2000, confidence=.995)
    if transform is None or mask is None or not np.all(np.isfinite(transform)):
        return 0
    scale = math.hypot(float(transform[0, 0]), float(transform[1, 0]))
    return int(np.sum(mask)) if scale > 1e-8 else 0


def estimate(first, second, a, b, covariance_inflation):
    started = time.perf_counter()
    detector = BASE.cv2.AKAZE_create(
        descriptor_type=BASE.cv2.AKAZE_DESCRIPTOR_MLDB, descriptor_size=0,
        descriptor_channels=3, threshold=5e-5, nOctaves=4, nOctaveLayers=4,
        diffusivity=BASE.cv2.KAZE_DIFF_PM_G2)
    ka, da = detector.detectAndCompute(BASE._u8(first), None)
    kb, db = detector.detectAndCompute(BASE._u8(second), None)
    row = {"keypoints_a": len(ka), "keypoints_b": len(kb),
           "detection_success": len(ka) >= 12 and len(kb) >= 12}
    if da is None or db is None:
        return {**row, "matches": 0, "match_success": False,
                "geometric_success": False, "localization_success": False,
                "runtime_ms": (time.perf_counter() - started) * 1000}
    pairs = BASE.cv2.BFMatcher(BASE.cv2.NORM_HAMMING).knnMatch(da, db, k=2)
    good = [x for x, y in pairs if x.distance < .80 * y.distance]
    row.update(matches=len(good), match_success=len(good) >= 12)
    if len(good) < 4:
        return {**row, "geometric_success": False, "localization_success": False,
                "runtime_ms": (time.perf_counter() - started) * 1000}
    src = np.float64([ka[m.queryIdx].pt for m in good])
    dst = np.float64([kb[m.trainIdx].pt for m in good])
    transform, mask = BASE.cv2.estimateAffinePartial2D(
        src, dst, method=BASE.cv2.RANSAC, ransacReprojThreshold=2,
        maxIters=2000, confidence=.995)
    if transform is None or mask is None or not np.all(np.isfinite(transform)):
        return {**row, "geometric_success": False, "localization_success": False,
                "runtime_ms": (time.perf_counter() - started) * 1000}
    aa, bb = float(transform[0, 0]), float(transform[1, 0])
    scale = math.hypot(aa, bb)
    if scale <= 1e-8:
        return {**row, "geometric_success": False, "localization_success": False,
                "degenerate_transform": True,
                "runtime_ms": (time.perf_counter() - started) * 1000}
    use = mask.ravel().astype(bool)
    x, y = src[use], dst[use]
    n = len(x)
    pred = x @ transform[:, :2].T + transform[:, 2]
    residual = y - pred
    norms = np.linalg.norm(residual, axis=1)
    support_a, support_b = _support(x), _support(y)
    alternative = _alternative_support(src, dst, use)

    jacobian = np.zeros((2 * n, 4))
    jacobian[0::2] = np.column_stack((x[:, 0], -x[:, 1], np.ones(n), np.zeros(n)))
    jacobian[1::2] = np.column_stack((x[:, 1], x[:, 0], np.zeros(n), np.ones(n)))
    # A half-pixel floor represents keypoint quantisation/model mismatch even
    # when RANSAC selects an unusually exact small sample.
    variance = max(float(np.sum(residual**2) / max(2 * n - 4, 1)), 0.25)
    try:
        covariance_parameters = np.linalg.inv(jacobian.T @ jacobian) * variance
    except np.linalg.LinAlgError:
        covariance_parameters = np.full((4, 4), np.nan)
    params = np.array([aa, bb, transform[0, 2], transform[1, 2]])
    centre_b, yaw_b, sb_est = BASE._centre_from_params(params, a)
    numerical = np.zeros((2, 4))
    for j in range(4):
        shifted = params.copy(); shifted[j] += 1e-5
        numerical[:, j] = (BASE._centre_from_params(shifted, a)[0] - centre_b) / 1e-5
    covariance_xy = numerical @ covariance_parameters @ numerical.T * covariance_inflation
    covariance_eigen = (np.linalg.eigvalsh(covariance_xy)
                        if np.all(np.isfinite(covariance_xy)) else np.array([np.nan, np.nan]))
    sigma = math.sqrt(float(np.max(covariance_eigen))) if np.all(covariance_eigen > 0) else math.inf
    inlier_fraction = n / len(good)
    geometry = (
        n >= 12 and inlier_fraction >= .50 and float(np.median(norms)) < 2
        and alternative < .50 * n and .60 <= scale <= 1.67
        and np.all(covariance_eigen > 0) and sigma < .10
    )
    truth = np.array([b.x_m, b.y_m])
    error_vector = centre_b - truth
    yaw_error = abs(math.atan2(math.sin(yaw_b - b.yaw_rad), math.cos(yaw_b - b.yaw_rad)))
    true_sb = 2 * b.altitude_m * math.tan(BASE.FOV / 2) / (WIDTH - 1)
    row.update(
        inliers=n, inlier_fraction=float(inlier_fraction),
        median_reprojection_px=float(np.median(norms)), alternative_inliers=alternative,
        estimated_scale=scale, **{f"{k}_a": v for k, v in support_a.items()},
        **{f"{k}_b": v for k, v in support_b.items()},
        covariance_m2=covariance_xy.tolist(), covariance_eigenvalues_m2=covariance_eigen.tolist(),
        translation_error_m=float(np.linalg.norm(error_vector)),
        yaw_error_deg=math.degrees(yaw_error),
        scale_error_fraction=abs(sb_est - true_sb) / true_sb,
        horizontal_nees=(float(error_vector @ np.linalg.solve(covariance_xy, error_vector))
                         if np.all(covariance_eigen > 0) else math.inf),
        geometric_success=bool(geometry), localization_success=bool(geometry),
        runtime_ms=(time.perf_counter() - started) * 1000)
    if geometry:
        row["ellipse_95_contains_truth"] = row["horizontal_nees"] <= 5.991464547
    return row


def summarize(rows):
    accepted = [r for r in rows if r["localization_success"]]
    def percentile(key, q):
        return float(np.percentile([r[key] for r in accepted], q)) if accepted else None
    return {
        "total": len(rows),
        "detection_success_rate": float(np.mean([r["detection_success"] for r in rows])),
        "match_success_rate": float(np.mean([r["match_success"] for r in rows])),
        "geometric_verification_success_rate": len(accepted) / len(rows),
        "localization_success_rate": len(accepted) / len(rows),
        "median_translation_error_m": percentile("translation_error_m", 50),
        "p95_translation_error_m": percentile("translation_error_m", 95),
        "median_yaw_error_deg": percentile("yaw_error_deg", 50),
        "p95_yaw_error_deg": percentile("yaw_error_deg", 95),
        "median_scale_error_fraction": percentile("scale_error_fraction", 50),
        "p95_scale_error_fraction": percentile("scale_error_fraction", 95),
        "ellipse_95_coverage": (float(np.mean([r["ellipse_95_contains_truth"] for r in accepted]))
                                if accepted else None),
        "median_runtime_ms": float(np.median([r["runtime_ms"] for r in rows])),
        "p95_runtime_ms": float(np.percentile([r["runtime_ms"] for r in rows], 95)),
    }


def run(manifest, covariance_inflation):
    seed_root = manifest["seed_root"]
    if manifest != build_manifest(seed_root):
        raise RuntimeError("manifest differs from deterministic generator")
    worlds = {k: BASE._world(k, BASE._seed(f"world:{k}"))
              for k in ("normal", "repeated", "feature_poor")}
    camera = BASE.CameraModel(WIDTH, HEIGHT, BASE.FOV)
    renderers = {k: BASE.GeoreferencedRenderer(v, camera, BASE._seed(f"sensor:{k}"), True)
                 for k, v in worlds.items()}
    raw = {}
    for pair in manifest["pairs"]:
        kind = pair["kind"]
        base_kind = "feature_poor" if kind == "feature_independent" else kind
        a, b = BASE._pose(pair["a"]), BASE._pose(pair["b"])
        water = BASE.WaterState(c=pair["attenuation"])
        first = renderers[base_kind].render(a, water, BASE.CAMERA_OFFAXIS)
        if kind == "feature_independent":
            renderer = BASE.GeoreferencedRenderer(
                BASE._world("feature_poor", pair["world_b_seed"]), camera,
                pair["world_b_seed"] + 1, True)
            second = renderer.render(b, water, BASE.CAMERA_OFFAXIS)
        else:
            second = renderers[base_kind].render(b, water, BASE.CAMERA_OFFAXIS)
        scored = estimate(first, second, a, b, covariance_inflation)
        scored["false_fix"] = bool(scored["localization_success"] and (
            pair["negative"] or scored.get("translation_error_m", math.inf) > .50
            or scored.get("yaw_error_deg", math.inf) > 5
            or scored.get("scale_error_fraction", math.inf) > .10))
        raw.setdefault(pair["stratum"], []).append(scored)
    summaries = {key: summarize(rows) for key, rows in raw.items()}
    negative = [r for key, rows in raw.items() if key.startswith("N_") for r in rows]
    positive = [r for key, rows in raw.items() if not key.startswith("N_") for r in rows]
    clear = [r for key in ("T", "Y", "S") for r in raw[key] if r["localization_success"]]
    ys = [r for key in ("Y", "S") for r in raw[key]]
    criteria = {
        "T_success_at_least_0_90": summaries["T"]["localization_success_rate"] >= .90,
        "YS_success_at_least_0_80": np.mean([r["localization_success"] for r in ys]) >= .80,
        "P_success_at_least_0_60": summaries["P"]["localization_success_rate"] >= .60,
        "clear_translation_error": bool(clear) and np.median([r["translation_error_m"] for r in clear]) < .10 and np.percentile([r["translation_error_m"] for r in clear], 95) < .25,
        "Y_yaw_error": bool(clear) and summaries["Y"]["median_yaw_error_deg"] < 1 and summaries["Y"]["p95_yaw_error_deg"] < 3,
        "S_scale_error": bool(clear) and summaries["S"]["median_scale_error_fraction"] < .02 and summaries["S"]["p95_scale_error_fraction"] < .05,
        "zero_negative_false_fixes": not any(r["false_fix"] for r in negative),
        "positive_false_fix_below_0_01_each": all(np.mean([r["false_fix"] for r in raw[k]]) < .01 for k in BASE.STRATA),
        "clear_ellipse_coverage_0_85_to_0_99": bool(clear) and .85 <= np.mean([r["ellipse_95_contains_truth"] for r in clear]) <= .99,
        "runtime_median_below_50_ms": np.median([r["runtime_ms"] for r in positive + negative]) < 50,
        "runtime_p95_below_100_ms": np.percentile([r["runtime_ms"] for r in positive + negative], 95) < 100,
        "success_monotonic_with_attenuation": np.mean([r["localization_success"] for key in ("T", "Y", "S") for r in raw[key]]) >= summaries["W1"]["localization_success_rate"] >= summaries["W2"]["localization_success_rate"],
    }
    return {"identifier": manifest["identifier"], "seed_root": seed_root,
            "covariance_inflation": covariance_inflation,
            "status": "DEVELOPMENT PASS" if all(criteria.values()) else "DEVELOPMENT FAIL",
            "criteria": {k: bool(v) for k, v in criteria.items()}, "summaries": summaries,
            "negative_false_fixes": sum(r["false_fix"] for r in negative), "raw": raw}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed-root", type=int, required=True)
    parser.add_argument("--covariance-inflation", type=float, default=1.0)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--prepare-manifest", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.prepare_manifest:
        args.output.write_text(json.dumps(build_manifest(args.seed_root), indent=2, sort_keys=True) + "\n")
        return 0
    if args.manifest is None:
        parser.error("--manifest is required")
    started = time.time()
    result = run(json.loads(args.manifest.read_text()), args.covariance_inflation)
    result["wall_time_s"] = time.time() - started
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "raw"}, indent=2, sort_keys=True))
    return 0 if result["status"] == "DEVELOPMENT PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
