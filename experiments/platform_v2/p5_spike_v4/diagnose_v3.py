#!/usr/bin/env python3
"""Read-only diagnostic replay of P5-v3 candidate geometry.

This does not score or replace P5-v3. It reconstructs metrics that its immutable
result did not store, using the same manifest, renderer, detector and matcher.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
V3_PATH = HERE.parent / "p5_spike_v3" / "run.py"
SPEC = importlib.util.spec_from_file_location("p5_v3_diagnostic_dependency", V3_PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


class GeometryRecorder:
    def __init__(self, wrapped):
        self.wrapped = wrapped
        self.rows = []

    def __getattr__(self, name):
        return getattr(self.wrapped, name)

    @staticmethod
    def _grid_occupancy(points, cells=4):
        if not len(points):
            return 0
        ij = np.floor(points / (192.0 / cells)).astype(int)
        ij = np.clip(ij, 0, cells - 1)
        return len({(int(x), int(y)) for x, y in ij})

    def estimateAffinePartial2D(self, src, dst, *args, **kwargs):
        transform, mask = self.wrapped.estimateAffinePartial2D(src, dst, *args, **kwargs)
        row = {"candidate_correspondences": int(len(src))}
        if transform is None or mask is None:
            row["transform_available"] = False
            self.rows.append(row)
            return transform, mask
        use = np.asarray(mask).ravel().astype(bool)
        a = np.asarray(src)[use]
        b = np.asarray(dst)[use]
        centred = a - np.mean(a, axis=0)
        singular = np.linalg.svd(centred, compute_uv=False)
        # Centred similarity Jacobian separates translation from rotation/scale.
        jacobian = np.zeros((2 * len(a), 4))
        jacobian[0::2] = np.column_stack(
            (centred[:, 0] / 192.0, -centred[:, 1] / 192.0, np.ones(len(a)), np.zeros(len(a)))
        )
        jacobian[1::2] = np.column_stack(
            (centred[:, 1] / 192.0, centred[:, 0] / 192.0, np.zeros(len(a)), np.ones(len(a)))
        )
        information = jacobian.T @ jacobian
        info_eigen = np.linalg.eigvalsh(information)
        row.update(
            transform_available=True,
            transform=np.asarray(transform, dtype=float).tolist(),
            inliers=int(len(a)),
            span_x_a=float(np.ptp(a[:, 0])) if len(a) else 0.0,
            span_y_a=float(np.ptp(a[:, 1])) if len(a) else 0.0,
            span_x_b=float(np.ptp(b[:, 0])) if len(b) else 0.0,
            span_y_b=float(np.ptp(b[:, 1])) if len(b) else 0.0,
            grid4_a=self._grid_occupancy(a),
            grid4_b=self._grid_occupancy(b),
            point_condition=float(singular[0] / singular[-1])
            if len(singular) > 1 and singular[-1] > 0
            else math.inf,
            transform_information_condition=float(info_eigen[-1] / info_eigen[0])
            if info_eigen[0] > 0
            else math.inf,
        )
        self.rows.append(row)
        return transform, mask


def quantiles(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    return {
        "finite_count": int(len(finite)),
        "nonfinite_count": int(len(values) - len(finite)),
        "minimum": float(np.min(finite)),
        "q05": float(np.quantile(finite, 0.05)),
        "median": float(np.median(finite)),
        "q95": float(np.quantile(finite, 0.95)),
        "maximum": float(np.max(finite)),
    }


def main():
    manifest = json.loads((HERE.parent / "p5_spike_v3" / "manifest.json").read_text())
    immutable = json.loads((HERE.parent / "p5_spike_v3" / "result.json").read_text())
    recorder = GeometryRecorder(V3.V2.cv2)
    V3.V2.cv2 = recorder
    replay = V3.run(manifest)
    flat = []
    index = 0
    recorder_index = 0
    for pair in manifest["pairs"]:
        scored = replay["raw"][pair["stratum"]].pop(0)
        original = immutable["raw"][pair["stratum"]][
            sum(1 for p in manifest["pairs"][:index] if p["stratum"] == pair["stratum"])
        ]
        if scored.get("matches", 0) >= 4:
            geometry = recorder.rows[recorder_index]
            recorder_index += 1
        else:
            geometry = {"transform_available": False,
                        "candidate_correspondences": int(scored.get("matches", 0))}
        index += 1
        # Exact replay check guards against accidentally diagnosing different imagery.
        for key in ("keypoints_a", "keypoints_b", "matches", "inliers"):
            if scored.get(key) != original.get(key):
                raise RuntimeError(f"replay mismatch {pair['id']} {key}")
        dangerous = bool(
            pair["negative"] or pair["stratum"] in {"R", "F"}
            or original.get("translation_error_m", math.inf) > 0.50
            or original.get("yaw_error_deg", math.inf) > 5.0
            or original.get("scale_error_fraction", math.inf) > 0.10
        )
        if geometry.get("transform_available"):
            t = np.asarray(geometry["transform"])
            params = np.array([t[0, 0], t[1, 0], t[0, 2], t[1, 2]])
            centre, _, _ = V3.V2._centre_from_params(params, V3.V2._pose(pair["a"]))
            error = centre - np.array([pair["b"]["x"], pair["b"]["y"]])
            covariance = np.asarray(original.get("covariance_m2"), dtype=float)
            geometry["horizontal_nees"] = (
                float(error @ np.linalg.solve(covariance, error))
                if covariance.shape == (2, 2) and np.all(np.isfinite(covariance))
                and np.all(np.linalg.eigvalsh(covariance) > 0)
                else math.inf
            )
        flat.append({"id": pair["id"], "stratum": pair["stratum"], "negative": pair["negative"],
                     "dangerous": dangerous, **original, **geometry})

    if recorder_index != len(recorder.rows):
        raise RuntimeError("unconsumed geometry records")

    candidates = [r for r in flat if r.get("transform_available")]
    safe = [r for r in candidates if not r["dangerous"]]
    dangerous = [r for r in candidates if r["dangerous"]]
    fields = (
        "inliers", "hull_a", "hull_b", "span_x_a", "span_y_a", "span_x_b",
        "span_y_b", "grid4_a", "grid4_b", "point_condition",
        "transform_information_condition", "median_reprojection_px",
        "translation_error_m", "yaw_error_deg", "scale_error_fraction",
        "horizontal_nees",
    )
    report = {
        "purpose": "P5-v3 read-only development diagnosis; not a result replacement",
        "replayed_pairs": len(flat),
        "scatter_threshold_px2": 0.04 * 192**2,
        "equivalent_minor_axis_standard_deviation_px": math.sqrt(0.04 * 192**2),
        "uniform_full_image_axis_variance_px2": 192**2 / 12,
        "candidate_transforms": len(candidates),
        "safe_candidate_transforms": len(safe),
        "dangerous_candidate_transforms": len(dangerous),
        "safe_distributions": {k: quantiles([r[k] for r in safe]) for k in fields},
        "dangerous_distributions": {k: quantiles([r[k] for r in dangerous]) for k in fields},
        "rows": flat,
    }
    (HERE / "diagnosis_v3.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
