#!/usr/bin/env python3
"""Execute the frozen P6-v2 TRN feasibility protocol exactly once."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from uuv_mode_aware_navigation.localization import TerrainMatcherV2
from uuv_mode_aware_navigation.maps import BathymetryMap
from uuv_mode_aware_navigation.sensor_models import AltimeterModel


IDENTIFIER = "p2v2_p6_trn_spike_v2"
SEED_ROOT = 22_220_000
NOISE_LEVELS_M = (0.02, 0.05, 0.10)
MAP_ERROR_LEVELS_M = (0.0, 0.02, 0.05)
HEADINGS_RAD = tuple(np.linspace(0.0, np.pi, 5, endpoint=False))
CALIBRATION_PER_STRATUM = 40
SCORING_PER_STRATUM = 100
CHI2 = {0.50: 1.3862943611198906, 0.90: 4.605170185988092,
        0.95: 5.991464547107979, 0.99: 9.210340371976184}


def _seed(label: str, index: int = 0) -> int:
    digest = hashlib.sha256(f"{SEED_ROOT}:{label}:{index}".encode()).digest()
    return int.from_bytes(digest[:8], "little")


class FlatMap:
    metres_per_cell = 0.10
    def sample(self, x, y):
        return np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape) - 20.0
    def gradient_vector(self, x, y):
        shape = np.broadcast(np.asarray(x), np.asarray(y)).shape
        return np.zeros(shape), np.zeros(shape)


@dataclass(frozen=True)
class PeriodicMap:
    crossed: bool
    phase_x: float
    phase_y: float
    metres_per_cell: float = 0.10

    def sample(self, x, y):
        x, y = np.asarray(x), np.asarray(y)
        value = 0.35 * np.sin(2.0 * np.pi * x + self.phase_x)
        if self.crossed:
            value = value + 0.25 * np.cos(2.0 * np.pi * y + self.phase_y)
        return -20.0 + value

    def gradient_vector(self, x, y):
        x, y = np.asarray(x), np.asarray(y)
        gx = 0.70 * np.pi * np.cos(2.0 * np.pi * x + self.phase_x)
        gy = np.zeros(np.broadcast(x, y).shape)
        if self.crossed:
            gy = -0.50 * np.pi * np.sin(2.0 * np.pi * y + self.phase_y)
        return gx, gy


@dataclass(frozen=True)
class NearPeriodicMap(PeriodicMap):
    def sample(self, x, y):
        x, y = np.asarray(x), np.asarray(y)
        return super().sample(x, y) + 0.08 * np.sin(0.37 * x + 0.23 * y) + 0.004 * (x*x + 0.7*y*y)

    def gradient_vector(self, x, y):
        step = self.metres_per_cell
        gx = (self.sample(np.asarray(x) + step, y) - self.sample(np.asarray(x) - step, y)) / (2.0 * step)
        gy = (self.sample(x, np.asarray(y) + step) - self.sample(x, np.asarray(y) - step)) / (2.0 * step)
        return gx, gy


@dataclass(frozen=True)
class PerturbedReference:
    base: object
    sigma_m: float
    phases: np.ndarray
    metres_per_cell: float = 0.10

    def _error(self, x, y):
        x, y = np.asarray(x), np.asarray(y)
        raw = (np.sin(2*np.pi*x/4.7 + self.phases[0])
               + np.cos(2*np.pi*y/5.9 + self.phases[1])
               + 0.7*np.sin(2*np.pi*(x+y)/7.3 + self.phases[2]))
        return self.sigma_m * raw / math.sqrt(2.49)

    def sample(self, x, y):
        return self.base.sample(x, y) + self._error(x, y)

    def gradient_vector(self, x, y):
        step = self.metres_per_cell
        gx = (self.sample(np.asarray(x)+step, y)-self.sample(np.asarray(x)-step, y))/(2*step)
        gy = (self.sample(x, np.asarray(y)+step)-self.sample(x, np.asarray(y)-step))/(2*step)
        return gx, gy


def _track(origin: np.ndarray, heading: float) -> np.ndarray:
    d = np.arange(49) * 0.25
    return origin + d[:, None] * np.array([math.cos(heading), math.sin(heading)])


def _initial(rng, truth, maximum=2.0):
    r = maximum * math.sqrt(float(rng.random()))
    a = float(rng.uniform(0, 2*np.pi))
    return truth + r * np.array([math.cos(a), math.sin(a)])


def _record(result, truth):
    row = {
        "success": bool(result.success), "reason": result.reason,
        "posterior_mass": float(result.posterior_mass),
        "hypothesis_delta_chi2": float(result.hypothesis_delta_chi2),
        "minimum_information_eigenvalue": float(result.minimum_information_eigenvalue),
        "coarse_basins": int(result.coarse_basins), "refined_basins": int(result.refined_basins),
        "window_disagreement_m": float(result.window_disagreement_m),
        "runtime_ms": float(result.runtime_ms),
    }
    if not result.success:
        row.update(error_m=None, nees=None, false_convergence=False)
        return row
    error = result.position_xy_m - truth
    nees = float(error @ np.linalg.solve(result.covariance_m2, error))
    row.update(
        error_m=float(np.linalg.norm(error)), nees=nees,
        covariance_eigenvalues_m2=np.linalg.eigvalsh(result.covariance_m2).tolist(),
        false_convergence=bool(np.linalg.norm(error) > 0.50 or nees > CHI2[0.99]),
    )
    return row


def _trial(terrain, reference, noise, truth, heading, initial, rng, matcher):
    profile = AltimeterModel(noise).sample_profile(
        terrain, _track(truth, heading), np.full(49, -17.0), rng
    )
    return _record(matcher.match(reference, profile, initial), truth)


def _binomial_cdf(k: int, n: int, p: float) -> float:
    return sum(math.comb(n, i) * p**i * (1-p)**(n-i) for i in range(k+1))


def _exact_interval(k: int, n: int, alpha: float = 0.05):
    if k == 0:
        lower = 0.0
    else:
        lo, hi = 0.0, 1.0
        for _ in range(70):
            mid = (lo + hi) / 2
            upper_tail = 1.0 - _binomial_cdf(k-1, n, mid)
            if upper_tail > alpha/2: hi = mid
            else: lo = mid
        lower = (lo + hi) / 2
    if k == n:
        upper = 1.0
    else:
        lo, hi = 0.0, 1.0
        for _ in range(70):
            mid = (lo + hi) / 2
            if _binomial_cdf(k, n, mid) > alpha/2: lo = mid
            else: hi = mid
        upper = (lo + hi) / 2
    return [lower, upper]


def _summary(rows):
    accepted = [r for r in rows if r["success"]]
    errors = np.asarray([r["error_m"] for r in accepted])
    false = sum(r["false_convergence"] for r in rows)
    coverages = {str(level): (sum(r["nees"] <= threshold for r in accepted) / len(accepted) if accepted else None)
                 for level, threshold in CHI2.items()}
    return {
        "total": len(rows), "successes": len(accepted), "fix_rate": len(accepted)/len(rows),
        "false_convergences": false, "false_convergence_rate": false/len(rows),
        "median_error_m": float(np.median(errors)) if len(errors) else None,
        "p95_error_m": float(np.percentile(errors,95)) if len(errors) else None,
        "maximum_error_m": float(np.max(errors)) if len(errors) else None,
        "coverage": coverages,
        "median_nees": float(np.median([r["nees"] for r in accepted])) if accepted else None,
        "median_runtime_ms": float(np.median([r["runtime_ms"] for r in rows])),
        "p95_runtime_ms": float(np.percentile([r["runtime_ms"] for r in rows],95)),
        "rejection_reasons": {reason: sum(r["reason"] == reason for r in rows) for reason in sorted({r["reason"] for r in rows})},
    }


def _informative_partition(label_prefix, count, covariance_scale=1.0):
    truth_map = BathymetryMap(metres_per_cell=0.10)
    output = {}
    for noise in NOISE_LEVELS_M:
        for map_error in MAP_ERROR_LEVELS_M:
            label = f"{label_prefix}_noise_{noise:.2f}_map_{map_error:.2f}"
            rng = np.random.default_rng(_seed(label))
            reference = PerturbedReference(truth_map, map_error, rng.uniform(0,2*np.pi,3))
            matcher = TerrainMatcherV2(map_sigma_m=map_error, covariance_scale=covariance_scale)
            rows=[]
            for i in range(count):
                truth=np.array([30.0,20.0])+rng.uniform(-1,1,2)
                rows.append(_trial(truth_map, reference, noise, truth,
                    HEADINGS_RAD[i % 5], _initial(rng, truth), rng, matcher))
            output[label]=rows
    return output


def build_manifest():
    strata = []
    for partition, count in (("calibration", CALIBRATION_PER_STRATUM),
                             ("informative", SCORING_PER_STRATUM)):
        for noise in NOISE_LEVELS_M:
            for map_error in MAP_ERROR_LEVELS_M:
                label = f"{partition}_noise_{noise:.2f}_map_{map_error:.2f}"
                strata.append({"label": label, "seed": _seed(label), "trials": count})
    for label, count in (("flat",100),("repeated_ridge",100),
                         ("repeated_crossed",100),("near_repeated",100),
                         ("truncated_search",100)):
        strata.append({"label": label, "seed": _seed(label), "trials": count})
    return {"identifier": IDENTIFIER, "seed_root": SEED_ROOT,
            "named_rng_streams": strata,
            "trial_identity": "label plus zero-based trial index"}


def run(manifest):
    if manifest != build_manifest():
        raise RuntimeError("pair/seed manifest does not match the frozen implementation")
    calibration = _informative_partition("calibration", CALIBRATION_PER_STRATUM)
    calibration_nees = [r["nees"] for rows in calibration.values() for r in rows if r["success"]]
    inflation = max(1.0, float(np.percentile(calibration_nees,95))/CHI2[0.95])
    scoring = _informative_partition("informative", SCORING_PER_STRATUM, inflation)

    special = {}
    flat=FlatMap(); rng=np.random.default_rng(_seed("flat")); rows=[]
    for i in range(100):
        truth=rng.uniform(-.4,.4,2)
        rows.append(_trial(flat,flat,.02,truth,HEADINGS_RAD[i%5],_initial(rng,truth),rng,TerrainMatcherV2(covariance_scale=inflation)))
    special["flat"]=rows

    for crossed in (False, True):
        label="repeated_crossed" if crossed else "repeated_ridge"
        rng=np.random.default_rng(_seed(label)); rows=[]
        for i in range(100):
            terrain=PeriodicMap(crossed,float(rng.uniform(0,2*np.pi)),float(rng.uniform(0,2*np.pi)))
            truth=rng.uniform(-.4,.4,2)
            rows.append(_trial(terrain,terrain,.02,truth,HEADINGS_RAD[i%5],_initial(rng,truth),rng,TerrainMatcherV2(covariance_scale=inflation)))
        special[label]=rows

    rng=np.random.default_rng(_seed("near_repeated")); rows=[]
    for i in range(100):
        terrain=NearPeriodicMap(True,float(rng.uniform(0,2*np.pi)),float(rng.uniform(0,2*np.pi)))
        truth=rng.uniform(-1,1,2)
        rows.append(_trial(terrain,terrain,.02,truth,HEADINGS_RAD[i%5],_initial(rng,truth),rng,TerrainMatcherV2(covariance_scale=inflation)))
    special["near_repeated"]=rows

    truth_map=BathymetryMap(metres_per_cell=.1); rng=np.random.default_rng(_seed("truncated_search")); rows=[]
    for i in range(100):
        truth=np.array([30.,20.])+rng.uniform(-1,1,2)
        angle=float(rng.uniform(0,2*np.pi)); initial=truth+3.2*np.array([math.cos(angle),math.sin(angle)])
        rows.append(_trial(truth_map,truth_map,.02,truth,HEADINGS_RAD[i%5],initial,rng,TerrainMatcherV2(covariance_scale=inflation)))
    special["truncated_search"]=rows

    all_scoring={**scoring,**special}; summaries={k:_summary(v) for k,v in all_scoring.items()}
    informative_rows=[r for rows in scoring.values() for r in rows]
    informative_accepted=[r for r in informative_rows if r["success"]]
    covered=sum(r["nees"] <= CHI2[.95] for r in informative_accepted)
    interval=_exact_interval(covered,len(informative_accepted)) if informative_accepted else [0,0]
    repeated=special["repeated_ridge"]+special["repeated_crossed"]
    repeated_rejected=sum(not r["success"] for r in repeated)
    reference=summaries["informative_noise_0.02_map_0.00"]
    all_rows=[r for rows in all_scoring.values() for r in rows]
    criteria={
      "repeated_zero_false_convergences":sum(r["false_convergence"] for r in repeated)==0,
      "repeated_rejection_at_least_0_95":repeated_rejected/len(repeated)>=.95,
      "flat_zero_accepted":summaries["flat"]["successes"]==0,
      "truncated_search_zero_accepted":summaries["truncated_search"]["successes"]==0,
      "near_repeated_fix_rate_at_least_0_50":summaries["near_repeated"]["fix_rate"]>=.50,
      "reference_fix_rate_at_least_0_85":reference["fix_rate"]>=.85,
      "reference_median_error_below_0_10":reference["median_error_m"] is not None and reference["median_error_m"]<.10,
      "reference_p95_error_below_0_25":reference["p95_error_m"] is not None and reference["p95_error_m"]<.25,
      "informative_false_convergence_below_0_01_each":all(v["false_convergence_rate"]<.01 for k,v in summaries.items() if k.startswith("informative_")),
      "covariance_finite_positive":all(all(np.isfinite(r["covariance_eigenvalues_m2"])) and min(r["covariance_eigenvalues_m2"])>0 for r in informative_accepted),
      "pooled_coverage_interval_contains_0_95":interval[0]<=.95<=interval[1],
      "pooled_median_nees_0_5_to_3_0":informative_accepted and .5<=float(np.median([r["nees"] for r in informative_accepted]))<=3.0,
      "runtime_median_below_100_ms":float(np.median([r["runtime_ms"] for r in all_rows]))<100,
      "runtime_p95_below_200_ms":float(np.percentile([r["runtime_ms"] for r in all_rows],95))<200,
    }
    return {"identifier":IDENTIFIER,"seed_root":SEED_ROOT,
      "status":"FEASIBILITY PASS" if all(criteria.values()) else "FAIL",
      "calibration":{"inflation":inflation,"accepted":len(calibration_nees),"q95_nees":float(np.percentile(calibration_nees,95)),"raw":calibration},
      "pooled_informative_95_coverage":{"covered":covered,"accepted":len(informative_accepted),"exact_95_interval":interval},
      "summaries":summaries,"criteria":{k:bool(v) for k,v in criteria.items()},"raw":all_scoring}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--manifest",type=Path)
    parser.add_argument("--prepare-manifest",action="store_true")
    args=parser.parse_args()
    if args.prepare_manifest:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(build_manifest(),indent=2,sort_keys=True)+"\n")
        return 0
    if args.manifest is None:
        parser.error("--manifest is required for execution")
    manifest=json.loads(args.manifest.read_text())
    started=time.time(); result=run(manifest); result["wall_time_s"]=time.time()-started
    args.output.parent.mkdir(parents=True,exist_ok=True); args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k not in {"raw","calibration"}},indent=2,sort_keys=True))
    return 0 if result["status"]=="FEASIBILITY PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
