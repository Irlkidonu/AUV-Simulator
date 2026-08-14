"""M4.3-M4.7: quantitative validation of the integrated sensor suite.

Signs and frames are **measured**, not read off the SDF. Every claim here is
checked against the true motion the vehicle actually performed.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from ..sensors import WaterColumn, contrast_metrics, degrade
from . import sensor_harness as sh

__all__ = ["Check", "run_all"]

OUT = Path("baselines/M4")
THRUST = 40.0


@dataclass
class Check:
    name: str
    title: str
    status: str
    measured: dict = field(default_factory=dict)
    detail: str = ""

    def line(self) -> str:
        mark = {"pass": "PASS", "fail": "FAIL", "info": "INFO"}[self.status]
        values = "  ".join(f"{k}={v}" for k, v in self.measured.items())
        return f"  {self.name:22s} {mark}  {values}"


def _body_velocity(log) -> np.ndarray:
    """True body-frame velocity from the pose track, for comparison."""
    times = np.array([row[0] for row in log.pose])
    positions = np.array([row[1] for row in log.pose])
    quats = np.array([row[2] for row in log.pose])
    world = np.gradient(positions, times, axis=0, edge_order=2)
    w, x, y, z = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    rotations = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], axis=1)
    return np.einsum("nji,nj->ni", rotations, world)


def _late(values: np.ndarray, times: np.ndarray, last_s: float = 3.0):
    return values[times >= times[-1] - last_s]


# --- M4.5 timing -------------------------------------------------------------

def timing(log) -> list[Check]:
    checks = []
    for name, stats in log.summary().items():
        ratio = stats["measured_hz"] / stats["nominal_hz"]
        ok = (abs(ratio - 1.0) < 0.05 and stats["monotonic"]
              and stats["duplicate_stamps"] == 0)
        checks.append(Check(
            f"timing/{name}", f"{name} rate and timestamps",
            "pass" if ok else "fail",
            {"measured_hz": stats["measured_hz"],
             "nominal_hz": stats["nominal_hz"],
             "monotonic": stats["monotonic"],
             "duplicates": stats["duplicate_stamps"],
             "messages": stats["messages"]}))
    return checks


# --- M4.4 IMU ----------------------------------------------------------------

def imu_stationary(log) -> Check:
    times = log.times("imu")
    gyro = np.array([row[1] for row in log.imu])
    accel = np.array([row[2] for row in log.imu])
    steady_gyro = np.abs(_late(gyro, times)).max()
    steady_accel = _late(accel, times).mean(axis=0)
    # Specific force: a stationary sensor with +Z up reads +g on Z, because it
    # measures the reaction to gravity, not gravity itself.
    ok = steady_gyro < 0.02 and abs(steady_accel[2] - 9.8) < 0.15
    return Check("imu/stationary", "IMU at rest", "pass" if ok else "fail",
                 {"max_gyro_radps": round(float(steady_gyro), 5),
                  "accel_xyz": [round(float(v), 4) for v in steady_accel]},
                 "specific force: +Z reads +g for a stationary +Z-up sensor")


def imu_yaw_sign(log) -> Check:
    """Positive yaw thrust must give positive IMU gyro-z."""
    times = log.times("imu")
    gyro_z = np.array([row[1][2] for row in log.imu])
    measured = float(_late(gyro_z, times).mean())
    truth_t = np.array([row[0] for row in log.pose])
    quats = np.array([row[2] for row in log.pose])
    yaw = np.unwrap(np.arctan2(2 * (quats[:, 0] * quats[:, 3] + quats[:, 1] * quats[:, 2]),
                               1 - 2 * (quats[:, 2] ** 2 + quats[:, 3] ** 2)))
    truth = float(np.gradient(yaw, truth_t, edge_order=2)[truth_t >= truth_t[-1] - 3.0].mean())
    ok = measured > 0.05 and math.copysign(1, measured) == math.copysign(1, truth)
    return Check("imu/yaw_sign", "+yaw command -> +gyro_z",
                 "pass" if ok else "fail",
                 {"gyro_z_radps": round(measured, 4),
                  "true_yaw_rate_radps": round(truth, 4)})


# --- M4.4 DVL ----------------------------------------------------------------

def dvl_axis(log, axis: int, label: str, expect_sign: float = +1.0) -> Check:
    times = log.times("dvl")
    velocity = np.array([row[1] for row in log.dvl])
    measured = _late(velocity, times).mean(axis=0)
    truth = _body_velocity(log)
    truth_t = np.array([row[0] for row in log.pose])
    truth_late = _late(truth, truth_t).mean(axis=0)

    error = abs(measured[axis] - truth_late[axis])
    ok = (measured[axis] * expect_sign > 0.05 and error < 0.05
          and abs(measured[axis]) > 3 * max(abs(measured[(axis + 1) % 3]),
                                            abs(measured[(axis + 2) % 3])))
    sign = "+" if expect_sign > 0 else "-"
    return Check(f"dvl/{label}", f"{sign}{label} command -> {sign}DVL {'xyz'[axis]}",
                 "pass" if ok else "fail",
                 {"dvl_xyz": [round(float(v), 4) for v in measured],
                  "true_body_xyz": [round(float(v), 4) for v in truth_late],
                  "abs_error": round(float(error), 4)})


def dvl_stationary(log) -> Check:
    times = log.times("dvl")
    velocity = np.array([row[1] for row in log.dvl])
    steady = np.abs(_late(velocity, times)).max()
    types = {row[2] for row in log.dvl}
    ok = steady < 0.01
    return Check("dvl/stationary", "DVL at rest", "pass" if ok else "fail",
                 {"max_abs_mps": round(float(steady), 5),
                  "tracking_types": sorted(types)},
                 "type 1 = bottom lock")


# --- M4.4 FLS ----------------------------------------------------------------

def fls_geometry(log) -> Check:
    _, ranges, angle_min, angle_step = log.fls[0]
    span = angle_step * (len(ranges) - 1)
    finite = np.isfinite(ranges)
    ok = (len(ranges) == 128 and abs(angle_min + 0.52) < 1e-3
          and abs(span - 1.04) < 5e-3)
    return Check("fls/geometry", "FLS field and sampling",
                 "pass" if ok else "fail",
                 {"rays": len(ranges), "angle_min_rad": round(angle_min, 4),
                  "span_rad": round(float(span), 4),
                  "finite_returns": int(finite.sum()),
                  "min_range_m": round(float(np.nanmin(ranges[finite])), 3)
                  if finite.any() else None,
                  "max_range_m": round(float(np.nanmax(ranges[finite])), 3)
                  if finite.any() else None})


def fls_responds_to_range(log) -> Check:
    """Closing on the dock must shorten the central returns.

    Early and late samples of one approach, taken before contact: comparing two
    separate runs risks measuring a vehicle already seated in the funnel, where
    the central rays fall below the 0.2 m minimum and read as no return.
    """
    times = log.times("fls")

    def central_at(target_s):
        index = int(np.argmin(np.abs(times - target_s)))
        ranges = log.fls[index][1]
        window = ranges[len(ranges) // 2 - 8: len(ranges) // 2 + 8]
        finite = window[np.isfinite(window)]
        return float(finite.min()) if finite.size else float("nan")

    far_range = central_at(0.5)
    near_range = central_at(times[-1] - 0.5)
    ok = np.isfinite(near_range) and np.isfinite(far_range) and near_range < far_range
    return Check("fls/range_response", "FLS range tracks closure",
                 "pass" if ok else "fail",
                 {"closer_m": round(near_range, 3), "farther_m": round(far_range, 3)})


# --- M4.7 end-to-end optical -------------------------------------------------

def rendered_turbidity(log) -> list[Check]:
    """The decisive test: degradation applied to real rendered frames."""
    frame = log.camera[-1][1]
    severities = (0.0, 0.2, 0.6, 1.2, 2.0)
    metrics = [contrast_metrics(degrade(frame, WaterColumn(beam_attenuation=c)))
               for c in severities]

    checks = []
    for key in ("rms_contrast", "michelson_contrast", "edge_strength"):
        series = [round(m[key], 6) for m in metrics]
        monotone = all(a >= b - 1e-9 for a, b in zip(series, series[1:]))
        checks.append(Check(f"optical/{key}", f"{key} decreases with severity",
                            "pass" if monotone else "fail",
                            {"c_values": list(severities), "series": series}))

    raw_before = frame.copy()
    degrade(frame, WaterColumn(beam_attenuation=1.0))
    checks.append(Check("optical/raw_preserved", "camera/raw is not mutated",
                        "pass" if np.array_equal(frame, raw_before) else "fail",
                        {"identical": bool(np.array_equal(frame, raw_before))}))

    repeats = [degrade(frame, WaterColumn(beam_attenuation=0.9)) for _ in range(3)]
    deterministic = all(np.array_equal(repeats[0], other) for other in repeats[1:])
    checks.append(Check("optical/deterministic", "repeated processing identical",
                        "pass" if deterministic else "fail",
                        {"identical_over_3_runs": deterministic}))
    return checks


def fls_unaffected_by_optical(log) -> Check:
    """Architecture property, not a claim about real sonar."""
    _, ranges, _, _ = log.fls[-1]
    before = ranges.copy()
    degrade(log.camera[-1][1], WaterColumn(beam_attenuation=2.0))
    return Check("fls/optical_independence",
                 "FLS unchanged by the optical parameter",
                 "pass" if np.array_equal(before, ranges) else "fail",
                 {"identical": bool(np.array_equal(before, ranges))},
                 "simulator architecture property; NOT a claim that real "
                 "acoustic sensing is turbidity-invariant")


# --- M4.6 dock observability -------------------------------------------------

DOCK_POSES = {
    "centered/far":      [8.0, 0.0, -15.0, 0.0, 0.0, math.pi],
    "centered/mid":      [4.0, 0.0, -15.0, 0.0, 0.0, math.pi],
    "centered/near":     [2.0, 0.0, -15.0, 0.0, 0.0, math.pi],
    "lateral offset":    [4.0, 0.8, -15.0, 0.0, 0.0, math.pi],
    "vertical offset":   [4.0, 0.0, -14.4, 0.0, 0.0, math.pi],
    "yaw error 15 deg":  [4.0, 0.0, -15.0, 0.0, 0.0, math.pi - 0.262],
}


def dock_observability() -> list[Check]:
    """Is the dock observable where geometry says it should be?

    Not a perception benchmark. It asks only whether the simulated sensors
    return something structured when the dock is in front of the vehicle.
    """
    import tempfile
    from . import protocol

    checks = []
    for label, pose in DOCK_POSES.items():
        with tempfile.TemporaryDirectory() as tmp:
            world = protocol.variant_world(
                {"vehicle_bluerov2_phys": {"spawn_pose": pose}},
                Path(tmp), validated=True)
            log = sh.capture(duration_s=3.0, settle_s=3.0, world=world)

        _, ranges, _, _ = log.fls[-1]
        finite = np.isfinite(ranges)
        frame = log.camera[-1][1]
        metrics = contrast_metrics(frame)
        # A dock in view produces structured returns well inside the 12 m
        # ceiling, and a rendered frame with more contrast than flat water.
        fls_sees = bool(finite.sum() >= 4 and np.nanmin(ranges[finite]) < 11.0)
        camera_sees = metrics["rms_contrast"] > 0.01
        checks.append(Check(
            f"observability/{label}", "dock observable",
            "pass" if (fls_sees and camera_sees) else "fail",
            {"fls_returns": int(finite.sum()),
             "fls_min_m": round(float(np.nanmin(ranges[finite])), 3)
             if finite.any() else None,
             "cam_rms_contrast": round(metrics["rms_contrast"], 4),
             "cam_edges": round(metrics["edge_strength"], 5)}))
    return checks


# --- driver ------------------------------------------------------------------

def run_all() -> list[Check]:
    checks: list[Check] = []
    print("capturing: stationary", flush=True)
    rest = sh.capture(duration_s=12.0, settle_s=4.0)
    checks += timing(rest)
    checks.append(imu_stationary(rest))
    checks.append(dvl_stationary(rest))
    checks.append(fls_geometry(rest))
    checks += rendered_turbidity(rest)
    checks.append(fls_unaffected_by_optical(rest))

    for label, axis, thrust in (
            # Surge is driven ASTERN: the dock now has collision geometry, so a
            # forward run of this length ends pressed against it and the
            # measurement window would read a stationary vehicle.
            ("surge", 0, {"prop_left_joint": -THRUST, "prop_right_joint": -THRUST}),
            ("sway", 1, {"prop_sway_joint": THRUST}),
            ("heave", 2, {"prop_vert_joint": THRUST})):
        print(f"capturing: {label}", flush=True)
        log = sh.capture(duration_s=14.0, settle_s=4.0, thrust=thrust)
        checks.append(dvl_axis(log, axis, label,
                               expect_sign=-1.0 if label == "surge" else +1.0))

    print("capturing: yaw", flush=True)
    yaw_log = sh.capture(duration_s=14.0, settle_s=4.0,
                         thrust={"prop_left_joint": -THRUST,
                                 "prop_right_joint": THRUST})
    checks.append(imu_yaw_sign(yaw_log))

    print("capturing: closure for FLS range response", flush=True)
    closure = sh.capture(duration_s=6.0, settle_s=4.0,
                         thrust={"prop_left_joint": 16.0, "prop_right_joint": 16.0})
    checks.append(fls_responds_to_range(closure))

    print("capturing: dock observability (6 poses)", flush=True)
    checks += dock_observability()

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "sensor_validation.json").write_text(
        json.dumps([asdict(c) for c in checks], indent=2, default=str) + "\n")
    return checks


if __name__ == "__main__":
    results = run_all()
    print()
    for check in results:
        print(check.line())
        if check.detail:
            print(f"        {check.detail}")
    passed = sum(1 for c in results if c.status == "pass")
    print(f"\n{passed}/{len(results)} pass")
