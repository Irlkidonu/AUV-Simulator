"""T1-T7: canonical closed-loop manoeuvre validation for M3.

These were not specified before M3; they are defined here. Each asks one
question about the control stack driving the *validated* plant, and none of
them is allowed to change the plant to get a better answer.

  T1  depth hold                 heave loop against a neutral vehicle
  T2  heading hold under surge   the L1 instability, closed-loop
  T3  straight-line track        cross-track error over a 6 m run
  T4  lateral offset correction  sway loop, 1 m step
  T5  yaw slew                   90 deg heading step, overshoot and settling
  T6  waypoint approach          guidance layer, approach toward the dock
  T7  repeatability              T3 three times; spread is the result

Mode is GROUND_TRUTH_CONTROL_VALIDATION throughout.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .. import world_builder
from ..gazebo_backend import GazeboBackend
from .controller import MODE, Gains, Reference
from .runner import ClosedLoopRunner

__all__ = ["Outcome", "run_all", "TESTS"]

SPAWN_YAW = math.pi          # vehicle faces the dock (world -X) at spawn
SPAWN_DEPTH = -15.0


@dataclass
class Outcome:
    name: str
    title: str
    status: str
    metrics: dict = field(default_factory=dict)
    detail: str = ""
    trace: object = None

    def line(self) -> str:
        mark = {"pass": "PASS", "fail": "FAIL"}[self.status]
        values = "  ".join(f"{k}={v:.4g}" for k, v in self.metrics.items()
                           if isinstance(v, (int, float)))
        return f"  {self.name:3s} {mark}  {self.title}\n        {values}"


def _session(duration: float = 0.0):
    return GazeboBackend()


def t1_depth_hold(**_) -> Outcome:
    target = SPAWN_DEPTH + 1.5
    with GazeboBackend() as backend:
        runner = ClosedLoopRunner(backend)
        trace = runner.run(Reference(depth_m=target, heading_rad=SPAWN_YAW), 30.0)
    depth = trace.position()[:, 2]
    settled = depth[trace.t >= trace.t[-1] - 5.0]
    error = float(np.mean(np.abs(settled - target)))
    overshoot = float(max(0.0, depth.max() - target))
    ok = error < 0.05
    return Outcome("T1", "depth hold (+1.5 m step)", "pass" if ok else "fail",
                   {"steady_error_m": error, "overshoot_m": overshoot,
                    "final_depth_m": float(depth[-1])},
                   f"target {target:.2f} m", trace)


def t2_heading_hold(**_) -> Outcome:
    """The L1 test. Open-loop this vehicle turns ~291 deg in 14 s."""
    with GazeboBackend() as backend:
        runner = ClosedLoopRunner(backend)
        trace = runner.run(Reference(surge_mps=0.4, heading_rad=SPAWN_YAW,
                                     depth_m=SPAWN_DEPTH), 30.0)
    yaw = np.unwrap(trace.array("yaw"))
    error = np.degrees(np.abs(yaw - SPAWN_YAW))
    ok = error.max() < 5.0
    return Outcome("T2", "heading hold under 0.4 m/s surge",
                   "pass" if ok else "fail",
                   {"max_heading_error_deg": float(error.max()),
                    "rms_heading_error_deg": float(np.sqrt((error ** 2).mean())),
                    "final_error_deg": float(error[-1])},
                   "open loop this plant turns ~291 deg in 14 s", trace)


def t3_straight_line(**_) -> Outcome:
    with GazeboBackend() as backend:
        runner = ClosedLoopRunner(backend)
        trace = runner.run(Reference(surge_mps=0.4, heading_rad=SPAWN_YAW,
                                     depth_m=SPAWN_DEPTH), 26.0)
    position = trace.position()
    cross = np.abs(position[:, 1] - position[0, 1])       # travel is along -X
    travelled = float(abs(position[-1, 0] - position[0, 0]))
    ok = cross.max() < 0.25 and travelled > 3.0
    return Outcome("T3", "straight-line track", "pass" if ok else "fail",
                   {"max_cross_track_m": float(cross.max()),
                    "final_cross_track_m": float(cross[-1]),
                    "distance_m": travelled},
                   "cross-track measured in world Y", trace)


def t4_lateral_offset(**_) -> Outcome:
    with GazeboBackend() as backend:
        runner = ClosedLoopRunner(backend)
        start_y = float(backend.position[1])
        target_y = start_y + 1.0

        def schedule(elapsed):
            return Reference(waypoint_xy=(float(backend.position[0]), target_y),
                             heading_rad=SPAWN_YAW, depth_m=SPAWN_DEPTH)

        trace = runner.run(Reference(), 30.0, schedule=schedule)
    y = trace.position()[:, 1]
    settled = y[trace.t >= trace.t[-1] - 5.0]
    error = float(np.mean(np.abs(settled - target_y)))
    ok = error < 0.15
    return Outcome("T4", "lateral offset correction (1 m)",
                   "pass" if ok else "fail",
                   {"steady_error_m": error, "final_y_m": float(y[-1]),
                    "target_y_m": target_y}, "", trace)


def t5_yaw_slew(**_) -> Outcome:
    target = SPAWN_YAW - math.pi / 2.0
    with GazeboBackend() as backend:
        runner = ClosedLoopRunner(backend)
        trace = runner.run(Reference(heading_rad=target, depth_m=SPAWN_DEPTH), 26.0)
    yaw = np.unwrap(trace.array("yaw"))
    start = yaw[0]
    span = target - start
    normalised = (yaw - start) / span if abs(span) > 1e-6 else yaw * 0
    overshoot = float(max(0.0, normalised.max() - 1.0) * 100.0)
    settled = np.degrees(np.abs(yaw[trace.t >= trace.t[-1] - 5.0] - target))
    reached = np.where(np.abs(normalised - 1.0) < 0.1)[0]
    rise = float(trace.t[reached[0]]) if len(reached) else float("nan")
    ok = settled.mean() < 3.0 and overshoot < 25.0
    return Outcome("T5", "yaw slew (-90 deg step)", "pass" if ok else "fail",
                   {"steady_error_deg": float(settled.mean()),
                    "overshoot_pct": overshoot, "rise_90_s": rise}, "", trace)


def t6_waypoint_approach(**_) -> Outcome:
    """Guidance layer: approach a hold point 2 m in front of the dock mouth."""
    target = (3.0, 0.0)
    with GazeboBackend() as backend:
        runner = ClosedLoopRunner(backend)
        trace = runner.run(Reference(waypoint_xy=target, heading_rad=SPAWN_YAW,
                                     depth_m=SPAWN_DEPTH), 34.0)
    position = trace.position()
    distance = np.linalg.norm(position[:, :2] - np.array(target), axis=1)
    ok = distance[-1] < 0.20
    return Outcome("T6", "waypoint approach", "pass" if ok else "fail",
                   {"final_distance_m": float(distance[-1]),
                    "closest_m": float(distance.min()),
                    "start_distance_m": float(distance[0])},
                   f"target {target}", trace)


def t7_repeatability(**_) -> Outcome:
    finals, crosses = [], []
    traces = []
    for _ in range(3):
        outcome = t3_straight_line()
        traces.append(outcome.trace)
        finals.append(outcome.metrics["distance_m"])
        crosses.append(outcome.metrics["max_cross_track_m"])
    spread = float(max(finals) - min(finals))
    ok = spread < 0.30
    return Outcome("T7", "repeatability (T3 x3)", "pass" if ok else "fail",
                   {"distance_spread_m": spread,
                    "distance_mean_m": float(np.mean(finals)),
                    "cross_track_max_m": float(max(crosses))},
                   f"distances {['%.3f' % d for d in finals]}", traces)


TESTS = {"T1": t1_depth_hold, "T2": t2_heading_hold, "T3": t3_straight_line,
         "T4": t4_lateral_offset, "T5": t5_yaw_slew, "T6": t6_waypoint_approach,
         "T7": t7_repeatability}


def run_all(names=None, output: Path | None = None) -> list[Outcome]:
    outcomes = []
    for name in (names or TESTS):
        try:
            outcome = TESTS[name]()
        except Exception as error:                           # noqa: BLE001
            import traceback
            traceback.print_exc()
            outcome = Outcome(name, name, "fail",
                              detail=f"{type(error).__name__}: {error}")
        outcomes.append(outcome)
        print(outcome.line(), flush=True)
        if outcome.detail:
            print(f"        {outcome.detail}", flush=True)
    if output:
        Path(output).write_text(json.dumps(
            [{"name": o.name, "title": o.title, "status": o.status,
              "metrics": o.metrics, "detail": o.detail, "mode": MODE}
             for o in outcomes], indent=2) + "\n")
    return outcomes


if __name__ == "__main__":
    import sys
    chosen = sys.argv[1:] or None
    results = run_all(chosen, Path("baselines/M3/maneuvers.json"))
    print(f"\n{sum(1 for r in results if r.status == 'pass')}/{len(results)} pass"
          f"   mode={MODE}")
