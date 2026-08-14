"""Measurement harness for the M2.5 physics validation protocol.

Three properties the M2 findings showed are non-negotiable:

**Atomic commands.** The thruster vector is computed in full and published from a
single process with no waiting in between. Issuing the surge pair through two
sequential ``gz topic -p`` invocations leaves ~100 ms of pure differential
thrust, which is 6.4 N.m on a 0.862 kg.m^2 yaw inertia -- enough to determine
the vehicle's heading for the rest of the run. Command timing must not be an
experimental variable.

**Body-frame measurement.** World-frame displacement is meaningless on a vehicle
with no heading hold. Every velocity here is rotated into the body frame, so a
result does not depend on where the vehicle happened to be pointing.

**Simulation time.** Wall clock is not the experiment's clock. Timestamps come
from the pose message header, so a slow host changes nothing.

Passive: it subscribes and publishes, and adds no plugin, sensor or body to the
world. The world under test is the world M3 will run.
"""

from __future__ import annotations

import os
import re
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .. import toolchain, world_builder

__all__ = ["Telemetry", "Command", "run", "MODEL"]

MODEL = "bluerov2_phys"

#: Thruster joints, in the canonical order used by every allocation vector here.
JOINTS = ("prop_left_joint", "prop_right_joint", "prop_sway_joint",
          "prop_vert_joint")


@dataclass(frozen=True)
class Command:
    """A thruster vector to apply, in Newtons, at a given simulation time."""
    at_s: float
    thrust_N: dict[str, float]


@dataclass
class Telemetry:
    t: np.ndarray                      # simulation seconds, from the msg header
    position: np.ndarray               # (N, 3) world
    quaternion: np.ndarray             # (N, 4) as w, x, y, z
    meta: dict = field(default_factory=dict)

    # -- derived ---------------------------------------------------------

    def rotation_matrices(self) -> np.ndarray:
        """(N, 3, 3) body->world rotation from each quaternion."""
        w, x, y, z = (self.quaternion[:, i] for i in range(4))
        return np.stack([
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
            np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
            np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
        ], axis=1)

    def world_velocity(self) -> np.ndarray:
        """Central-difference world velocity. Exact input, so no filtering."""
        return np.gradient(self.position, self.t, axis=0, edge_order=2)

    def body_velocity(self) -> np.ndarray:
        """World velocity expressed in the body frame: v_b = R^T v_w."""
        return np.einsum("nji,nj->ni", self.rotation_matrices(),
                         self.world_velocity())

    def euler_rpy(self) -> np.ndarray:
        """(N, 3) roll, pitch, yaw. Unwrapped, so a run through +/-pi is smooth."""
        w, x, y, z = (self.quaternion[:, i] for i in range(4))
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.stack([np.unwrap(roll), pitch, np.unwrap(yaw)], axis=-1)

    def body_rates(self) -> np.ndarray:
        """(N, 3) p, q, r by differentiating unwrapped Euler angles.

        Adequate here because every validation manoeuvre is single-axis and
        stays far from gimbal lock; a full quaternion-rate derivation would add
        nothing measurable.
        """
        return np.gradient(self.euler_rpy(), self.t, axis=0, edge_order=2)

    def window(self, start_s: float, stop_s: float) -> "Telemetry":
        mask = (self.t >= start_s) & (self.t <= stop_s)
        return Telemetry(self.t[mask], self.position[mask],
                         self.quaternion[mask], dict(self.meta))

    def steady_value(self, series: np.ndarray, last_s: float = 2.0) -> np.ndarray:
        """Mean of ``series`` over the final ``last_s`` seconds."""
        mask = self.t >= (self.t[-1] - last_s)
        return series[mask].mean(axis=0)


def _apply_pin() -> dict:
    env = toolchain.environment()
    for key, value in env.items():
        if key.startswith("GZ_"):
            os.environ[key] = value
    return env


def run(commands: list[Command] | None = None, *, duration_s: float = 20.0,
        settle_s: float = 4.0, world: Path | None = None,
        partition: str | None = None) -> Telemetry:
    """Execute one controlled run and return its telemetry.

    ``settle_s`` of simulation elapses before t = 0 so the vehicle starts from
    its own equilibrium rather than from the spawn transient. Command times are
    measured from that zero.
    """
    world_path = Path(world) if world else world_builder.WORLD_PATH
    # The topic namespace follows the <world name>, which differs between the
    # reference and validated worlds, so it is read from the file rather than
    # assumed.
    match = re.search(r'<world name="([^"]+)"', world_path.read_text())
    world_name = match.group(1) if match else world_builder.WORLD_NAME
    stack = toolchain.verify()                    # fails closed
    env = _apply_pin()
    env = dict(env)
    env["GZ_PARTITION"] = partition or f"m25_{os.getpid()}_{int(time.time()*1e3)%100000}"
    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]

    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    server = subprocess.Popen(
        [str(toolchain.GZ_EXECUTABLE), "sim", "-s", "-r", "-v", "0", str(world_path)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)

    samples: list[tuple[float, tuple, tuple]] = []

    def on_pose(msg: Pose_V) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for pose in msg.pose:
            if pose.name != MODEL:
                continue
            samples.append((
                stamp,
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.w, pose.orientation.x,
                 pose.orientation.y, pose.orientation.z)))

    try:
        node = Node()
        topic = f"/world/{world_name}/dynamic_pose/info"
        deadline = time.time() + 30.0
        while not node.subscribe(Pose_V, topic, on_pose):
            if time.time() > deadline:
                raise RuntimeError(f"could not subscribe to {topic}")
            time.sleep(0.5)

        publishers = {joint: node.advertise(
            f"/model/{MODEL}/joint/{joint}/cmd_thrust", Double) for joint in JOINTS}
        while not all(p.valid() for p in publishers.values()):
            if time.time() > deadline:
                raise RuntimeError("thruster publishers never became valid")
            time.sleep(0.2)

        # Wait for the stream, then let the vehicle settle.
        while not samples:
            if time.time() > deadline:
                raise RuntimeError("no pose messages received")
            time.sleep(0.2)
        settle_start = samples[-1][0]
        while samples[-1][0] - settle_start < settle_s:
            time.sleep(0.05)

        zero = samples[-1][0]
        pending = sorted(commands or [], key=lambda c: c.at_s)
        end = zero + duration_s

        while samples[-1][0] < end:
            now = samples[-1][0] - zero
            while pending and pending[0].at_s <= now:
                command = pending.pop(0)
                # Atomic: the whole vector leaves this loop with no sleep, no
                # subprocess, and no wall-clock wait between publishes.
                message = Double()
                for joint, newtons in command.thrust_N.items():
                    message.data = float(newtons)
                    publishers[joint].publish(message)
            time.sleep(0.01)
            if not samples:                                  # pragma: no cover
                raise RuntimeError("pose stream stopped")
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:                    # pragma: no cover
            server.kill()

    stamps = np.array([s[0] for s in samples], dtype=float) - zero
    keep = stamps >= -1e-9
    return Telemetry(
        t=stamps[keep],
        position=np.array([s[1] for s in samples], dtype=float)[keep],
        quaternion=np.array([s[2] for s in samples], dtype=float)[keep],
        meta={"world": str(world_path), "gz_sim_version": stack["gz_sim_version"],
              "settle_s": settle_s, "duration_s": duration_s,
              "sample_count": int(keep.sum()),
              "mean_rate_hz": float(keep.sum() / max(stamps[keep][-1], 1e-9))},
    )
