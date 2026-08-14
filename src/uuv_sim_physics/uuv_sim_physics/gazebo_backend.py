"""``DynamicsBackend`` over the Gazebo/DART plant.

Closes the loop that M1 left open: the same protocol the reduced backend
implements, now backed by rigid-body physics on the pinned stack.

Two properties inherited from the M2.5 findings are structural here, not
incidental:

* **Atomic actuation.** ``apply_thrust`` publishes the whole four-element vector
  from one call with nothing between the publishes. Splitting a body wrench
  across wall-clock time injects a yaw moment that exists in no equation of the
  model -- M2.5 measured 40 N at a 0.16 m lever spinning the vehicle up at
  7.4 rad/s^2 from a ~100 ms gap.
* **Body-frame state.** ``velocity`` is reported in the body frame, because a
  world-frame reading on a vehicle with no heading hold says more about where it
  happens to be pointing than about what it is doing.

Importing this module requires Gazebo. It is deliberately not re-exported from
``uuv_sim_physics.__init__``, so the headless install keeps working; the
dependency-isolation test enforces that.
"""

from __future__ import annotations

import math
import os
import re
import subprocess
import time
from pathlib import Path

import numpy as np

from . import toolchain, world_builder

__all__ = ["GazeboBackend", "MODEL"]

MODEL = "bluerov2_phys"
JOINTS = ("prop_left_joint", "prop_right_joint",
          "prop_sway_joint", "prop_vert_joint")


class GazeboBackend:
    """Rigid-body backend. Satisfies ``DynamicsBackend``, plus wrench actuation."""

    BACKEND_NAME = "gazebo_dart"
    MODELS = ("rigid_body_6dof", "buoyancy", "added_mass",
              "hydrodynamic_damping", "thruster_forces", "contact")
    DOES_NOT_MODEL = ("ambient_current",)

    def __init__(self, world: Path | None = None, *, settle_s: float = 3.0,
                 partition: str | None = None) -> None:
        self.world_path = Path(world) if world else world_builder.VALIDATED_WORLD_PATH
        match = re.search(r'<world name="([^"]+)"', self.world_path.read_text())
        self.world_name = match.group(1)
        self.stack = toolchain.verify()             # fails closed
        self._settle_s = settle_s
        self._partition = partition or f"m3_{os.getpid()}_{int(time.time()*1e3)%100000}"
        self._server: subprocess.Popen | None = None
        self._samples: list[tuple] = []
        self._zero = 0.0
        self.trace: list[dict] = []

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "GazeboBackend":
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        env = dict(toolchain.environment())
        env["GZ_PARTITION"] = self._partition
        for key, value in env.items():
            if key.startswith("GZ_"):
                os.environ[key] = value

        self._server = subprocess.Popen(
            [str(toolchain.GZ_EXECUTABLE), "sim", "-s", "-r", "-v", "0",
             str(self.world_path)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)

        def on_pose(msg: Pose_V) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            for pose in msg.pose:
                if pose.name == MODEL:
                    self._samples.append((
                        stamp,
                        (pose.position.x, pose.position.y, pose.position.z),
                        (pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z)))

        self._node = Node()
        self._Double = Double
        deadline = time.time() + 40.0
        topic = f"/world/{self.world_name}/dynamic_pose/info"
        while not self._node.subscribe(Pose_V, topic, on_pose):
            if time.time() > deadline:
                raise RuntimeError(f"could not subscribe to {topic}")
            time.sleep(0.4)

        self._publishers = {
            joint: self._node.advertise(
                f"/model/{MODEL}/joint/{joint}/cmd_thrust", Double)
            for joint in JOINTS}
        while not all(p.valid() for p in self._publishers.values()):
            if time.time() > deadline:
                raise RuntimeError("thruster publishers never became valid")
            time.sleep(0.2)

        while not self._samples:
            if time.time() > deadline:
                raise RuntimeError("no pose messages received")
            time.sleep(0.2)
        start = self._samples[-1][0]
        while self._samples[-1][0] - start < self._settle_s:
            time.sleep(0.05)
        self._zero = self._samples[-1][0]
        return self

    def close(self) -> None:
        if self._server is None:
            return
        self.apply_thrust({joint: 0.0 for joint in JOINTS})
        self._server.terminate()
        try:
            self._server.wait(timeout=10)
        except subprocess.TimeoutExpired:                    # pragma: no cover
            self._server.kill()
        self._server = None

    def __enter__(self) -> "GazeboBackend":
        return self.start()

    def __exit__(self, *_exc) -> None:
        self.close()

    # -- state -------------------------------------------------------------

    @property
    def time_s(self) -> float:
        return self._samples[-1][0] - self._zero

    @property
    def position(self) -> np.ndarray:
        return np.array(self._samples[-1][1], dtype=float)

    @property
    def quaternion(self) -> np.ndarray:
        return np.array(self._samples[-1][2], dtype=float)

    def _rotation(self) -> np.ndarray:
        w, x, y, z = self.quaternion
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    @property
    def yaw(self) -> float:
        w, x, y, z = self.quaternion
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _finite_difference(self, span_s: float = 0.12):
        """World velocity and yaw rate from the recent pose history."""
        if len(self._samples) < 3:
            return np.zeros(3), 0.0
        now = self._samples[-1][0]
        window = [s for s in self._samples if now - s[0] <= span_s]
        if len(window) < 3:
            window = self._samples[-3:]
        times = np.array([s[0] for s in window])
        positions = np.array([s[1] for s in window])
        if times[-1] - times[0] < 1e-6:
            return np.zeros(3), 0.0
        velocity = np.polyfit(times, positions, 1)[0]

        yaws = np.unwrap([math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                     1 - 2 * (q[2] ** 2 + q[3] ** 2))
                          for _, _, q in window])
        yaw_rate = float(np.polyfit(times, yaws, 1)[0])
        return velocity, yaw_rate

    @property
    def velocity(self) -> np.ndarray:
        """Body-frame velocity, per the protocol and the M2.5 finding."""
        world, _ = self._finite_difference()
        return self._rotation().T @ world

    @property
    def velocity_world(self) -> np.ndarray:
        return self._finite_difference()[0]

    @property
    def yaw_rate(self) -> float:
        return self._finite_difference()[1]

    @property
    def current(self) -> np.ndarray:
        # No ambient current in the plant; see M2.5 limitation L2.
        return np.zeros(3)

    @property
    def path_length_m(self) -> float:
        positions = np.array([s[1] for s in self._samples if s[0] >= self._zero])
        if len(positions) < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())

    # -- actuation ---------------------------------------------------------

    def apply_thrust(self, thrusts: dict[str, float]) -> None:
        """Publish the whole vector atomically. See the module docstring."""
        message = self._Double()
        for joint, newtons in thrusts.items():
            message.data = float(newtons)
            self._publishers[joint].publish(message)

    def step(self, commanded_velocity, dt: float) -> np.ndarray:
        """Protocol conformance.

        Present so ``GazeboBackend`` satisfies ``DynamicsBackend``, but a
        velocity command is not how this plant is driven -- it has no inner loop
        of its own. ``ClosedLoopRunner`` drives it through the controller and
        the allocator instead, which is the whole point of keeping those layers
        separate.
        """
        raise NotImplementedError(
            "GazeboBackend is actuated by wrench, not by commanded velocity; "
            "use ClosedLoopRunner (controller -> allocator -> apply_thrust)")

    def reset(self, position, current_mps=(0.0, 0.0, 0.0)) -> None:
        raise NotImplementedError(
            "restart the backend to reset the plant; in-place pose reset would "
            "bypass the solver and invalidate the dynamic state")

    def wait(self, seconds: float) -> None:
        """Block until ``seconds`` of *simulation* time have passed."""
        target = self._samples[-1][0] + seconds
        while self._samples[-1][0] < target:
            time.sleep(0.005)
