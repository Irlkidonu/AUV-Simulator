"""Sensor telemetry capture for M4.4-M4.7.

Subscribes to the four Gazebo sensors on ``bluerov2_phys`` and records every
message with its **simulation** timestamp, so measured rates and cross-modal
timing are properties of the simulation rather than of the host.

This is a *validation harness*: it is allowlisted to read privileged state,
because validating that the DVL reports the right sign requires knowing which
way the vehicle actually went. Nothing here produces an observation; it records
observations that Gazebo produced and compares them against truth. The
distinction is the whole point of ``privileged.py``.
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

__all__ = ["SensorLog", "capture", "MODEL"]

MODEL = "bluerov2_phys"
TOPICS = {"camera": "/bluerov2_phys/camera",
          "fls": "/bluerov2_phys/fls/raw",
          "imu": "/bluerov2_phys/imu",
          "dvl": "/bluerov2_phys/dvl"}
NOMINAL_HZ = {"camera": 20.0, "fls": 10.0, "imu": 100.0, "dvl": 10.0}


@dataclass
class SensorLog:
    """Per-sensor message records, plus the true pose track for comparison."""
    camera: list = field(default_factory=list)     # (t, HxWx3 uint8)
    fls: list = field(default_factory=list)        # (t, ranges, angle_min, angle_step)
    imu: list = field(default_factory=list)        # (t, gyro xyz, accel xyz, quat)
    dvl: list = field(default_factory=list)        # (t, velocity xyz, is_valid)
    pose: list = field(default_factory=list)       # (t, position, quaternion)
    meta: dict = field(default_factory=dict)

    def times(self, sensor: str) -> np.ndarray:
        return np.array([row[0] for row in getattr(self, sensor)], dtype=float)

    def measured_rate(self, sensor: str) -> float:
        t = self.times(sensor)
        if len(t) < 3:
            return float("nan")
        return float((len(t) - 1) / (t[-1] - t[0])) if t[-1] > t[0] else float("nan")

    def monotonic(self, sensor: str) -> bool:
        t = self.times(sensor)
        return bool(len(t) < 2 or np.all(np.diff(t) > 0))

    def duplicates(self, sensor: str) -> int:
        t = self.times(sensor)
        return int(len(t) - len(np.unique(t)))

    def summary(self) -> dict:
        return {name: {"messages": len(getattr(self, name)),
                       "measured_hz": round(self.measured_rate(name), 3),
                       "nominal_hz": NOMINAL_HZ[name],
                       "monotonic": self.monotonic(name),
                       "duplicate_stamps": self.duplicates(name)}
                for name in TOPICS}


def _stamp(header) -> float:
    return header.stamp.sec + header.stamp.nsec * 1e-9


def capture(duration_s: float = 12.0, *, settle_s: float = 4.0,
            thrust: dict[str, float] | None = None,
            thrust_at_s: float = 0.0,
            world: Path | None = None,
            spawn_override: dict | None = None) -> SensorLog:
    """Run the validated world and record all four sensor streams.

    ``thrust`` optionally applies a constant thruster vector at ``thrust_at_s``
    so a sensor can be validated against a known, commanded motion.
    """
    world_path = Path(world) if world else world_builder.VALIDATED_WORLD_PATH
    world_name = re.search(r'<world name="([^"]+)"',
                           world_path.read_text()).group(1)
    stack = toolchain.verify()

    env = dict(toolchain.environment())
    env["GZ_PARTITION"] = f"m4_{os.getpid()}_{int(time.time()*1e3)%100000}"
    for key, value in env.items():
        if key.startswith("GZ_"):
            os.environ[key] = value

    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    log = SensorLog()
    server = subprocess.Popen(
        [str(toolchain.GZ_EXECUTABLE), "sim", "-s", "-r", "-v", "0",
         str(world_path)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)

    def on_camera(msg: Image) -> None:
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * 3
        if frame.size >= expected:
            log.camera.append((_stamp(msg.header),
                               frame[:expected].reshape(msg.height, msg.width, 3)))

    def on_fls(msg: LaserScan) -> None:
        log.fls.append((_stamp(msg.header), np.array(msg.ranges, dtype=float),
                        msg.angle_min, msg.angle_step))

    def on_imu(msg: IMU) -> None:
        log.imu.append((
            _stamp(msg.header),
            np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z]),
            np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z]),
            np.array([msg.orientation.w, msg.orientation.x,
                      msg.orientation.y, msg.orientation.z])))

    def on_dvl(msg: DVLVelocityTracking) -> None:
        # `covariance` is a flat repeated scalar, not a submessage. Validity
        # comes from the tracking type: DVL_TYPE_BOTTOM (1) means bottom lock.
        velocity = msg.velocity.mean
        log.dvl.append((_stamp(msg.header),
                        np.array([velocity.x, velocity.y, velocity.z]),
                        int(msg.type)))

    def on_pose(msg: Pose_V) -> None:
        stamp = _stamp(msg.header)
        for pose in msg.pose:
            if pose.name == MODEL:
                log.pose.append((
                    stamp,
                    np.array([pose.position.x, pose.position.y, pose.position.z]),
                    np.array([pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z])))

    try:
        node = Node()
        deadline = time.time() + 60.0
        subscriptions = ((Image, TOPICS["camera"], on_camera),
                         (LaserScan, TOPICS["fls"], on_fls),
                         (IMU, TOPICS["imu"], on_imu),
                         (DVLVelocityTracking, TOPICS["dvl"], on_dvl),
                         (Pose_V, f"/world/{world_name}/dynamic_pose/info", on_pose))
        for message_type, topic, callback in subscriptions:
            while not node.subscribe(message_type, topic, callback):
                if time.time() > deadline:
                    raise RuntimeError(f"could not subscribe to {topic}")
                time.sleep(0.3)

        publishers = {joint: node.advertise(
            f"/model/{MODEL}/joint/{joint}/cmd_thrust", Double)
            for joint in ("prop_left_joint", "prop_right_joint",
                          "prop_sway_joint", "prop_vert_joint")}
        while not all(p.valid() for p in publishers.values()):
            if time.time() > deadline:
                raise RuntimeError("thruster publishers never became valid")
            time.sleep(0.2)

        while not log.pose:
            if time.time() > deadline:
                raise RuntimeError("no pose messages received")
            time.sleep(0.2)
        settle_start = log.pose[-1][0]
        while log.pose[-1][0] - settle_start < settle_s:
            time.sleep(0.05)

        zero = log.pose[-1][0]
        for bucket in ("camera", "fls", "imu", "dvl", "pose"):
            getattr(log, bucket).clear()

        applied = thrust is None
        while True:
            if not log.pose:
                time.sleep(0.02)
                continue
            elapsed = log.pose[-1][0] - zero
            if not applied and elapsed >= thrust_at_s:
                message = Double()
                for joint, newtons in thrust.items():
                    message.data = float(newtons)
                    publishers[joint].publish(message)
                applied = True
            if elapsed >= duration_s:
                break
            time.sleep(0.01)

        message = Double()
        message.data = 0.0
        for publisher in publishers.values():
            publisher.publish(message)
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:                    # pragma: no cover
            server.kill()

    # Re-base every stream onto the same zero so cross-modal timing is readable.
    for bucket in ("camera", "fls", "imu", "dvl", "pose"):
        rows = getattr(log, bucket)
        setattr(log, bucket, [(row[0] - zero,) + tuple(row[1:]) for row in rows])

    log.meta = {"world": str(world_path), "gz_sim_version": stack["gz_sim_version"],
                "duration_s": duration_s, "settle_s": settle_s,
                "thrust": thrust or {}}
    return log
