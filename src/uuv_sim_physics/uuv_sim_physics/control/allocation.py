"""Body wrench -> thruster vector, for the validated four-thruster arrangement.

Kept as its own layer so the allocation can be inspected, tested and replaced
without touching the controller above it or the backend below it. The matrix is
derived from the thruster geometry in the configuration, not hard-coded, so a
geometry change cannot leave a stale allocation behind.

Controllable degrees of freedom: surge (X), sway (Y), heave (Z) and yaw (N).
Roll and pitch are *not* actuated -- they are left to the vehicle's own
restoring moment, which M2.5 P9 measured at a 1.579 s natural period.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["Wrench", "Allocator", "JOINT_ORDER"]

#: Canonical ordering. Every thrust vector in this package is in this order.
JOINT_ORDER = ("prop_left_joint", "prop_right_joint",
               "prop_sway_joint", "prop_vert_joint")


@dataclass(frozen=True)
class Wrench:
    """Desired force and moment in the body frame. X fwd, Y port, Z up."""
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mz: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.fx, self.fy, self.fz, self.mz], dtype=float)


class Allocator:
    """Maps a body wrench to four thrusts, with explicit saturation policy."""

    def __init__(self, config: dict) -> None:
        thrusters = config["vehicle_bluerov2_phys"]["thrusters"]
        units = {u["name"]: u for u in thrusters["units"]}
        self.limit = float(thrusters["common"]["max_thrust_cmd_N"])

        # Yaw lever of the surge pair. prop_left sits at +Y (port); M = r x F
        # gives Mz = -y*T, so the PORT unit produces NEGATIVE yaw. This is the
        # sign M2.5 P13 caught and the reason the lever is read from the
        # configuration rather than assumed.
        self.arm = float(units["prop_left"]["position_m"][1])       # +0.16

        # rows: [fx, fy, fz, mz]   cols: JOINT_ORDER
        self.B = np.array([
            [1.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [-self.arm, self.arm, 0.0, 0.0],
        ], dtype=float)

    # -- allocation --------------------------------------------------------

    def allocate(self, wrench: Wrench) -> np.ndarray:
        """Exact inverse of B for this geometry; the system is square."""
        fx, fy, fz, mz = wrench.as_array()
        yaw_thrust = mz / (2.0 * self.arm)
        return np.array([fx / 2.0 - yaw_thrust,     # prop_left
                         fx / 2.0 + yaw_thrust,     # prop_right
                         fy,                        # prop_sway
                         fz], dtype=float)          # prop_vert

    def saturate(self, thrusts: np.ndarray) -> tuple[np.ndarray, dict]:
        """Clamp to the actuator limit, preserving yaw before surge.

        Sway and heave are independent axes and simply clamp. The surge pair is
        shared between surge force and yaw moment, and when it saturates one of
        the two has to give. Yaw is preserved: a docking vehicle that loses
        heading authority stops being able to point at the dock at all, whereas
        losing some surge only makes the approach slower. The couple is
        retained and the common-mode component is reduced to fit.
        """
        out = thrusts.astype(float).copy()
        info = {"saturated": [], "surge_scaled": False}

        for index in (2, 3):
            if abs(out[index]) > self.limit:
                info["saturated"].append(JOINT_ORDER[index])
                out[index] = np.clip(out[index], -self.limit, self.limit)

        left, right = out[0], out[1]
        common = 0.5 * (left + right)               # surge component
        differential = 0.5 * (right - left)         # yaw component

        if abs(differential) > self.limit:
            differential = np.clip(differential, -self.limit, self.limit)
            info["saturated"].append("yaw_couple")

        headroom = self.limit - abs(differential)
        if abs(common) > headroom:
            common = np.clip(common, -headroom, headroom)
            info["surge_scaled"] = True
            info["saturated"].append("surge_common_mode")

        out[0] = common - differential
        out[1] = common + differential
        return out, info

    def __call__(self, wrench: Wrench) -> tuple[dict[str, float], dict]:
        thrusts, info = self.saturate(self.allocate(wrench))
        return dict(zip(JOINT_ORDER, thrusts.tolist())), info

    # -- inspection --------------------------------------------------------

    def achieved(self, thrusts: np.ndarray) -> Wrench:
        """Wrench actually produced by a thrust vector; for allocation tests."""
        fx, fy, fz, mz = self.B @ np.asarray(thrusts, dtype=float)
        return Wrench(fx, fy, fz, mz)
