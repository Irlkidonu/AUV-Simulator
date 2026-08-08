#!/usr/bin/env python3
"""Replays any of the campaign's nineteen scenario families in the demonstrator.

The campaign answers what happens across 380 runs. This answers what one of them
looks like. Selecting ``E7_compound`` here drives the same turbidity ramp and
opens the same fault windows the campaign's E7 runs used, in wall-clock time,
against a rendered scene.

The scenario definitions are not copied. They are imported from the campaign's
own ``scenario_family()``, so a family cannot drift between the table in the
paper and the thing a reader flies. This node only reads them.

What it is not
--------------

This is a demonstration, not a measurement. Wall-clock replay against a rendered
camera is not the headless campaign, the seed is not a campaign seed, and nothing
here writes a result file. No number in the paper comes from this node.

    ros2 run uuv_mode_aware_navigation scenario_director --ros-args -p scenario:=E7
    ros2 topic pub --once /uuv/set_scenario std_msgs/String "{data: E12}"
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

# The campaign runner installs to share/, not onto the module path, because it
# is a script rather than a library. Importing it is read-only: the definitions
# below are the campaign's, unmodified.
_SCRIPTS = Path(get_package_share_directory("uuv_mode_aware_navigation")) / "scripts"
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from run_campaign import scenario_family  # noqa: E402

from ..campaign import NoiseProfile, TerrainProfile, WaterProfile  # noqa: E402
from ..sensors import FaultKind  # noqa: E402

#: Injectable kinds to the fault names the vehicle node accepts. Kinds with no
#: entry are still applied by the scenario itself; they simply have no separate
#: keyboard toggle.
_FAULT_NAMES = {
    FaultKind.DVL_BOTTOM_LOCK_LOSS: "dvl",
    FaultKind.DVL_WATER_TRACK_LOSS: "dvl",
    FaultKind.ACOUSTIC_OUTAGE: "acoustic",
    FaultKind.OPTICAL_BLACKOUT: "optical",
    FaultKind.SURFACE_ASSET_LOSS: "vessel_gone",
}


class ScenarioDirector(Node):
    """Drives water, faults and terrain from a selected scenario family."""

    def __init__(self) -> None:
        super().__init__("scenario_director")
        self.declare_parameter("scenario", "E1_nominal")
        self.declare_parameter("period_s", 0.2)
        self.declare_parameter("duration_s", 180.0)

        self._families = {entry[0]: entry for entry in scenario_family()}

        self._turbidity = self.create_publisher(Float32, "/uuv/set_turbidity", 10)
        self._fault = self.create_publisher(String, "/uuv/inject_fault", 10)
        self._info = self.create_publisher(String, "/uuv/scenario_info", 10)
        self._reset = self.create_publisher(Bool, "/uuv/reset", 10)

        self.create_subscription(String, "/uuv/set_scenario", self._on_select, 10)

        # --- who is driving the water ---------------------------------------
        # The replay writes turbidity and fault state several times a second. If
        # it keeps doing that while somebody is flying by hand, every slider
        # they move is overwritten within one tick and the controls look broken:
        # the value changes and then springs back. So the replay yields.
        #
        # It holds while the vehicle is under manual control, and it holds as
        # soon as anyone sets water or faults directly. Loading a scenario is
        # the explicit way to take the schedule back.
        self._held = False
        self.create_subscription(
            String, "/uuv/control_mode",
            lambda m: self._set_hold(m.data == "manual", "manual control"), 10)
        self.create_subscription(
            Bool, "/uuv/scenario_hold",
            lambda m: self._set_hold(bool(m.data), "operator override"), 10)
        self._state = self.create_publisher(Bool, "/uuv/scenario_running", 10)

        self._t = 0.0
        self._active: dict[str, bool] = {}
        self._dt = float(self.get_parameter("period_s").value)
        self._duration = float(self.get_parameter("duration_s").value)
        self._select(str(self.get_parameter("scenario").value))
        self.create_timer(self._dt, self._step)

    def _set_hold(self, held: bool, why: str) -> None:
        if held == self._held:
            return
        self._held = held
        self.get_logger().info(
            f"scenario {'held: ' + why if held else 'resumed'}"
        )
        self._state.publish(Bool(data=not held))

    # -- selection -------------------------------------------------------
    def _resolve(self, name: str) -> str | None:
        """Accept a full family name or its ``E7``-style prefix."""
        if name in self._families:
            return name
        prefix = f"{name}_"
        for key in self._families:
            if key.startswith(prefix):
                return key
        return None

    def _on_select(self, msg: String) -> None:
        self._select(msg.data.strip())

    def _select(self, name: str) -> None:
        key = self._resolve(name)
        if key is None:
            self.get_logger().warn(
                f"no scenario {name!r}; known: {', '.join(sorted(self._families))}"
            )
            return

        entry = self._families[key]
        self._name = key
        self._water: WaterProfile = entry[1]
        self._schedule = entry[2]
        self._terrain: TerrainProfile = (
            entry[5] if len(entry) > 5 else TerrainProfile.constant(0.12)
        )
        self._prior_map = entry[6] if len(entry) > 6 else True
        self._noise: NoiseProfile = (
            entry[4] if len(entry) > 4 else NoiseProfile.constant(40.0)
        )
        self._t = 0.0

        # Clear whatever the previous scenario left asserted, so families are
        # not silently additive when one is selected after another.
        for name_ in set(_FAULT_NAMES.values()) | {"no_map"}:
            self._publish_fault(name_, False)
        self._publish_fault("no_map", not self._prior_map)
        self._reset.publish(Bool(data=True))
        # Loading a scenario is an explicit request for the schedule, so it
        # clears any hold that manual flying or a slider had put on it.
        self._held = False
        self._state.publish(Bool(data=True))

        window = ", ".join(
            f"{w.kind.value} at {w.start_s:.0f}s for {w.duration_s:.0f}s"
            for w in self._schedule.windows
        ) or "no injected faults"
        self.get_logger().info(f"scenario {key}: {window}")
        self._info.publish(String(data=f"{key}|{window}"))

    # -- replay ----------------------------------------------------------
    def _publish_fault(self, name: str, active: bool) -> None:
        if self._active.get(name) == active:
            return
        self._active[name] = active
        self._fault.publish(String(data=f"{name}:{'on' if active else 'off'}"))

    def _step(self) -> None:
        if self._held:
            # Time does not advance while held, so returning to autonomous
            # control resumes the scenario where it was rather than skipping
            # forward to wherever the wall clock has reached.
            self._info.publish(String(
                data=f"{self._name}|HELD at t={self._t:.0f}s|manual override"))
            return
        self._t += self._dt
        if self._t > self._duration:
            return

        self._turbidity.publish(Float32(data=float(self._water.at(self._t).c)))

        # A name is asserted if any kind mapped to it is in an open window, so
        # that "dvl" stays failed while either Doppler mode is out.
        wanted = {name: False for name in set(_FAULT_NAMES.values())}
        for kind, name in _FAULT_NAMES.items():
            if self._schedule.active(kind, self._t):
                wanted[name] = True
        for name, active in wanted.items():
            self._publish_fault(name, active)

        self._info.publish(String(
            data=f"{self._name}|t={self._t:.0f}s"
                 f"|c={self._water.at(self._t).c:.2f}"
                 f"|grad={self._terrain.at(self._t):.3f}"
                 f"|noise={self._noise.at(self._t).spectral_level_db:.0f}dB"
                 f"|map={'yes' if self._prior_map else 'no'}"
        ))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioDirector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what arrives when the launch system tears
        # the session down -- which is the normal way this node ends, since
        # closing Gazebo now shuts everything with it. Treating it as an error
        # printed a stack trace on every clean exit.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
