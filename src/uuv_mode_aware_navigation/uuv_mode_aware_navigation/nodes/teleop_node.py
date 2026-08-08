#!/usr/bin/env python3
"""Keyboard control for the demonstrator: fly the vehicle, break its sensors.

This is the interactive half of the simulation environment. The campaign runner
answers "which policy is better"; this answers "what does losing a Doppler log
in turbid water actually look like, and what does the manager do about it?"
Those are different questions and the second one is hard to get from a table.

    ros2 run uuv_mode_aware_navigation teleop

Flight, in the vehicle's own frame:

    arrow up / down     surge forward and back
    arrow left / right  yaw
    w / s               ascend and descend
    i / k               pitch: nose up and nose down
    z / x               roll: left and right
    space               all stop

Sensing, which is the point of the exercise:

    1 2 3               optical channel: coaxial camera, off-axis camera, laser
    l                   off-axis lighting: switches the lamp baseline
    t / T               turbidity down / up, in steps of 0.2 m^-1
    d                   Doppler velocity log: fail and restore
    a                   acoustic positioning: fail and restore
    v                   surface vessel: on station or departed
    o                   optical blackout: fail and restore
    m                   prior bathymetric map: available or absent

Control:

    e                   hand control back to the manager, or take it
    r                   reset the vehicle to the first waypoint
    q                   quit

Nothing here touches the campaign. The keys publish onto topics the vehicle and
sensor nodes already subscribe to, so an interactive session and a headless
campaign exercise the same physics, the same estimator and the same manager.
What differs is only who is steering and who decides when a sensor dies.
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

#: Surge and heave increments per key press, in m/s. Chosen so that a press is
#: visible in the rendered scene without overshooting the survey box.
SURGE_STEP = 0.1
HEAVE_STEP = 0.1
YAW_STEP = 0.3
PITCH_STEP = 0.25
ROLL_STEP = 0.25
MAX_RATE = 1.5
MAX_SURGE = 1.0
MAX_HEAVE = 0.4

CHANNELS = {"1": "camera_coaxial", "2": "camera_offaxis", "3": "lidar"}

BANNER = """
\033[96m╔══════════════════════════════════════════════════════════════════════════╗
║   MODE-AWARE ADAPTIVE NAVIGATION  ·  interactive demonstrator            ║
╚══════════════════════════════════════════════════════════════════════════╝\033[0m

  \033[93mFLIGHT\033[0m                          \033[93mSENSING\033[0m
   ↑ ↓   surge                     1 2 3   optical channel
   ← →   yaw                         l     off-axis lighting
   w s   ascend / descend           t T    turbidity  −  +
   i k   pitch up / down
   z x   roll left / right           d     Doppler log        fail / restore
   space stop
                                     a     acoustic fix       fail / restore
                                     o     optical blackout   fail / restore
  \033[93mCONTROL\033[0m                          v     surface vessel     here / gone
    e    manual / manager            m     prior map          have / none
    r    reset position
    q    quit
"""


class Teleop(Node):
    """Publishes flight commands and sensor faults from the keyboard."""

    def __init__(self) -> None:
        super().__init__("teleop")
        self._cmd = self.create_publisher(Twist, "/uuv/teleop_cmd", 10)
        self._mode = self.create_publisher(String, "/uuv/control_mode", 10)
        self._channel = self.create_publisher(String, "/uuv/force_channel", 10)
        self._turbidity = self.create_publisher(Float32, "/uuv/set_turbidity", 10)
        self._fault = self.create_publisher(String, "/uuv/inject_fault", 10)
        self._reset = self.create_publisher(Bool, "/uuv/reset", 10)

        self.surge = 0.0
        self.heave = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.manual = True
        self.turbidity = 0.2
        self.lighting_offaxis = True
        # Every fault starts healthy. The demonstrator is meant to be broken
        # deliberately by whoever is driving, not to start broken.
        self.faults = {"dvl": False, "acoustic": False, "vessel_gone": False,
                       "optical": False, "no_map": False}

        self.create_timer(0.1, self._publish)
        self._log = self.get_logger()

    # -- publishing ------------------------------------------------------
    def _publish(self) -> None:
        msg = Twist()
        msg.linear.x = self.surge
        msg.linear.z = self.heave
        msg.angular.x = self.roll
        msg.angular.y = self.pitch
        msg.angular.z = self.yaw
        self._cmd.publish(msg)

    def announce(self) -> None:
        self._mode.publish(String(data="manual" if self.manual else "auto"))
        self._turbidity.publish(Float32(data=float(self.turbidity)))
        for name, active in self.faults.items():
            self._fault.publish(String(data=f"{name}:{'on' if active else 'off'}"))

    # -- key handling ----------------------------------------------------
    def handle(self, key: str) -> bool:
        """Return False to quit."""
        if key == "q":
            return False

        if key == "\x1b[A":
            self.surge = min(self.surge + SURGE_STEP, MAX_SURGE)
        elif key == "\x1b[B":
            self.surge = max(self.surge - SURGE_STEP, -MAX_SURGE)
        elif key == "\x1b[D":
            self.yaw = max(self.yaw - YAW_STEP, -1.5)
        elif key == "\x1b[C":
            self.yaw = min(self.yaw + YAW_STEP, 1.5)
        elif key == "w":
            self.heave = min(self.heave + HEAVE_STEP, MAX_HEAVE)
        elif key == "s":
            self.heave = max(self.heave - HEAVE_STEP, -MAX_HEAVE)
        elif key == "i":
            self.pitch = min(self.pitch + PITCH_STEP, MAX_RATE)
        elif key == "k":
            self.pitch = max(self.pitch - PITCH_STEP, -MAX_RATE)
        elif key == "z":
            self.roll = max(self.roll - ROLL_STEP, -MAX_RATE)
        elif key == "x":
            self.roll = min(self.roll + ROLL_STEP, MAX_RATE)
        elif key == " ":
            self.surge = self.heave = self.yaw = 0.0
            self.pitch = self.roll = 0.0
            self._log.info("all stop")

        elif key in CHANNELS:
            self._channel.publish(String(data=CHANNELS[key]))
            self._log.info(f"optical channel -> {CHANNELS[key]}")
        elif key == "l":
            # Off-axis lighting is a lamp geometry, not a separate switch: it is
            # the difference between the coaxial configuration, whose lamp sits
            # 0.02 m from the lens and scatters the illuminated water column
            # straight back into it, and the off-axis one, whose 0.35 m baseline
            # moves the brightest scattering volume out of the field of view.
            # So this key selects between those two configurations.
            self.lighting_offaxis = not self.lighting_offaxis
            channel = "camera_offaxis" if self.lighting_offaxis else "camera_coaxial"
            self._channel.publish(String(data=channel))
            self._log.info(
                f"off-axis lighting {'on' if self.lighting_offaxis else 'off'}"
                f" -> {channel}"
            )
        elif key in ("t", "T"):
            step = 0.2 if key == "T" else -0.2
            self.turbidity = max(0.05, min(4.0, self.turbidity + step))
            self._turbidity.publish(Float32(data=float(self.turbidity)))
            self._log.info(f"turbidity c = {self.turbidity:.2f} 1/m")

        elif key in ("d", "a", "v", "m", "o"):
            name = {"d": "dvl", "a": "acoustic", "v": "vessel_gone",
                    "m": "no_map", "o": "optical"}[key]
            self.faults[name] = not self.faults[name]
            state = "on" if self.faults[name] else "off"
            self._fault.publish(String(data=f"{name}:{state}"))
            self._log.info(f"fault {name} -> {state}")

        elif key == "e":
            self.manual = not self.manual
            self._mode.publish(String(data="manual" if self.manual else "auto"))
            self._log.info(
                "you have control" if self.manual else "manager has control"
            )
        elif key == "r":
            self._reset.publish(Bool(data=True))
            self.surge = self.heave = self.yaw = 0.0
            self.pitch = self.roll = 0.0
            self._log.info("reset to first waypoint")
        return True


def _read_key(timeout: float = 0.1) -> str | None:
    """One keypress, including the three-byte arrow sequences."""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    if select.select([sys.stdin], [], [], 0.02)[0]:
        return ch + sys.stdin.read(2)
    return ch


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Teleop()
    print(BANNER)

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    node.announce()

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            key = _read_key()
            if key is not None and not node.handle(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.surge = node.heave = node.yaw = 0.0
        node.pitch = node.roll = 0.0
        node._publish()
        node.destroy_node()
        rclpy.shutdown()
        print("\n\033[96mdemonstrator stopped\033[0m")


if __name__ == "__main__":
    main()
