#!/usr/bin/env python3
"""Live terminal display of what the manager is doing and why.

The point of the demonstration is that a person can watch the system reason. A
plot of cross-track error after the fact does not show that; this does. It
redraws in place at 4 Hz and needs no GUI beyond a terminal.

Truth-side quantities are shown under a clearly separated heading, because they
are visible to the operator and to the evaluator but not to any decision.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32, String

BAR_WIDTH = 24

MODE_COLOUR = {
    "M0_NOMINAL": "\033[92m",
    "M1_OPTICAL_DEGRADED": "\033[93m",
    "M2_OPTICAL_LOST": "\033[93m",
    "M3_VELOCITY_AIDING_LOST": "\033[91m",
    "M4_DR_CRITICAL": "\033[1;91m",
    "M5_RECOVERY": "\033[96m",
}
RESET = "\033[0m"


def bar(value: float, lo: float = 0.0, hi: float = 1.0) -> str:
    frac = 0.0 if hi <= lo else max(0.0, min(1.0, (value - lo) / (hi - lo)))
    filled = int(round(frac * BAR_WIDTH))
    return "█" * filled + "·" * (BAR_WIDTH - filled)


class StatusDisplay(Node):
    def __init__(self) -> None:
        super().__init__("status_display")
        self._state = {
            "mode": "-", "channel": "-", "action": "-", "reason": "-",
            "quality": 0.0, "contrast": 0.0, "snr": 0.0,
            "altitude": 0.0, "cmd_altitude": 0.0, "cmd_speed": 0.0,
            "turbidity": 0.0, "bottom_lock": True, "optical": True,
            "acoustic_age": 0.0, "cov": 0.0, "error": 0.0, "waypoint": 0.0,
            "scenario": "-", "control": "auto",
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        }

        def sub(msg_type, topic, key, cast):
            self.create_subscription(
                msg_type, topic,
                lambda m, k=key, c=cast: self._state.__setitem__(k, c(m.data)),
                10,
            )

        sub(String, "/uuv/nav_mode", "mode", str)
        sub(String, "/uuv/optical_channel", "channel", str)
        sub(String, "/uuv/mission_action", "action", str)
        sub(String, "/uuv/decision_reason", "reason", str)
        sub(Float32, "/uuv/optical_quality", "quality", float)
        sub(Float32, "/uuv/optical_structure_contrast", "contrast", float)
        sub(Float32, "/uuv/optical_structure_to_noise", "snr", float)
        sub(Float32, "/uuv/altitude", "altitude", float)
        sub(Float32, "/uuv/commanded_altitude", "cmd_altitude", float)
        sub(Float32, "/uuv/commanded_speed", "cmd_speed", float)
        sub(Float32, "/uuv/turbidity_c", "turbidity", float)
        sub(Bool, "/uuv/dvl_bottom_lock", "bottom_lock", bool)
        sub(Bool, "/uuv/optical_available", "optical", bool)
        sub(Float32, "/uuv/acoustic_fix_age", "acoustic_age", float)
        sub(Float32, "/uuv/position_covariance_trace", "cov", float)
        sub(Float32, "/uuv/position_error", "error", float)
        sub(Float32, "/uuv/waypoint_index", "waypoint", float)
        sub(String, "/uuv/scenario_info", "scenario", str)
        sub(String, "/uuv/control_mode", "control", str)
        self.create_subscription(Vector3, "/uuv/attitude_rpy", self._on_rpy, 10)

        self.create_timer(0.25, self._draw)

    def _on_rpy(self, m) -> None:
        deg = 180.0 / 3.141592653589793
        self._state["roll"] = m.x * deg
        self._state["pitch"] = m.y * deg
        self._state["yaw"] = m.z * deg

    def _draw(self) -> None:
        s = self._state
        colour = MODE_COLOUR.get(s["mode"], "")
        ok = lambda flag: "\033[92m yes\033[0m" if flag else "\033[91m  NO\033[0m"

        print("\033[H\033[J", end="")
        print("┌─ MODE-AWARE ADAPTIVE NAVIGATION " + "─" * 44 + "┐")
        print(f"│ MODE   {colour}{s['mode']:<28}{RESET}"
              f" action: {s['action']:<26}│")
        print(f"│ why    {s['reason'][:66]:<66}│")
        print("├─ OPTICAL FEEDBACK (estimated from the camera image alone) " +
              "─" * 18 + "┤")
        print(f"│ quality      {bar(s['quality'])} {s['quality']:5.3f}"
              f"   structure/noise {s['snr']:8.2f}      │")
        print(f"│ contrast     {s['contrast']:6.4f}"
              f"                fix available: {ok(s['optical'])}          │")
        print("├─ SENSING AND MOTION " + "─" * 56 + "┤")
        print(f"│ channel      {s['channel']:<20}"
              f"commanded speed  {s['cmd_speed']:5.2f} m/s      │")
        print(f"│ altitude     {s['altitude']:5.2f} m  ->{s['cmd_altitude']:5.2f} m"
              f"      DVL bottom lock: {ok(s['bottom_lock'])}       │")
        print(f"│ acoustic fix age {s['acoustic_age']:6.1f} s"
              f"          waypoint {int(s['waypoint']):>2}                 │")
        print("├─ ESTIMATE " + "─" * 66 + "┤")
        # Under continuous aiding the trace reaches the 1e-5 range, which the
        # fixed-point format rendered as "0.0000" -- indistinguishable on screen
        # from an estimator that had stopped publishing. Small values switch to
        # exponent form so that a healthy filter does not look like a dead one.
        cov = f"{s['cov']:8.2e}" if 0.0 < s["cov"] < 1e-3 else f"{s['cov']:8.4f}"
        print(f"│ covariance trace {cov} m²"
              f"     {bar(s['cov'], 0.0, 4.0)}          │")
        # Who is flying, and which scenario is being replayed. Both belong on
        # screen because the same display is used hands-off and hands-on, and a
        # reading means something different depending on which is true.
        driver = ("\033[95mMANUAL\033[0m" if s["control"] == "manual"
                  else "\033[92mMANAGER\033[0m")
        print("├─ SESSION " + "─" * 67 + "┤")
        print(f"│ flown by     {driver}{'':<12}"
              f"scenario {s['scenario'].split('|')[0][:28]:<28}│")
        print(f"│ attitude     roll {s['roll']:7.1f}°   pitch {s['pitch']:7.1f}°"
              f"   yaw {s['yaw']:7.1f}°      │")
        print("├─ TRUTH (evaluator only; no decision reads this) " +
              "─" * 29 + "┤")
        print(f"│ position error {s['error']:6.3f} m"
              f"        water c = {s['turbidity']:4.2f} m⁻¹              │")
        print("└" + "─" * 77 + "┘")
        print("  keys: ros2 run uuv_mode_aware_navigation teleop"
              "     scenario: /uuv/set_scenario")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
