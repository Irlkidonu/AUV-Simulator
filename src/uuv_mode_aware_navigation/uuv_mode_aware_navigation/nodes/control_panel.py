#!/usr/bin/env python3
"""Graphical control panel: fly the vehicle, break its sensors, watch it think.

The keyboard teleop works but asks the driver to remember eighteen keys and
gives nothing back except log lines. This is the same set of controls as
buttons, with the vehicle's state beside them, so that what the manager is doing
can be watched while it is being interfered with.

    ros2 run uuv_mode_aware_navigation control_panel

Everything here publishes onto topics the vehicle and sensor nodes already
subscribe to. It holds no state of its own beyond what is on screen, adds no
privileged path, and writes no result file: a session cannot move a number in
the paper.
"""

from __future__ import annotations

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

# --- palette ---------------------------------------------------------------
BG = "#0b1418"
PANEL = "#12222a"
EDGE = "#1d3a45"
INK = "#d7e8ee"
DIM = "#6f8f9b"
CYAN = "#35d6f0"
AMBER = "#f0b429"
RED = "#f2545b"
GREEN = "#3ddc84"
VIOLET = "#b98cf5"

MODE_COLOUR = {
    "M0_NOMINAL": GREEN,
    "M1_OPTICAL_DEGRADED": AMBER,
    "M2_OPTICAL_LOST": AMBER,
    "M3_VELOCITY_AIDING_LOST": RED,
    "M4_DR_CRITICAL": RED,
    "M5_RECOVERY": CYAN,
}

CHANNELS = (("coaxial camera", "camera_coaxial"),
            ("off-axis camera", "camera_offaxis"),
            ("laser", "lidar"))

FAULTS = (("Doppler log", "dvl"),
          ("acoustic fix", "acoustic"),
          ("optical", "optical"),
          ("surface vessel", "vessel_gone"),
          ("prior map", "no_map"))

FAMILIES = [
    "E1_nominal", "E2_dvl_short", "E3_dvl_long", "E4_optical_graded",
    "E5_optical_loss", "E6_acoustic_intermittent", "E7_compound",
    "E8_turbid_dvl_loss", "E9_current_unobservable", "E10_current_steady",
    "E11_current_building", "E12_current_rotating", "E13_acoustic_noise",
    "E14_noisy_dvl_loss", "E15_turbid_and_noisy", "E16_featureless_plain",
    "E17_terrain_recoverable", "E18_vessel_departs", "E19_unprepared_area",
]


class PanelNode(Node):
    """Publishers and subscriptions. Owns no widgets."""

    def __init__(self) -> None:
        super().__init__("control_panel")
        self.cmd = self.create_publisher(Twist, "/uuv/teleop_cmd", 10)
        self.mode_pub = self.create_publisher(String, "/uuv/control_mode", 10)
        self.channel = self.create_publisher(String, "/uuv/force_channel", 10)
        self.turbidity = self.create_publisher(Float32, "/uuv/set_turbidity", 10)
        self.fault = self.create_publisher(String, "/uuv/inject_fault", 10)
        self.reset = self.create_publisher(Bool, "/uuv/reset", 10)
        self.scenario = self.create_publisher(String, "/uuv/set_scenario", 10)
        # Touching water or faults takes the schedule off the scenario director,
        # which would otherwise overwrite the change on its next tick.
        self.hold = self.create_publisher(Bool, "/uuv/scenario_hold", 10)
        self.current = self.create_publisher(Vector3, "/uuv/set_current", 10)
        # Six axes make it easy to end up tilted with no obvious way back.
        self.level = self.create_publisher(Bool, "/uuv/level_attitude", 10)

        self.state: dict = {
            "mode": "-", "reason": "-", "quality": 0.0, "altitude": 0.0,
            "error": 0.0, "cov": 0.0, "turbidity": 0.0, "scenario": "-",
            "lock": True, "optical": True, "fix_age": 0.0, "channel": "-",
            "rpy": (0.0, 0.0, 0.0), "running": True, "cur_speed": 0.0,
        }

        def sub(t, topic, key, cast=lambda v: v):
            self.create_subscription(
                t, topic,
                lambda m, k=key, c=cast: self.state.__setitem__(k, c(m.data)), 10)

        sub(String, "/uuv/nav_mode", "mode", str)
        sub(String, "/uuv/decision_reason", "reason", str)
        sub(String, "/uuv/optical_channel", "channel", str)
        sub(String, "/uuv/scenario_info", "scenario", str)
        sub(Float32, "/uuv/optical_quality", "quality", float)
        sub(Float32, "/uuv/altitude", "altitude", float)
        sub(Float32, "/uuv/position_error", "error", float)
        sub(Float32, "/uuv/position_covariance_trace", "cov", float)
        sub(Float32, "/uuv/turbidity_c", "turbidity", float)
        sub(Float32, "/uuv/acoustic_fix_age", "fix_age", float)
        sub(Bool, "/uuv/dvl_bottom_lock", "lock", bool)
        sub(Bool, "/uuv/optical_available", "optical", bool)
        sub(Bool, "/uuv/scenario_running", "running", bool)
        sub(Float32, "/uuv/current_speed", "cur_speed", float)
        self.create_subscription(
            Vector3, "/uuv/attitude_rpy",
            lambda m: self.state.__setitem__("rpy", (m.x, m.y, m.z)), 10)


class Panel:
    def __init__(self, node: PanelNode) -> None:
        self.n = node
        self.manual = False
        self.faults = {key: False for _, key in FAULTS}
        self._held: set[str] = set()
        self._release_jobs: dict[str, str] = {}
        self._pad_surge = self._pad_sway = self._pad_yaw = self._pad_heave = 0.0
        self._pad_pitch = self._pad_roll = 0.0

        self.root = tk.Tk()
        self.root.title("Mode-Aware Adaptive Navigation - control")
        self.root.configure(bg=BG)
        self.root.geometry("880x720")
        # Narrower than this and the two columns overlap; shorter is fine,
        # because the viewport scrolls.
        self.root.minsize(840, 320)

        self._build()
        self._bind_keys()
        self.root.after(50, self._tick)

    # -- construction ----------------------------------------------------
    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=EDGE, padx=1, pady=1)
        inner = tk.Frame(outer, bg=PANEL, padx=12, pady=10)
        inner.pack(fill="both", expand=True)
        tk.Label(inner, text=title.upper(), bg=PANEL, fg=DIM,
                 font=("DejaVu Sans", 8, "bold")).pack(anchor="w")
        return outer, inner

    def _viewport(self) -> tk.Frame:
        """Put the whole panel inside a scrolling viewport.

        Everything used to pack straight into the root window, so shrinking it
        did not rescale the layout -- it simply cut off whatever fell past the
        bottom edge. The turbidity slider is the last control in the left
        column, so it was the first thing to disappear, and a control you cannot
        see is a control you do not have.

        A minimum width goes with this. Vertical overflow can be scrolled to,
        but horizontal overflow cannot without a second scrollbar, so instead
        the window is not allowed to get narrower than the two columns need.
        """
        wrap = tk.Frame(self.root, bg=BG)
        wrap.pack(fill="both", expand=True)
        canvas = tk.Canvas(wrap, bg=BG, highlightthickness=0)
        bar = tk.Scrollbar(wrap, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        page = tk.Frame(canvas, bg=BG)
        window = canvas.create_window((0, 0), window=page, anchor="nw")
        # The page tracks the viewport's width so the cards still stretch, and
        # the scroll region tracks the page's height so the bar knows how far
        # there is to go.
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(window, width=e.width))
        page.bind("<Configure>",
                  lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        def wheel(delta: int) -> None:
            canvas.yview_scroll(delta, "units")

        # X11 reports the wheel as buttons 4 and 5; everywhere else it arrives
        # as <MouseWheel> with a signed delta.
        self.root.bind_all("<Button-4>", lambda e: wheel(-2))
        self.root.bind_all("<Button-5>", lambda e: wheel(2))
        self.root.bind_all(
            "<MouseWheel>",
            lambda e: wheel(-2 if e.delta > 0 else 2))
        return page

    def _build(self) -> None:
        page = self._viewport()
        tk.Label(page, text="MODE-AWARE ADAPTIVE NAVIGATION", bg=BG,
                 fg=CYAN, font=("DejaVu Sans", 15, "bold")).pack(pady=(14, 0))
        tk.Label(page, text="interactive demonstrator", bg=BG, fg=DIM,
                 font=("DejaVu Sans", 9)).pack()

        body = tk.Frame(page, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=12)
        left = tk.Frame(body, bg=BG); left.pack(side="left", fill="both",
                                                expand=True, padx=(0, 8))
        right = tk.Frame(body, bg=BG); right.pack(side="right", fill="both",
                                                  expand=True, padx=(8, 0))

        # --- who is flying ------------------------------------------------
        card, inner = self._card(left, "control")
        card.pack(fill="x", pady=(0, 10))
        self.control_btn = tk.Button(
            inner, text="TAKE MANUAL CONTROL", command=self._toggle_manual,
            bg=CYAN, fg=BG, font=("DejaVu Sans", 13, "bold"),
            activebackground=CYAN, relief="flat", pady=12, cursor="hand2")
        self.control_btn.pack(fill="x", pady=(8, 6))
        self.control_lbl = tk.Label(
            inner, text="the manager is flying", bg=PANEL, fg=DIM,
            font=("DejaVu Sans", 9))
        self.control_lbl.pack()

        # --- thruster -----------------------------------------------------
        card, inner = self._card(left, "thrusters")
        card.pack(fill="x", pady=(0, 10))
        tk.Label(inner, text="commanded surge", bg=PANEL, fg=INK,
                 font=("DejaVu Sans", 9)).pack(anchor="w", pady=(6, 0))
        self.speed = tk.Scale(
            inner, from_=0.0, to=1.0, resolution=0.05, orient="horizontal",
            bg=PANEL, fg=CYAN, troughcolor=BG, highlightthickness=0,
            font=("DejaVu Sans", 9), length=320)
        self.speed.set(0.4)
        self.speed.pack(fill="x")
        tk.Label(inner, text="m/s  ·  0.50 m/s = 1.8 km/h is the survey nominal",
                 bg=PANEL, fg=DIM, font=("DejaVu Sans", 8)).pack(anchor="w")

        pad = tk.Frame(inner, bg=PANEL); pad.pack(pady=8)
        # Translation only. The pad used to put yaw on its left and right
        # buttons, which is why pressing "left" turned the vehicle instead of
        # moving it sideways, and why the diagonals turned the wrong way.
        grid = [("\u2196", 1, 1), ("\u2191", 1, 0), ("\u2197", 1, -1),
                ("\u2190", 0, 1), ("STOP", 0, 0), ("\u2192", 0, -1),
                ("\u2199", -1, 1), ("\u2193", -1, 0), ("\u2198", -1, -1)]
        # NB: the loop variable is "port", not "left". "left" is the name of the
        # panel column this pad is being built into, and shadowing it here made
        # tkinter receive an int where it wanted a parent widget.
        for i, (label, fwd, port) in enumerate(grid):
            btn = tk.Button(pad, text=label, width=5, height=1,
                            bg=BG if label != "STOP" else RED,
                            fg=INK if label != "STOP" else BG,
                            relief="flat", font=("DejaVu Sans", 11, "bold"),
                            cursor="hand2")
            btn.grid(row=i // 3, column=i % 3, padx=3, pady=3)
            btn.bind("<ButtonPress-1>",
                     lambda e, f=fwd, l=port: self._drive(f, l))
            btn.bind("<ButtonRelease-1>", lambda e: self._drive(0, 0))

        row = tk.Frame(inner, bg=PANEL); row.pack(pady=(2, 6))
        for label, sign in (("\u21ba yaw left  q", 1), ("yaw right  e \u21bb", -1)):
            btn = tk.Button(row, text=label, bg=BG, fg=INK, relief="flat",
                            font=("DejaVu Sans", 9), width=14, cursor="hand2")
            btn.pack(side="left", padx=4)
            btn.bind("<ButtonPress-1>", lambda e, s=sign: self._yaw(s))
            btn.bind("<ButtonRelease-1>", lambda e: self._yaw(0))

        vert = tk.Frame(inner, bg=PANEL); vert.pack()
        for label, dz in (("ascend  r", 1), ("descend  f", -1)):
            btn = tk.Button(vert, text=label, bg=BG, fg=INK, relief="flat",
                            font=("DejaVu Sans", 9), width=12, cursor="hand2")
            btn.pack(side="left", padx=4)
            btn.bind("<ButtonPress-1>", lambda e, d=dz: self._heave(d))
            btn.bind("<ButtonRelease-1>", lambda e: self._heave(0))

        # Attitude. It aims the camera and the lamps; it does not steer, because
        # translation stays square to the vehicle whatever it is tilted to.
        tk.Label(inner, text="attitude  ·  aims the camera, does not steer",
                 bg=PANEL, fg=DIM, font=("DejaVu Sans", 8)).pack(anchor="w",
                                                                 pady=(8, 2))
        att = tk.Frame(inner, bg=PANEL); att.pack()
        for label, axis, sign in (("nose up  ↑", "pitch", -1),
                                  ("nose down  ↓", "pitch", 1),
                                  ("roll ←", "roll", -1),
                                  ("roll →", "roll", 1)):
            btn = tk.Button(att, text=label, bg=BG, fg=INK, relief="flat",
                            font=("DejaVu Sans", 9), width=11, cursor="hand2")
            btn.pack(side="left", padx=3)
            btn.bind("<ButtonPress-1>",
                     lambda e, a=axis, s=sign: self._attitude(a, s))
            btn.bind("<ButtonRelease-1>",
                     lambda e, a=axis: self._attitude(a, 0))
        tk.Button(inner, text="level the vehicle", bg=BG, fg=CYAN,
                  relief="flat", font=("DejaVu Sans", 8), cursor="hand2",
                  command=self._level).pack(anchor="w", pady=(4, 0))

        # --- sensing ------------------------------------------------------
        card, inner = self._card(left, "sensing")
        card.pack(fill="x")
        tk.Label(inner, text="optical channel", bg=PANEL, fg=INK,
                 font=("DejaVu Sans", 9)).pack(anchor="w", pady=(6, 2))
        row = tk.Frame(inner, bg=PANEL); row.pack(fill="x")
        for label, value in CHANNELS:
            tk.Button(row, text=label, bg=BG, fg=INK, relief="flat",
                      font=("DejaVu Sans", 8), cursor="hand2",
                      command=lambda v=value: self._channel(v)
                      ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(inner, text="return the channel to the manager", bg=PANEL,
                  fg=DIM, relief="flat", font=("DejaVu Sans", 8), cursor="hand2",
                  command=lambda: self._channel("")).pack(anchor="w", pady=(4, 8))

        tk.Label(inner, text="ocean current", bg=PANEL, fg=INK,
                 font=("DejaVu Sans", 9)).pack(anchor="w", pady=(8, 0))
        row = tk.Frame(inner, bg=PANEL); row.pack(fill="x")
        self.cur_speed = tk.Scale(
            row, from_=0.0, to=0.6, resolution=0.02, orient="horizontal",
            bg=PANEL, fg=GREEN, troughcolor=BG, highlightthickness=0,
            length=150, font=("DejaVu Sans", 8), label="speed m/s",
            command=self._set_current)
        self.cur_speed.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.cur_dir = tk.Scale(
            row, from_=-180, to=180, resolution=5, orient="horizontal",
            bg=PANEL, fg=GREEN, troughcolor=BG, highlightthickness=0,
            length=150, font=("DejaVu Sans", 8), label="bearing deg",
            command=self._set_current)
        self.cur_dir.pack(side="left", expand=True, fill="x")

        tk.Label(inner, text="turbidity  c", bg=PANEL, fg=INK,
                 font=("DejaVu Sans", 9)).pack(anchor="w", pady=(8, 0))
        self.turb = tk.Scale(inner, from_=0.05, to=4.0, resolution=0.05,
                             orient="horizontal", bg=PANEL, fg=AMBER,
                             troughcolor=BG, highlightthickness=0, length=320,
                             font=("DejaVu Sans", 9), command=self._turbidity)
        self.turb.set(0.2)
        self.turb.pack(fill="x")

        # --- faults -------------------------------------------------------
        card, inner = self._card(right, "break something")
        card.pack(fill="x", pady=(0, 10))
        self.fault_btns = {}
        for label, key in FAULTS:
            b = tk.Button(inner, text=f"  {label}", anchor="w", bg=BG, fg=GREEN,
                          relief="flat", font=("DejaVu Sans", 9), cursor="hand2",
                          command=lambda k=key: self._fault(k))
            b.pack(fill="x", pady=2)
            self.fault_btns[key] = b

        # --- scenario -----------------------------------------------------
        card, inner = self._card(right, "scenario")
        card.pack(fill="x", pady=(0, 10))
        self.family = ttk.Combobox(inner, values=FAMILIES, state="readonly",
                                   font=("DejaVu Sans", 9))
        self.family.set(FAMILIES[0])
        self.family.pack(fill="x", pady=(8, 6))
        row = tk.Frame(inner, bg=PANEL); row.pack(fill="x")
        tk.Button(row, text="load scenario", bg=CYAN, fg=BG, relief="flat",
                  font=("DejaVu Sans", 9, "bold"), cursor="hand2",
                  command=self._scenario).pack(side="left", expand=True,
                                               fill="x", padx=2)
        tk.Button(row, text="reset position", bg=BG, fg=INK, relief="flat",
                  font=("DejaVu Sans", 9), cursor="hand2",
                  command=self._reset).pack(side="left", expand=True,
                                            fill="x", padx=2)

        # --- telemetry ----------------------------------------------------
        card, inner = self._card(right, "what it believes")
        card.pack(fill="both", expand=True)
        self.mode_lbl = tk.Label(inner, text="-", bg=PANEL, fg=GREEN,
                                 font=("DejaVu Sans Mono", 12, "bold"))
        self.mode_lbl.pack(anchor="w", pady=(8, 0))
        self.reason_lbl = tk.Label(inner, text="", bg=PANEL, fg=DIM,
                                   font=("DejaVu Sans", 8), wraplength=330,
                                   justify="left")
        self.reason_lbl.pack(anchor="w", pady=(0, 8))
        self.rows = {}
        for key, label in (("quality", "optical quality"),
                           ("altitude", "altitude  (m)"),
                           ("cov", "covariance trace  (m²)"),
                           ("fix_age", "acoustic fix age  (s)"),
                           ("channel", "channel in use"),
                           ("current", "current  (m/s)"),
                           ("attitude", "roll / pitch / yaw"),
                           ("error", "position error  (m)  · truth")):
            r = tk.Frame(inner, bg=PANEL); r.pack(fill="x", pady=1)
            tk.Label(r, text=label, bg=PANEL, fg=DIM, width=22, anchor="w",
                     font=("DejaVu Sans", 8)).pack(side="left")
            v = tk.Label(r, text="-", bg=PANEL, fg=INK, anchor="e",
                         font=("DejaVu Sans Mono", 9))
            v.pack(side="right")
            self.rows[key] = v
        self.drive_lbl = tk.Label(inner, text="", bg=PANEL, fg=DIM,
                                  font=("DejaVu Sans", 8, "bold"))
        self.drive_lbl.pack(anchor="w", pady=(10, 0))
        self.scenario_lbl = tk.Label(inner, text="", bg=PANEL, fg=VIOLET,
                                     font=("DejaVu Sans", 8), wraplength=330,
                                     justify="left")
        self.scenario_lbl.pack(anchor="w")

        tk.Label(page,
                 text="w/s forward-back \u00b7 a/d strafe \u00b7 r/f up-down "
                      "\u00b7 q/e yaw \u00b7 \u2191\u2193 pitch "
                      "\u00b7 \u2190\u2192 roll \u00b7 space stop",
                 bg=BG, fg=DIM, font=("DejaVu Sans", 8)).pack(pady=(0, 10))

    # -- actions ---------------------------------------------------------
    #: Axis contributed by each key while it is held. Several may be held at
    #: once and their contributions add, so the vehicle flies diagonally,
    #: descends while turning, or banks into a climb -- which is how anything
    #: with six thrusters actually moves.
    #: Five degrees of freedom, mapped the way a quadcopter flies.
    #:
    #:   arrows  translate: forward, back, left, right. No turning. Ever.
    #:   w / s   ascend and descend
    #:   a / d   yaw left and right, on the spot
    #:   i / k   pitch the nose, which aims the camera and lamps
    #:
    #: Translation and rotation are on separate keys on purpose. Yaw used to sit
    #: on the left and right arrows, so sidestepping turned the vehicle and
    #: "forward" silently became a different direction a few seconds later.
    #: Six axes, on the layout a drone pilot already knows: the left hand
    #: translates, the arrows attitude. Translation and rotation never share a
    #: key, so sidestepping cannot turn the vehicle.
    #:
    #: Signs follow the frame the rest of the code uses, x forward, y to port,
    #: z up. Positive yaw therefore swings the nose from forward towards port,
    #: which is a turn to the LEFT; positive roll lifts the port side, which is
    #: a roll to starboard; and positive pitch drops the nose. Two of those read
    #: backwards from the key that commands them, hence the negative gains.
    KEYMAP = {
        "w":     ("surge",  1.0), "s":     ("surge", -1.0),
        "a":     ("sway",   1.0), "d":     ("sway",  -1.0),
        "r":     ("heave",  1.0), "f":     ("heave", -1.0),
        "q":     ("yaw",    1.0), "e":     ("yaw",   -1.0),
        "Up":    ("pitch", -1.0), "Down":  ("pitch",  1.0),
        "Left":  ("roll",  -1.0), "Right": ("roll",   1.0),
    }
    GAIN = {"surge": 1.0, "sway": 1.0, "yaw": 0.8,
            "heave": 0.30, "pitch": 0.4, "roll": 0.4}

    def _bind_keys(self) -> None:
        for key in self.KEYMAP:
            self.root.bind(f"<KeyPress-{key}>",
                           lambda e, k=key: self._key_down(k))
            self.root.bind(f"<KeyRelease-{key}>",
                           lambda e, k=key: self._key_up(k))
        self.root.bind("<space>", lambda e: self._all_stop())
        self.root.focus_set()

    def _key_down(self, key: str) -> None:
        # X11 auto-repeat fires KeyRelease immediately before each repeated
        # KeyPress. Taken at face value that reads as the key being released
        # thirty times a second, and holding two keys at once becomes
        # impossible. A pending release is therefore cancelled if the key comes
        # back within one repeat interval.
        pending = self._release_jobs.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self._held.add(key)
        self._ensure_manual()

    def _key_up(self, key: str) -> None:
        self._release_jobs[key] = self.root.after(
            60, lambda k=key: self._release(k))

    def _release(self, key: str) -> None:
        self._held.discard(key)
        self._release_jobs.pop(key, None)

    def _axes(self) -> dict:
        """Compose every held key into one command."""
        axes = {"surge": 0.0, "sway": 0.0, "yaw": 0.0,
                "heave": 0.0, "pitch": 0.0, "roll": 0.0}
        for key in self._held:
            if key not in self.KEYMAP:
                continue
            axis, sign = self.KEYMAP[key]
            axes[axis] += sign * self.GAIN[axis]
        for axis in ("yaw", "heave", "pitch", "roll"):
            axes[axis] = max(-1.5, min(1.5, axes[axis]))
        speed = float(self.speed.get())
        # Normalise the horizontal pair so a diagonal is not faster than a
        # straight run: the classic eight-way movement bug.
        hx = max(-1.0, min(1.0, axes["surge"]))
        hy = max(-1.0, min(1.0, axes["sway"]))
        mag = math.hypot(hx, hy)
        if mag > 1.0:
            hx, hy = hx / mag, hy / mag
        axes["surge"] = hx * speed
        axes["sway"] = hy * speed
        # Button-pad contributions, which coexist with the keyboard.
        axes["surge"] += self._pad_surge
        axes["sway"] += self._pad_sway
        axes["yaw"] += self._pad_yaw
        axes["heave"] += self._pad_heave
        axes["pitch"] += self._pad_pitch
        axes["roll"] += self._pad_roll
        return axes

    def _ensure_manual(self) -> None:
        if not self.manual:
            self._toggle_manual()

    # -- button pad (click and hold) --------------------------------------
    def _drive(self, fwd: int, left: int) -> None:
        self._ensure_manual()
        speed = float(self.speed.get())
        mag = math.hypot(fwd, left) or 1.0
        self._pad_surge = fwd / mag * speed
        self._pad_sway = left / mag * speed

    def _yaw(self, sign: int) -> None:
        self._ensure_manual()
        self._pad_yaw = sign * self.GAIN["yaw"]

    def _heave(self, d: int) -> None:
        self._ensure_manual()
        self._pad_heave = d * self.GAIN["heave"]

    def _attitude(self, axis: str, sign: int) -> None:
        self._ensure_manual()
        setattr(self, f"_pad_{axis}", sign * self.GAIN[axis])

    def _level(self) -> None:
        """Put roll and pitch back to zero without moving the vehicle."""
        self._pad_pitch = self._pad_roll = 0.0
        self.n.level.publish(Bool(data=True))

    def _all_stop(self) -> None:
        self._held.clear()
        for job in list(self._release_jobs.values()):
            self.root.after_cancel(job)
        self._release_jobs.clear()
        self._pad_surge = self._pad_sway = self._pad_yaw = self._pad_heave = 0.0
        self._pad_pitch = self._pad_roll = 0.0

    def _toggle_manual(self) -> None:
        self.manual = not self.manual
        self.n.mode_pub.publish(String(data="manual" if self.manual else "auto"))
        if self.manual:
            self.control_btn.configure(text="RETURN TO THE MANAGER", bg=VIOLET)
            self.control_lbl.configure(text="you are flying", fg=VIOLET)
        else:
            self.control_btn.configure(text="TAKE MANUAL CONTROL", bg=CYAN)
            self.control_lbl.configure(text="the manager is flying", fg=DIM)
            self._all_stop()

    def _channel(self, value: str) -> None:
        self.n.channel.publish(String(data=value))

    def _turbidity(self, value) -> None:
        # Claim the schedule first, then set the value. The other order loses:
        # the director's next tick lands between the two and puts its own
        # number back, which is exactly what made the slider look dead.
        self.n.hold.publish(Bool(data=True))
        self.n.turbidity.publish(Float32(data=float(value)))

    def _set_current(self, _value=None) -> None:
        """Set the flow the vehicle is swimming in.

        Bearing is where the water is going, in degrees about the vertical, so
        0 pushes the vehicle along +x. The current is not a fault and does not
        hold the scenario: several families drive it on a schedule, and a
        driver who wants to feel it can simply turn it up.
        """
        import math
        speed = float(self.cur_speed.get())
        bearing = math.radians(float(self.cur_dir.get()))
        v = Vector3()
        v.x = speed * math.cos(bearing)
        v.y = speed * math.sin(bearing)
        self.n.current.publish(v)

    def _fault(self, key: str) -> None:
        self.faults[key] = not self.faults[key]
        state = "on" if self.faults[key] else "off"
        self.n.hold.publish(Bool(data=True))
        self.n.fault.publish(String(data=f"{key}:{state}"))
        b = self.fault_btns[key]
        b.configure(fg=RED if self.faults[key] else GREEN,
                    text=("  " + b.cget("text").strip()))

    def _scenario(self) -> None:
        """Load a family. This also hands the schedule back to the director."""
        self.n.scenario.publish(String(data=self.family.get()))

    def _reset(self) -> None:
        self.n.reset.publish(Bool(data=True))

    # -- loop -------------------------------------------------------------
    def _tick(self) -> None:
        a = self._axes()
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = a["surge"], a["sway"], a["heave"]
        t.angular.x, t.angular.y, t.angular.z = a["roll"], a["pitch"], a["yaw"]
        self.n.cmd.publish(t)

        s = self.n.state
        mode = str(s["mode"])
        self.mode_lbl.configure(text=mode, fg=MODE_COLOUR.get(mode, INK))
        self.reason_lbl.configure(text=str(s["reason"])[:150])
        r, p, y = s["rpy"]
        deg = 57.29577951308232
        cov = s["cov"]
        vals = {
            "quality": f"{s['quality']:.3f}",
            "altitude": f"{s['altitude']:.2f}",
            "cov": f"{cov:.2e}" if 0 < cov < 1e-3 else f"{cov:.4f}",
            "fix_age": f"{s['fix_age']:.1f}",
            "channel": str(s["channel"]),
            "current": f"{s.get('cur_speed', 0.0):.2f}",
            "attitude": f"{r*deg:6.1f} {p*deg:6.1f} {y*deg:6.1f}",
            "error": f"{s['error']:.3f}",
        }
        for key, text in vals.items():
            self.rows[key].configure(text=text)
        self.rows["error"].configure(
            fg=RED if s["error"] > 1.0 else INK)
        if s["running"]:
            self.drive_lbl.configure(
                text="WATER: driven by the scenario schedule", fg=DIM)
        else:
            self.drive_lbl.configure(
                text="WATER: yours — scenario held", fg=VIOLET)
        self.scenario_lbl.configure(text=str(s["scenario"]).replace("|", "  ·  "))
        if not rclpy.ok():
            # The session has been shut down underneath us; close the window
            # rather than sit there publishing into nothing.
            self.root.quit()
            return
        self.root.after(50, self._tick)

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PanelNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    panel = Panel(node)
    try:
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the vehicle on the way out, but only if there is still a context
        # to publish into. When the launch system tears the session down it
        # destroys the context first, and an unguarded publish here raised on
        # every ordinary exit.
        try:
            if rclpy.ok():
                node.cmd.publish(Twist())
        except Exception:  # noqa: BLE001 - shutting down; nothing to recover
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
