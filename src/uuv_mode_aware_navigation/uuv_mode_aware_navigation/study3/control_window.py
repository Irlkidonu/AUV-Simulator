"""Tk control window for the non-campaign real-time Study-3 demonstrator."""
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .interactive import run_interactive_session, save_recording
from .policies import PolicyKind


BG = "#081319"
PANEL = "#10242d"
EDGE = "#1c3c48"
INK = "#d9edf2"
DIM = "#7899a5"
CYAN = "#38d5ec"
GREEN = "#48dc8b"
AMBER = "#f2b84b"
RED = "#ff6570"


class Study3ControlWindow:
    def __init__(self, *, seed=31_895_000, horizon_s=900., dt_s=1.):
        self.seed = int(seed)
        self.horizon_s = float(horizon_s)
        self.dt_s = float(dt_s)
        self.environment = None
        self.worker = None
        self.replay_path = None
        self.record_armed = False
        self.messages = queue.Queue()
        self.truth = deque(maxlen=1200)
        self.estimate = deque(maxlen=1200)
        self.mode_events = deque(maxlen=10)
        self.last_packet = None

        self.root = tk.Tk()
        self.root.title("Study 3 · interactive mode-aware navigation")
        self.root.geometry("1320x850")
        self.root.minsize(1050, 700)
        self.root.configure(bg=BG)
        self._style()
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(50, self._poll)

    def _style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL, foreground=INK, padding=(10, 6))
        style.map("TNotebook.Tab", background=[("selected", EDGE)])

    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=EDGE, padx=1, pady=1)
        inner = tk.Frame(outer, bg=PANEL, padx=10, pady=8)
        inner.pack(fill="both", expand=True)
        tk.Label(inner, text=title.upper(), bg=PANEL, fg=DIM,
                 font=("DejaVu Sans", 8, "bold")).pack(anchor="w")
        return outer, inner

    def _build(self):
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=14, pady=(12, 6))
        tk.Label(header, text="STUDY 3 LIVE ENVIRONMENT LAB", bg=BG, fg=CYAN,
                 font=("DejaVu Sans", 16, "bold")).pack(side="left")
        self.status = tk.Label(header, text="READY", bg=BG, fg=DIM,
                               font=("DejaVu Sans", 10, "bold"))
        self.status.pack(side="right")

        toolbar = tk.Frame(self.root, bg=BG)
        toolbar.pack(fill="x", padx=14, pady=(0, 8))
        self.policy_var = tk.StringVar(value="reactive")
        ttk.Combobox(toolbar, textvariable=self.policy_var, state="readonly", width=18,
                     values=("deployment_fixed", "reactive", "predictive")).pack(side="left", padx=(0, 6))
        self.start_btn = self._button(toolbar, "START", self._start, GREEN)
        self.pause_btn = self._button(toolbar, "PAUSE", self._pause, AMBER)
        self._button(toolbar, "RESET", self._reset, RED)
        self.record_btn = self._button(toolbar, "RECORD", self._record, CYAN)
        self._button(toolbar, "REPLAY…", self._replay, CYAN)
        tk.Label(toolbar, text="speed", bg=BG, fg=DIM).pack(side="left", padx=(16, 2))
        self.rate = tk.DoubleVar(value=1.0)
        rate = ttk.Combobox(toolbar, textvariable=self.rate, state="readonly", width=5,
                            values=(0.5, 1.0, 2.0, 4.0))
        rate.pack(side="left")
        rate.bind("<<ComboboxSelected>>", lambda _e: self.environment and
                  self.environment.set_realtime_factor(self.rate.get()))

        body = tk.PanedWindow(self.root, orient="horizontal", bg=BG,
                              sashwidth=6, sashrelief="flat")
        body.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        controls = tk.Frame(body, bg=BG, width=440)
        display = tk.Frame(body, bg=BG)
        body.add(controls, minsize=390)
        body.add(display, minsize=590)
        self._build_controls(controls)
        self._build_display(display)

    def _button(self, parent, text, command, colour):
        button = tk.Button(parent, text=text, command=command, bg=colour, fg=BG,
                           activebackground=colour, relief="flat", padx=14, pady=6,
                           font=("DejaVu Sans", 9, "bold"), cursor="hand2")
        button.pack(side="left", padx=3)
        return button

    def _scale(self, parent, label, control, minimum, maximum, resolution, initial):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=3)
        tk.Label(row, text=label, bg=PANEL, fg=INK, width=25, anchor="w",
                 font=("DejaVu Sans", 8)).pack(side="left")
        value = tk.DoubleVar(value=initial)
        scale = tk.Scale(row, from_=minimum, to=maximum, resolution=resolution,
                         variable=value, orient="horizontal", showvalue=True,
                         bg=PANEL, fg=CYAN, troughcolor=BG, highlightthickness=0,
                         length=180, command=lambda v, c=control: self._set(c, float(v)))
        scale.pack(side="right", fill="x", expand=True)
        return value

    def _toggle(self, parent, label, control, initial=False):
        value = tk.BooleanVar(value=initial)
        box = tk.Checkbutton(parent, text=label, variable=value, bg=PANEL, fg=INK,
                             selectcolor=BG, activebackground=PANEL,
                             activeforeground=INK, anchor="w",
                             command=lambda c=control, v=value: self._set(c, v.get()))
        box.pack(fill="x", pady=2)
        return value

    def _build_controls(self, parent):
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        scroll = tk.Scrollbar(parent, command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        page = tk.Frame(canvas, bg=BG)
        window = canvas.create_window((0, 0), window=page, anchor="nw")
        page.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))

        card, optical = self._card(page, "optical water column")
        card.pack(fill="x", pady=(0, 8))
        self.turbidity = self._scale(optical, "turbidity / visibility loss", "turbidity", 0, 1, .02, .10)
        self.optical_failure = self._toggle(optical, "optical sensor failure", "optical_failure")

        card, dvl = self._card(page, "DVL")
        card.pack(fill="x", pady=8)
        self.bottom = self._scale(dvl, "bottom-lock health", "dvl_bottom_probability", .01, 1, .01, .98)
        self.water = self._scale(dvl, "water-track health", "dvl_water_probability", .01, 1, .01, .90)
        self.dvl_noise = self._scale(dvl, "DVL noise multiplier", "dvl_noise_scale", 1, 8, .25, 1)
        self.dvl_crash = self._toggle(dvl, "DVL crashout", "dvl_crashout")

        card, ocean = self._card(page, "current")
        card.pack(fill="x", pady=8)
        self.current_e = self._scale(ocean, "east current (m/s)", "current_east_mps", -.5, .5, .01, 0)
        self.current_n = self._scale(ocean, "north current (m/s)", "current_north_mps", -.5, .5, .01, 0)

        card, acoustic = self._card(page, "acoustic services")
        card.pack(fill="x", pady=8)
        self.acoustic_noise = self._scale(acoustic, "ambient noise (dB)", "acoustic_noise_db", 35, 95, 1, 48)
        self.lbl_geometry = self._scale(acoustic, "LBL geometry scale", "lbl_geometry_scale", .05, 1, .05, 1)
        self.lbl = self._toggle(acoustic, "LBL field deployed", "lbl_available", True)
        self.usbl = self._toggle(acoustic, "USBL vessel present", "usbl_available", True)
        self.acoustic_failure = self._toggle(acoustic, "acoustic system failure", "acoustic_failure")

        card, compound = self._card(page, "compound faults / recovery")
        card.pack(fill="x", pady=8)
        for text, name, colour in (
            ("OPTICAL + DVL LOSS", "optical_dvl", AMBER),
            ("ACOUSTIC + DVL LOSS", "acoustic_dvl", AMBER),
            ("ALL HORIZONTAL AIDING LOST", "all_horizontal", RED),
            ("RECOVER ALL", "recover_all", GREEN),
        ):
            tk.Button(compound, text=text, command=lambda n=name: self._compound(n),
                      bg=colour, fg=BG, relief="flat", pady=5,
                      font=("DejaVu Sans", 8, "bold")).pack(fill="x", pady=2)

    def _build_display(self, parent):
        card, plot = self._card(parent, "true and estimated horizontal trajectory")
        card.pack(fill="both", expand=True, pady=(0, 8))
        self.plot = tk.Canvas(plot, bg="#061015", highlightthickness=0, height=400)
        self.plot.pack(fill="both", expand=True, pady=(6, 0))
        self.plot.bind("<Configure>", lambda _e: self._draw_plot())

        lower = tk.Frame(parent, bg=BG)
        lower.pack(fill="x")
        card, live = self._card(lower, "live observable navigation state")
        card.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self.live_labels = {}
        for key, label in (
            ("time_s", "simulation time"), ("horizontal_error_m", "horizontal error"),
            ("navigation_mode", "navigation mode"), ("mode_reason", "mode reason"),
            ("evidence", "sensor evidence"), ("selected_acoustic_technique", "selected acoustic"),
            ("altitude", "altitude / depth"), ("uncertainty_trace_m2", "position uncertainty"),
            ("mission_action", "mission action"), ("gps_status", "surface / GPS"),
        ):
            row = tk.Frame(live, bg=PANEL); row.pack(fill="x", pady=1)
            tk.Label(row, text=label, bg=PANEL, fg=DIM, width=20, anchor="w",
                     font=("DejaVu Sans", 8)).pack(side="left")
            value = tk.Label(row, text="—", bg=PANEL, fg=INK, anchor="w",
                             font=("DejaVu Sans", 8, "bold"))
            value.pack(side="left", fill="x", expand=True)
            self.live_labels[key] = value

        card, events = self._card(lower, "recent mode changes")
        card.pack(side="right", fill="both", expand=True, padx=(4, 0))
        self.event_text = tk.Text(events, height=12, width=38, bg=PANEL, fg=INK,
                                  relief="flat", state="disabled",
                                  font=("DejaVu Sans Mono", 8))
        self.event_text.pack(fill="both", expand=True, pady=(5, 0))

    def _set(self, control, value):
        if self.environment and not self.replay_path:
            self.environment.set_control(control, value)

    def _compound(self, name):
        if self.environment and not self.replay_path:
            self.environment.apply_compound(name)
            self._sync_controls(self.environment.controls)

    def _sync_controls(self, values):
        mapping = {
            "turbidity": self.turbidity, "optical_failure": self.optical_failure,
            "dvl_bottom_probability": self.bottom, "dvl_water_probability": self.water,
            "dvl_noise_scale": self.dvl_noise, "dvl_crashout": self.dvl_crash,
            "current_east_mps": self.current_e, "current_north_mps": self.current_n,
            "acoustic_noise_db": self.acoustic_noise, "lbl_geometry_scale": self.lbl_geometry,
            "lbl_available": self.lbl, "usbl_available": self.usbl,
            "acoustic_failure": self.acoustic_failure,
        }
        for key, variable in mapping.items():
            variable.set(values[key])

    def _start(self):
        if self.worker and self.worker.is_alive():
            return
        self.truth.clear(); self.estimate.clear(); self.mode_events.clear()
        self.last_packet = None
        self.status.configure(text="STARTING", fg=AMBER)
        policy = PolicyKind(self.policy_var.get())
        replay = self.replay_path

        def created(environment):
            self.environment = environment
            environment.set_realtime_factor(self.rate.get())
            self.messages.put(("environment", environment.controls))

        def work():
            run_interactive_session(policy_kind=policy, seed=self.seed,
                root=31_895_100, horizon_s=self.horizon_s, dt_s=self.dt_s,
                realtime_factor=self.rate.get(), replay_record=replay,
                on_environment=created,
                on_telemetry=lambda packet: self.messages.put(("telemetry", packet)),
                on_complete=lambda result: self.messages.put(("complete", result)))

        self.worker = threading.Thread(target=work, name="study3-interactive", daemon=True)
        self.worker.start()

    def _pause(self):
        if not self.environment:
            return
        paused = self.pause_btn.cget("text") == "PAUSE"
        self.environment.pause(paused)
        self.pause_btn.configure(text="RESUME" if paused else "PAUSE")
        self.status.configure(text="PAUSED" if paused else "RUNNING",
                              fg=AMBER if paused else GREEN)

    def _reset(self):
        if self.environment:
            self.environment.stop()
        self.environment = None
        self.replay_path = None
        self.pause_btn.configure(text="PAUSE")
        self.status.configure(text="RESETTING", fg=AMBER)
        self.root.after(150, self._start_when_stopped)

    def _start_when_stopped(self):
        if self.worker and self.worker.is_alive():
            self.root.after(100, self._start_when_stopped)
        else:
            self._start()

    def _record(self):
        if not self.environment:
            messagebox.showinfo("Record", "Start a session first.")
            return
        if not self.record_armed:
            self.record_armed = True
            self.record_btn.configure(text="SAVE RECORDING", bg=RED)
            return
        path = filedialog.asksaveasfilename(
            title="Save exact disturbance sequence", defaultextension=".json",
            initialfile=f"study3_disturbance_seed_{self.environment.seed}.json",
            filetypes=(("JSON recording", "*.json"),))
        if path:
            save_recording(path, self.environment, self.policy_var.get(),
                           root=31_895_100, index=0)
            self.status.configure(text=f"SAVED {Path(path).name}", fg=CYAN)
        self.record_armed = False
        self.record_btn.configure(text="RECORD", bg=CYAN)

    def _replay(self):
        path = filedialog.askopenfilename(title="Replay disturbance sequence",
            filetypes=(("JSON recording", "*.json"),))
        if not path:
            return
        if self.environment:
            self.environment.stop()
        self.replay_path = path
        self.environment = None
        self.status.configure(text=f"REPLAY {Path(path).name}", fg=CYAN)
        self.root.after(150, self._start_when_stopped)

    def _poll(self):
        try:
            while True:
                kind, value = self.messages.get_nowait()
                if kind == "environment":
                    self._sync_controls(value)
                    self.status.configure(text="REPLAYING" if self.replay_path else "RUNNING",
                                          fg=CYAN if self.replay_path else GREEN)
                elif kind == "telemetry":
                    self._telemetry(value)
                elif kind == "complete":
                    status = value["status"].upper()
                    self.status.configure(text=status, fg=GREEN if status == "COMPLETE" else RED)
                    result = value.get("result", {})
                    if result.get("gps_reacquired"):
                        self.live_labels["gps_status"].configure(
                            text="GPS reacquired · mission terminated", fg=AMBER)
                    if value["status"] == "error":
                        messagebox.showerror("Simulation error", value["error"])
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def _telemetry(self, packet):
        self.last_packet = packet
        self.truth.append(packet["true_position"][:2])
        self.estimate.append(packet["estimated_position"][:2])
        if packet["mode_changed"]:
            self.mode_events.appendleft(
                f'{packet["time_s"]:6.1f}s  {packet["navigation_mode"]}\n'
                f'         {packet["mode_reason"]}')
        evidence = (f'opt={"fix" if packet["optical_available"] else "—"} '
                    f'({packet["optical_quality"]:.2f}), '
                    f'DVL={"B" if packet["dvl_bottom_lock"] else "—"}/'
                    f'{"W" if packet["dvl_water_track"] else "—"}, '
                    f'acoustic={",".join(packet["responding_services"]) or "—"} '
                    f'pkt={"yes" if packet["acoustic_packet"] else "no"}')
        values = {
            "time_s": f'{packet["time_s"]:.1f} s',
            "horizontal_error_m": f'{packet["horizontal_error_m"]:.3f} m',
            "navigation_mode": packet["navigation_mode"],
            "mode_reason": packet["mode_reason"], "evidence": evidence,
            "selected_acoustic_technique": packet["selected_acoustic_technique"],
            "altitude": (f'{packet["observed_altitude_m"]:.2f} m / '
                         f'{-packet["true_position"][2]:.2f} m depth'),
            "uncertainty_trace_m2": f'{packet["uncertainty_trace_m2"]:.3f} m²',
            "mission_action": packet["mission_action"], "gps_status": packet["gps_status"],
        }
        for key, text in values.items():
            self.live_labels[key].configure(text=text,
                fg=RED if key == "mission_action" and text == "surface_for_gps" else INK)
        self.event_text.configure(state="normal")
        self.event_text.delete("1.0", "end")
        self.event_text.insert("1.0", "\n\n".join(self.mode_events))
        self.event_text.configure(state="disabled")
        self._draw_plot()

    def _draw_plot(self):
        canvas = self.plot
        canvas.delete("all")
        width, height = max(2, canvas.winfo_width()), max(2, canvas.winfo_height())
        canvas.create_text(12, 10, anchor="nw", text="TRUE", fill=GREEN,
                           font=("DejaVu Sans", 8, "bold"))
        canvas.create_text(62, 10, anchor="nw", text="ESTIMATE", fill=CYAN,
                           font=("DejaVu Sans", 8, "bold"))
        points = list(self.truth) + list(self.estimate)
        if len(points) < 2:
            return
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
        span = max(xmax - xmin, ymax - ymin, 1.0) * 1.15
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        scale = min((width - 40) / span, (height - 50) / span)
        def project(p):
            return width / 2 + (p[0] - cx) * scale, height / 2 - (p[1] - cy) * scale
        for values, colour in ((self.truth, GREEN), (self.estimate, CYAN)):
            coords = [v for p in values for v in project(p)]
            if len(coords) >= 4:
                canvas.create_line(*coords, fill=colour, width=2)
            x, y = project(values[-1])
            canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill=colour, outline="")

    def _close(self):
        if self.environment:
            self.environment.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Interactive Study-3 environment laboratory")
    parser.add_argument("--seed", type=int, default=31_895_000)
    parser.add_argument("--horizon-s", type=float, default=900.)
    parser.add_argument("--dt-s", type=float, default=1.)
    args = parser.parse_args(argv)
    Study3ControlWindow(seed=args.seed, horizon_s=args.horizon_s, dt_s=args.dt_s).run()


if __name__ == "__main__":
    main()
