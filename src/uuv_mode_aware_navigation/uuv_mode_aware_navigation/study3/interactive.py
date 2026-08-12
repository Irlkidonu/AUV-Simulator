"""Interactive, truth-side Study-3 environment control and replay.

This module deliberately sits on the simulator side of the observation boundary.
It changes :class:`PhysicalState` values consumed by the sensor generators; it
does not expose control names, values, or event timing to a policy.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable

import numpy as np

from .environment_generator import (
    AvailabilityProcess, BoundedProcess, EnvironmentConfig,
    EnvironmentRealization, deployment_informed_environment_configuration,
    generate_environment,
)
from .policies import FixedConfiguration, PolicyKind, Study3Policy
from .scenarios import InfrastructureContext, PhysicalState
from .simulation import run_one


SCHEMA = "study3_interactive_disturbance_v1"
CONTROL_DEFAULTS = {
    "turbidity": 0.10,
    "current_east_mps": 0.0,
    "current_north_mps": 0.0,
    "dvl_bottom_probability": 0.98,
    "dvl_water_probability": 0.90,
    "dvl_noise_scale": 1.0,
    "dvl_crashout": False,
    "acoustic_noise_db": 48.0,
    "acoustic_failure": False,
    "lbl_available": True,
    "lbl_geometry_scale": 1.0,
    "usbl_available": True,
    "optical_failure": False,
}


def interactive_environment_config() -> EnvironmentConfig:
    """Static benign background; every change comes from a recorded control."""
    constant = lambda value: BoundedProcess(value, value, value, 60.0, 0.0, value)
    healthy = AvailabilityProcess(True, 0.0, 0.0)
    return EnvironmentConfig(
        "study3_interactive_manual", constant(0.10), constant(0.0), constant(0.0),
        constant(48.0), constant(1.0), constant(0.90), healthy, healthy,
        healthy, healthy, healthy, False,
    )


@dataclass(frozen=True)
class DisturbanceEvent:
    effective_step: int
    simulation_time_s: float
    sequence: int
    control: str
    value: float | bool


class SessionStopped(RuntimeError):
    """Private control-flow exception used to terminate/reset a live session."""


class InteractiveEnvironment:
    """Thread-safe physical-state adapter with deterministic event replay.

    ``physical_state`` is the only interface seen by ``run_one``.  Ordinary
    execution calls are paced; evaluator-only future-state queries are not and
    receive a snapshot of the controls currently in force.
    """

    def __init__(self, base: EnvironmentRealization, *, realtime_factor: float = 1.0,
                 replay_events=(), on_physical_state: Callable | None = None,
                 pace: bool = True):
        self.base = base
        self.config = base.config
        self.seed = base.seed
        self.dt_s = base.dt_s
        self.digest = base.digest
        self._catalogue = ("lbl", "usbl")
        self._controls = dict(CONTROL_DEFAULTS)
        self._events: list[DisturbanceEvent] = []
        self._replay = sorted((DisturbanceEvent(**x) if isinstance(x, dict) else x
                               for x in replay_events),
                              key=lambda x: (x.effective_step, x.sequence))
        self._replay_cursor = 0
        self._sequence = 0
        self._expected_step = 0
        self._current_step = -1
        self._realtime_factor = max(0.05, float(realtime_factor))
        self._pace = bool(pace)
        self._paused = False
        self._stopped = False
        self._condition = threading.Condition()
        self._wall_origin = None
        self._paused_started = None
        self._paused_total = 0.0
        self.on_physical_state = on_physical_state
        self.last_truth = None

    @property
    def horizon_s(self):
        return self.base.horizon_s

    @property
    def service_catalogue(self):
        # Installed service identities are legitimate pre-mission knowledge.
        # Current availability still requires a response to a serialized probe.
        return self._catalogue

    @property
    def current_step(self):
        with self._condition:
            return self._current_step

    @property
    def controls(self):
        with self._condition:
            return dict(self._controls)

    @property
    def events(self):
        with self._condition:
            return tuple(self._events or self._replay)

    def set_realtime_factor(self, value: float):
        with self._condition:
            self._realtime_factor = max(0.05, float(value))
            # Re-anchor pacing without changing simulation time.
            if self._current_step >= 0:
                self._wall_origin = time.monotonic() - (
                    self._current_step * self.dt_s / self._realtime_factor)
                self._paused_total = 0.0
            self._condition.notify_all()

    def pause(self, paused=True):
        with self._condition:
            paused = bool(paused)
            if paused and not self._paused:
                self._paused_started = time.monotonic()
            elif not paused and self._paused and self._paused_started is not None:
                self._paused_total += time.monotonic() - self._paused_started
                self._paused_started = None
            self._paused = paused
            self._condition.notify_all()

    def stop(self):
        with self._condition:
            self._stopped = True
            self._condition.notify_all()

    def set_control(self, control: str, value, *, record=True):
        if control not in CONTROL_DEFAULTS:
            raise KeyError(f"unknown interactive control {control}")
        if isinstance(CONTROL_DEFAULTS[control], bool):
            value = bool(value)
        else:
            value = float(value)
        with self._condition:
            effective = max(0, self._current_step + 1)
            self._controls[control] = value
            if record:
                self._events.append(DisturbanceEvent(
                    effective, effective * self.dt_s, self._sequence, control, value))
                self._sequence += 1

    def apply_compound(self, name: str):
        presets = {
            "optical_dvl": {
                "turbidity": .90, "optical_failure": True,
                "dvl_crashout": True,
            },
            "acoustic_dvl": {
                "acoustic_failure": True, "dvl_crashout": True,
                "lbl_available": False, "usbl_available": False,
            },
            "all_horizontal": {
                "optical_failure": True, "dvl_crashout": True,
                "acoustic_failure": True, "lbl_available": False,
                "usbl_available": False,
            },
            "recover_all": dict(CONTROL_DEFAULTS),
        }
        if name not in presets:
            raise KeyError(name)
        for control, value in presets[name].items():
            self.set_control(control, value)

    def _apply_replay(self, step):
        while (self._replay_cursor < len(self._replay) and
               self._replay[self._replay_cursor].effective_step <= step):
            event = self._replay[self._replay_cursor]
            self._controls[event.control] = event.value
            self._replay_cursor += 1

    def _wait_for_step(self, step):
        with self._condition:
            if self._wall_origin is None:
                self._wall_origin = time.monotonic()
            while self._paused and not self._stopped:
                self._condition.wait(.1)
            if self._stopped:
                raise SessionStopped("interactive session stopped")
            if self._pace:
                while not self._stopped:
                    target = (self._wall_origin + self._paused_total +
                              step * self.dt_s / self._realtime_factor)
                    remaining = target - time.monotonic()
                    if remaining <= 0:
                        break
                    self._condition.wait(min(.05, remaining))
                    while self._paused and not self._stopped:
                        self._condition.wait(.1)
                if self._stopped:
                    raise SessionStopped("interactive session stopped")
            self._current_step = step
            self._apply_replay(step)
            return dict(self._controls)

    @staticmethod
    def _controlled_state(base: PhysicalState, controls) -> PhysicalState:
        optical_failed = controls["optical_failure"]
        dvl_failed = controls["dvl_crashout"]
        acoustic_failed = controls["acoustic_failure"]
        turbidity = 1.0 if optical_failed else np.clip(controls["turbidity"], 0, 1)
        bottom = .01 if dvl_failed else np.clip(controls["dvl_bottom_probability"], .01, 1)
        water = .01 if dvl_failed else np.clip(controls["dvl_water_probability"], .01, 1)
        noise = np.clip(controls["acoustic_noise_db"], 35, 95)
        response = (.01 if acoustic_failed else
                    1.0 / (1.0 + math.exp((noise - 64.0) / 5.0)))
        services = set()
        if controls["lbl_available"]:
            services.add("lbl")
        if controls["usbl_available"]:
            services.add("usbl")
        lbl_geometry = np.clip(controls["lbl_geometry_scale"], .05, 1)
        service_response = (
            ("lbl", float(np.clip(response * lbl_geometry if "lbl" in services else .01, .01, 1))),
            ("usbl", float(np.clip(response if "usbl" in services else .01, .01, 1))),
        )
        degraded = bool(turbidity > .35 or bottom < .5 or water < .5 or
                        response < .5 or abs(controls["current_east_mps"]) > .08 or
                        abs(controls["current_north_mps"]) > .08 or not services)
        return replace(
            base, turbidity=float(turbidity),
            dvl_lock_probability=float(bottom),
            dvl_water_track_probability=float(water),
            acoustic_response_probability=float(response),
            acoustic_noise_db=float(noise), infrastructure_available=bool(services),
            degradation_active=degraded,
            infrastructure_context=InfrastructureContext.INFRASTRUCTURE_TRANSITION,
            deployed_acoustic_services=frozenset(services),
            current_east_mps=float(controls["current_east_mps"]),
            current_north_mps=float(controls["current_north_mps"]),
            dvl_noise_scale=float(np.clip(controls["dvl_noise_scale"], 1, 8)),
            lbl_geometry_scale=float(lbl_geometry),
            service_response_probability=service_response,
            dvl_forced_unavailable=bool(dvl_failed),
        )

    def physical_state(self, step: int, *, altitude_m: float, position_xy=(0., 0.)):
        # ``run_one`` may ask an evaluator-only future question for forecast
        # scoring. It must not advance the live clock or consume replay events.
        ordinary = int(step) == self._expected_step
        if ordinary:
            controls = self._wait_for_step(int(step))
            self._expected_step += 1
        else:
            controls = self.controls
        state = self._controlled_state(
            self.base.physical_state(step, altitude_m=altitude_m, position_xy=position_xy),
            controls)
        if ordinary:
            self.last_truth = {
                "step": int(step), "time_s": int(step) * self.dt_s,
                "position_xy": tuple(map(float, position_xy)),
                "altitude_m": float(altitude_m), "physical": asdict(state),
            }
            if self.on_physical_state:
                self.on_physical_state(dict(self.last_truth))
        return state


def recording_record(environment: InteractiveEnvironment, policy: str,
                     *, root: int, index: int):
    payload = {
        "schema": SCHEMA, "policy_recorded": policy, "root": int(root),
        "index": int(index), "seed": environment.seed,
        "dt_s": environment.dt_s, "horizon_s": environment.horizon_s,
        "base_config": asdict(environment.base.config),
        "base_environment_digest": environment.base.digest,
        "service_catalogue": list(environment.service_catalogue),
        "events": [asdict(x) for x in environment.events],
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    payload["sha256"] = hashlib.sha256(canonical).hexdigest()
    return payload


def save_recording(path, environment, policy, *, root, index):
    record = recording_record(environment, policy, root=root, index=index)
    Path(path).write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    return record


def load_recording(path):
    record = json.loads(Path(path).read_text())
    if record.get("schema") != SCHEMA:
        raise ValueError("unsupported interactive recording schema")
    supplied = record.pop("sha256", None)
    canonical = json.dumps(record, sort_keys=True, separators=(",", ":")).encode()
    if supplied != hashlib.sha256(canonical).hexdigest():
        raise ValueError("interactive recording checksum mismatch")
    record["sha256"] = supplied
    config = EnvironmentConfig.from_dict(record["base_config"])
    base = generate_environment(config, record["seed"], record["horizon_s"], record["dt_s"])
    if base.digest != record["base_environment_digest"]:
        raise ValueError("base environment no longer reproduces recording")
    return record, base


def run_interactive_session(*, policy_kind=PolicyKind.REACTIVE, seed=31_895_000,
                            root=31_895_100, index=0, horizon_s=900., dt_s=1.,
                            realtime_factor=1., replay_record=None,
                            on_telemetry=None, on_complete=None,
                            on_environment=None, pace=True):
    """Construct and execute one non-campaign interactive demonstration."""
    if replay_record is None:
        base = generate_environment(interactive_environment_config(), seed, horizon_s, dt_s)
        events = ()
    else:
        record, base = load_recording(replay_record) if not isinstance(replay_record, dict) else (replay_record, None)
        if base is None:
            config = EnvironmentConfig.from_dict(record["base_config"])
            base = generate_environment(config, record["seed"], record["horizon_s"], record["dt_s"])
        seed, horizon_s, dt_s = record["seed"], record["horizon_s"], record["dt_s"]
        events = record["events"]
    environment = InteractiveEnvironment(base, realtime_factor=realtime_factor,
        replay_events=events, pace=pace)
    if on_environment:
        on_environment(environment)
    fixed = FixedConfiguration(optical_channel="lidar", altitude_m=5., speed_mps=.5,
                               acoustic_technique="lbl", fusion_mode="weight")
    if policy_kind is PolicyKind.DEPLOYMENT_FIXED:
        fixed = deployment_informed_environment_configuration(fixed, environment)

    class TelemetryPolicy(Study3Policy):
        previous_mode = None

        def step(self, observation):
            action, output = super().step(observation)
            truth = environment.last_truth or {}
            true_xy = np.asarray(truth.get("position_xy", (math.nan, math.nan)), dtype=float)
            estimated = self.estimator.position.copy()
            mode_changed = action.navigation_mode != self.__class__.previous_mode
            self.__class__.previous_mode = action.navigation_mode
            services = tuple(sorted(x.name for x in observation.acoustic.service_evidence
                                    if x.responding))
            packet = {
                "time_s": observation.time_s,
                "true_position": (float(true_xy[0]), float(true_xy[1]),
                                  -float(observation.altitude_m)),
                "estimated_position": tuple(map(float, estimated)),
                "horizontal_error_m": float(np.linalg.norm(estimated[:2] - true_xy)),
                "navigation_mode": action.navigation_mode,
                "mode_reason": self.last_mode_decision.reason,
                "mode_changed": mode_changed,
                "optical_available": observation.optical.available,
                "optical_quality": observation.optical.quality,
                "dvl_bottom_lock": observation.dvl.bottom_lock,
                "dvl_water_track": observation.dvl.water_track,
                "acoustic_packet": observation.acoustic.available,
                "responding_services": services,
                "selected_acoustic_technique": action.acoustic_technique,
                "commanded_altitude_m": action.altitude_m,
                "observed_altitude_m": observation.altitude_m,
                "uncertainty_trace_m2": float(np.trace(self.estimator.P[:3, :3])),
                "mission_action": action.mission_action,
                "recovery_reason": output.recovery.reason,
                "terminal_or_surfacing": action.mission_action == "surface_for_gps",
                "gps_status": "surfacing" if action.mission_action == "surface_for_gps" else "submerged",
                "belief": dict(output.belief.usable_probability),
                "controls": environment.controls,
            }
            if on_telemetry:
                on_telemetry(packet)
            return action, output

    try:
        result, trace = run_one(root, base.config.name, index, policy_kind, fixed,
            horizon_s=horizon_s, dt_s=dt_s, image_period_s=4., keep_trace=True,
            redesign_version=3, policy_factory=TelemetryPolicy,
            environment_realization=environment)
        completion = {"status": "complete", "result": asdict(result), "trace": trace}
    except SessionStopped:
        completion = {"status": "stopped"}
    except Exception as exc:  # surfaced to the UI; never silently converted to evidence
        completion = {"status": "error", "error": f"{type(exc).__name__}: {exc}"}
    if on_complete:
        on_complete(completion)
    return environment, completion
