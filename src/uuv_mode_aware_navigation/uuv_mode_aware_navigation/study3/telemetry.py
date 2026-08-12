"""Analysis-only navigation-mode telemetry for Study 3.

This module observes. It holds no thresholds, returns nothing any policy
reads, and is fed after the policy step has already produced its action, so
it cannot influence a decision, the estimator, scenario physics or the
per-step trace digest.

Everything recorded here is derived from ``ModeDecision`` -- the value the
selector already returns -- plus quantities the run loop has already
computed. No truth-side state enters this module.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class ModeTransition:
    """One observed change of navigation mode."""

    time_s: float
    source: str
    target: str
    reason: str
    absolute_source: str | None
    velocity_source: str


@dataclass
class ModeTelemetry:
    """Accumulates observable mode behaviour over a run.

    ``observe`` is called once per step with the decision the policy has
    already taken. It never returns a control value.
    """

    dwell_s: dict[str, float] = field(default_factory=dict)
    transitions: list[ModeTransition] = field(default_factory=list)
    velocity_source_s: dict[str, float] = field(default_factory=dict)
    absolute_source_s: dict[str, float] = field(default_factory=dict)
    unaided_s: float = 0.0
    #: First entry into each behaviourally distinct state, with the observable
    #: reason recorded at that instant.
    first_relative_entry: tuple[float, str] | None = None
    first_terminal_entry: tuple[float, str] | None = None
    first_fallback_entry: tuple[float, str] | None = None
    #: Recovery actions actually executed, by name.
    recovery_executed: dict[str, int] = field(default_factory=dict)
    _mode: str | None = None
    _entered_s: float = 0.0

    def observe(self, time_s, decision, dt_s, *, aided, recovery_action=None,
                recovery_executed=False):
        """Record one step. Called after the action is fixed."""
        mode = decision.mode.value
        velocity = decision.velocity_source
        absolute = decision.absolute_source or "none"

        if self._mode is None:
            self._mode = mode
            self._entered_s = float(time_s)
        elif mode != self._mode:
            self.transitions.append(ModeTransition(
                float(time_s), self._mode, mode, decision.reason,
                decision.absolute_source, velocity))
            self._mode = mode
            self._entered_s = float(time_s)

        self.dwell_s[mode] = self.dwell_s.get(mode, 0.0) + dt_s
        self.velocity_source_s[velocity] = self.velocity_source_s.get(velocity, 0.0) + dt_s
        self.absolute_source_s[absolute] = self.absolute_source_s.get(absolute, 0.0) + dt_s
        if not aided:
            self.unaided_s += dt_s

        if mode == "relative_dead_reckoning" and self.first_relative_entry is None:
            self.first_relative_entry = (float(time_s), decision.reason)
        if mode == "terminal_degraded" and self.first_terminal_entry is None:
            self.first_terminal_entry = (float(time_s), decision.reason)
        if decision.fallback_required and self.first_fallback_entry is None:
            self.first_fallback_entry = (float(time_s), decision.reason)

        if recovery_executed and recovery_action:
            self.recovery_executed[recovery_action] = (
                self.recovery_executed.get(recovery_action, 0) + 1)

    # -- read-only summaries -------------------------------------------------

    @property
    def transition_count(self) -> int:
        return len(self.transitions)

    @property
    def modes_visited(self) -> tuple[str, ...]:
        """Modes in first-entry order, unlike the unordered set already stored."""
        seen: list[str] = []
        for transition in self.transitions:
            if transition.source not in seen:
                seen.append(transition.source)
            if transition.target not in seen:
                seen.append(transition.target)
        if not seen and self._mode is not None:
            seen.append(self._mode)
        return tuple(seen)

    def directed_transition_counts(self) -> tuple[tuple[str, int], ...]:
        """``source->target`` counts, so direction is preserved."""
        counts: dict[str, int] = {}
        for transition in self.transitions:
            key = f"{transition.source}->{transition.target}"
            counts[key] = counts.get(key, 0) + 1
        return tuple(sorted(counts.items()))

    def transition_reasons(self) -> tuple[tuple[str, int], ...]:
        counts: dict[str, int] = {}
        for transition in self.transitions:
            counts[transition.reason] = counts.get(transition.reason, 0) + 1
        return tuple(sorted(counts.items()))

    def longest_dwell(self) -> tuple[str, float]:
        if not self.dwell_s:
            return ("none", 0.0)
        mode = max(self.dwell_s, key=lambda key: self.dwell_s[key])
        return (mode, self.dwell_s[mode])

    def mean_dwell_s(self) -> float:
        """Mean time between mode changes; infinite when the mode never changes."""
        if not self.transitions:
            return math.inf
        total = sum(self.dwell_s.values())
        return total / (len(self.transitions) + 1)

    def as_record(self) -> dict:
        """Flat, JSON-serialisable summary for analysis scripts."""
        mode, dwell = self.longest_dwell()
        return {
            "mode_dwell_s": tuple(sorted(self.dwell_s.items())),
            "mode_transitions": self.transition_count,
            "mode_transitions_directed": self.directed_transition_counts(),
            "mode_transition_reasons": self.transition_reasons(),
            "modes_visited_in_order": self.modes_visited,
            "velocity_source_s": tuple(sorted(self.velocity_source_s.items())),
            "absolute_source_s": tuple(sorted(self.absolute_source_s.items())),
            "time_without_horizontal_absolute_s": round(self.unaided_s, 6),
            "first_relative_entry": self.first_relative_entry,
            "first_terminal_entry": self.first_terminal_entry,
            "first_fallback_entry": self.first_fallback_entry,
            "recovery_executed": tuple(sorted(self.recovery_executed.items())),
            "longest_dwell_mode": mode,
            "longest_dwell_s": round(dwell, 6),
            "mean_dwell_s": self.mean_dwell_s(),
        }
