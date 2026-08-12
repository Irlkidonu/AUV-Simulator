"""Stable public contracts for mode-aware navigation policies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

from ..manager import VehicleConfiguration
from ..modes import Mode, Observables


@dataclass(frozen=True)
class PolicyDecision:
    """One policy decision at a navigation decision epoch."""

    configuration: VehicleConfiguration
    mode: Optional[Mode] = None
    use_absolute_aiding: bool = True
    optical_covariance_scale: float = 1.0


class Policy(Protocol):
    """Minimum interface implemented by every benchmark policy."""

    name: str

    def update(self, obs: Observables, dt: float, t: float) -> PolicyDecision:
        ...


@dataclass(frozen=True)
class PolicyDeclaration:
    """Machine-readable disclosure of a policy's experimental privileges."""

    name: str
    information_available: Tuple[str, ...]
    action_space: Tuple[str, ...]
    tuning_budget: str
    internal_objective: str
    deployable: bool
    expected_compute_ms: Optional[float] = None

