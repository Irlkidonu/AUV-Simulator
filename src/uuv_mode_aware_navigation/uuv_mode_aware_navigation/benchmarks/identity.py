"""Resolve benchmark identity before constructing scientific components."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
from pathlib import Path
from typing import Mapping


OPTICAL_FIDELITIES = frozenset({"study2_abstract", "pose_rendered", "image_localizer"})
TRN_FIDELITIES = frozenset({"study2_gradient", "terrain_matcher"})
VEHICLE_FIDELITIES = frozenset({"study2_first_order", "pose_kinematic", "six_dof_future"})
RNG_MODES = frozenset({"study2_shared_stream", "component_substreams"})


@dataclass(frozen=True)
class BenchmarkIdentity:
    benchmark: str
    development_seed_root: int
    optical_fidelity: str
    trn_fidelity: str
    vehicle_fidelity: str
    rng_mode: str
    manager_fidelity: str
    metric_fidelity: str
    allow_fidelity_overrides: bool

    def __post_init__(self) -> None:
        allowed = {
            "optical_fidelity": OPTICAL_FIDELITIES,
            "trn_fidelity": TRN_FIDELITIES,
            "vehicle_fidelity": VEHICLE_FIDELITIES,
            "rng_mode": RNG_MODES,
        }
        for field, values in allowed.items():
            if getattr(self, field) not in values:
                raise ValueError(f"unknown {field}: {getattr(self, field)!r}")
        if self.development_seed_root < 0:
            raise ValueError("development_seed_root must be non-negative")

    def with_overrides(self, **overrides: str) -> "BenchmarkIdentity":
        if overrides and not self.allow_fidelity_overrides:
            raise ValueError(
                f"{self.benchmark} is immutable and refuses fidelity overrides"
            )
        permitted = {
            "optical_fidelity", "trn_fidelity", "vehicle_fidelity", "rng_mode"
        }
        unknown = set(overrides) - permitted
        if unknown:
            raise ValueError(f"unsupported fidelity override(s): {sorted(unknown)}")
        return replace(self, **overrides)


def load_benchmark(path: str | Path) -> BenchmarkIdentity:
    payload: Mapping[str, object] = json.loads(Path(path).read_text())
    required = {
        "benchmark", "development_seed_root", "optical_fidelity",
        "trn_fidelity", "vehicle_fidelity", "rng_mode", "manager_fidelity",
        "metric_fidelity", "allow_fidelity_overrides",
    }
    missing = required - set(payload)
    if missing:
        raise ValueError(f"benchmark declaration missing: {sorted(missing)}")
    return BenchmarkIdentity(**{key: payload[key] for key in required})

