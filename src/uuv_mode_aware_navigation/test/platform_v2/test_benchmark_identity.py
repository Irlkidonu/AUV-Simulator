"""Benchmark selection cannot silently mutate the published identity."""

from pathlib import Path

import pytest

from uuv_mode_aware_navigation.benchmarks import load_benchmark


ROOT = Path(__file__).resolve().parents[4]


def test_legacy_identity_resolves_exact_fidelities() -> None:
    identity = load_benchmark(ROOT / "benchmarks" / "study2_legacy.json")
    assert identity.benchmark == "study2_legacy_v1.0"
    assert identity.development_seed_root == 20_000_000
    assert identity.optical_fidelity == "study2_abstract"
    assert identity.trn_fidelity == "study2_gradient"
    assert identity.vehicle_fidelity == "study2_first_order"
    assert identity.rng_mode == "study2_shared_stream"
    assert identity.manager_fidelity == "study2_frozen"
    assert identity.metric_fidelity == "study2_frozen"


def test_legacy_refuses_every_fidelity_override() -> None:
    identity = load_benchmark(ROOT / "benchmarks" / "study2_legacy.json")
    with pytest.raises(ValueError, match="immutable"):
        identity.with_overrides(optical_fidelity="pose_rendered")


def test_platform_v2_accepts_declared_override() -> None:
    identity = load_benchmark(ROOT / "benchmarks" / "platform_v2.json")
    changed = identity.with_overrides(trn_fidelity="terrain_matcher")
    assert changed.trn_fidelity == "terrain_matcher"
    assert identity.trn_fidelity == "study2_gradient"

