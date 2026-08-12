"""Characterise the public policy extension point."""

from __future__ import annotations

import pytest

from uuv_mode_aware_navigation.comparators import FixedPolicy
from uuv_mode_aware_navigation.policies import PolicyDeclaration, PolicyRegistry


def _declaration(name: str = "third_party") -> PolicyDeclaration:
    return PolicyDeclaration(
        name=name,
        information_available=("onboard_observables",),
        action_space=("fixed_configuration",),
        tuning_budget="none",
        internal_objective="constant policy smoke test",
        deployable=True,
        expected_compute_ms=0.01,
    )


def test_third_party_policy_registers_without_core_edit() -> None:
    registry = PolicyRegistry()
    registry.register(
        _declaration(),
        lambda: FixedPolicy(name="third_party"),
    )
    policy = registry.create("third_party")
    assert policy.name == "third_party"
    assert registry.declaration("third_party").deployable


def test_duplicate_policy_name_is_rejected() -> None:
    registry = PolicyRegistry()
    registry.register(_declaration(), lambda: FixedPolicy(name="third_party"))
    with pytest.raises(ValueError, match="already registered"):
        registry.register(_declaration(), lambda: FixedPolicy(name="third_party"))


def test_factory_must_honour_registered_name() -> None:
    registry = PolicyRegistry()
    registry.register(_declaration(), lambda: FixedPolicy(name="wrong"))
    with pytest.raises(ValueError, match="produced policy named"):
        registry.create("third_party")

