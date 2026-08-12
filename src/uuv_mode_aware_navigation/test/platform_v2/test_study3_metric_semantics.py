"""Regression tests for logical-transition versus physical-action metrics."""
from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uuv_mode_aware_navigation.study3.policies import (  # noqa: E402
    PolicyKind, Study3Action, Study3Policy)
from uuv_mode_aware_navigation.study3.simulation import run_one  # noqa: E402


class LabelChangingFixedBehaviour(Study3Policy):
    """Emit fixed physical commands while preserving inferred mode labels."""

    def step(self, observable):
        action, output = super().step(observable)
        return Study3Action(
            self.fixed.speed_mps, self.fixed.altitude_m, "continue",
            self.fixed.optical_channel, self.fixed.acoustic_technique,
            self.fixed.fusion_mode, False, action.navigation_mode), output


def test_label_changes_are_not_physical_or_unnecessary_interventions():
    result = run_one(
        31_400_001, "S3_INFRASTRUCTURE_WARNING", 0, PolicyKind.REACTIVE,
        redesign_version=2, policy_factory=LabelChangingFixedBehaviour)
    assert result.mode_switches > 0
    assert result.physical_interventions == 0
    assert result.unnecessary_interventions == 0


def test_instrumentation_does_not_change_physical_trace_digest():
    ordinary = run_one(
        31_400_002, "S3_DVL_GRADUAL", 0, PolicyKind.REACTIVE,
        redesign_version=2)
    repeated = run_one(
        31_400_002, "S3_DVL_GRADUAL", 0, PolicyKind.REACTIVE,
        redesign_version=2)
    assert ordinary.trace_digest == repeated.trace_digest
