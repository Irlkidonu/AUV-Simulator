"""Mechanism test: mode labels alone must not produce the behavioural effect.

The criticism this test answers is that a "mode-aware" controller is really
sensor fusion wearing different names. The ablation keeps mode inference fully
alive -- the selector runs, the mode changes, the reason is recorded -- but
prevents the selected mode from changing the sensing, fusion or navigation
configuration actually applied to the vehicle.

If the mode-aware effect came from the labels, the ablation would behave like
the real reactive policy. It does not: it behaves like the fixed baseline.

This is a mechanism test, not a scientific comparator. It uses development
roots only and asserts mechanism, not benefit.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uuv_mode_aware_navigation.study3.policies import (  # noqa: E402
    FixedConfiguration, PolicyKind, Study3Action, Study3Policy)
from uuv_mode_aware_navigation.study3.simulation import run_one  # noqa: E402
from uuv_mode_aware_navigation.estimator import FusionMode  # noqa: E402

DEVELOPMENT_ROOT = 31_400_000
#: Families whose declared mechanism is a capability transition with a
#: configuration consequence. Chosen before running, on mechanism grounds.
FAMILIES = ("S3_COMPOUND_OPTICAL_ACOUSTIC", "S3_INFRASTRUCTURE_WARNING")

#: Physical outcome fields. `trace_digest` is deliberately excluded: it hashes
#: the action record, which carries the mode *name*, so a pure label change
#: alters it by construction and it cannot discriminate label from behaviour.
PHYSICAL = ("completed", "overall_rmse_m", "peak_error_m", "optical_fixes",
            "acoustic_fixes", "survey_coverage_fraction", "unaided_time_s",
            "longest_unaided_gap_s")


class LabelOnlyPolicy(Study3Policy):
    """Infer and record the mode; emit the fixed configuration regardless.

    Everything upstream of the action is untouched: the coordinator runs, the
    belief updates, ``mode_selector.select`` is called and
    ``last_mode_decision`` is set, so telemetry still observes real mode
    changes. Only the returned action is replaced.
    """

    def step(self, observable):
        action, output = super().step(observable)
        labelled = Study3Action(
            self.fixed.speed_mps,
            self.fixed.altitude_m,
            # Terminal safety is a safety mechanism, not a mode label, and is
            # deliberately still honoured so the ablation cannot look better by
            # ignoring a safety stop.
            "surface_for_gps" if action.mission_action == "surface_for_gps" else "continue",
            self.fixed.optical_channel,
            self.fixed.acoustic_technique,
            self.fixed.fusion_mode,
            False,
            action.navigation_mode,
        )
        # ``Study3Policy.step`` applies its selected fusion mode directly to
        # the estimator before returning. Restore the fixed treatment as well
        # as replacing the outward action, or this would not be label-only.
        self.estimator.filter.fusion = FusionMode(self.fixed.fusion_mode)
        return labelled, output


def _run(kind, family, index, factory=None):
    return run_one(DEVELOPMENT_ROOT, family, index, kind,
                   redesign_version=2, policy_factory=factory)


@pytest.mark.parametrize("family", FAMILIES)
def test_label_only_ablation_still_infers_and_records_mode_changes(family):
    """The ablation must keep the labels alive, or it proves nothing."""
    result = _run(PolicyKind.REACTIVE, family, 0, LabelOnlyPolicy)
    telemetry = result.mode_telemetry
    assert telemetry is not None
    assert len(telemetry["modes_visited_in_order"]) >= 1
    # Mode inference is genuinely running: an absolute-aiding source is
    # attributed for part of the run and the selector recorded a decision.
    assert telemetry["absolute_source_s"], "selector produced no attribution"


@pytest.mark.parametrize("family", FAMILIES)
def test_label_only_ablation_matches_fixed_behaviour_not_reactive(family):
    """Labels without configuration authority reproduce the fixed baseline."""
    fixed = _run(PolicyKind.FIXED, family, 0)
    label_only = _run(PolicyKind.REACTIVE, family, 0, LabelOnlyPolicy)

    # The applied configuration is the fixed one on every axis the mode could
    # otherwise have changed.
    reference = FixedConfiguration()
    assert label_only.optical_channels_used == (reference.optical_channel,)
    assert label_only.acoustic_techniques_used == (reference.acoustic_technique,)
    assert label_only.fusion_modes_used == (reference.fusion_mode,)

    # And every physical outcome is the fixed one.
    for field in PHYSICAL:
        expected, observed = getattr(fixed, field), getattr(label_only, field)
        if isinstance(expected, float):
            assert observed == pytest.approx(expected, rel=1e-12, abs=1e-12), field
        else:
            assert observed == expected, field


@pytest.mark.parametrize("family", FAMILIES)
def test_real_reactive_policy_differs_from_label_only_ablation(family):
    """The effect must come from configuration change, not from naming."""
    reactive = _run(PolicyKind.REACTIVE, family, 0)
    label_only = _run(PolicyKind.REACTIVE, family, 0, LabelOnlyPolicy)

    differing = [f for f in PHYSICAL
                 if getattr(reactive, f) != getattr(label_only, f)]
    assert differing, (
        "reactive policy is physically indistinguishable from label-only: "
        "mode selection is not reaching the vehicle")

    # The difference is visible in what was actually applied, on at least one
    # configuration axis.
    applied_differs = (
        reactive.optical_channels_used != label_only.optical_channels_used
        or reactive.acoustic_techniques_used != label_only.acoustic_techniques_used
        or reactive.fusion_modes_used != label_only.fusion_modes_used
        or reactive.mission_actions_used != label_only.mission_actions_used
        or reactive.recovery_action_counts != label_only.recovery_action_counts)
    assert applied_differs, "no applied configuration axis differs"
