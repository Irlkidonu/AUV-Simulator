"""Mechanism tests for the one-shot held-out runner.

None of these executes a held-out simulation. They exercise plan enumeration
and the guard logic only, so the one-shot authorization is not spent.

The substantive assertion is that the runner's enumerated execution plan
matches ``STUDY3_HELDOUT_DESIGN_V1.json`` exactly: same policies, families,
seed counts and totals, with no run outside the frozen design and none missing.
"""
from __future__ import annotations

import collections
import importlib.util
import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
REPOSITORY = Path(__file__).resolve().parents[4]
STUDY3 = REPOSITORY / "experiments/study3"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def _module():
    specification = importlib.util.spec_from_file_location(
        "run_heldout", STUDY3 / "run_heldout.py")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def runner():
    return _module()


@pytest.fixture(scope="module")
def design():
    return json.loads((STUDY3 / "STUDY3_HELDOUT_DESIGN_V1.json").read_text())


@pytest.fixture(scope="module")
def plan(runner):
    _, tasks, _ = runner.build_plan()
    return tasks


def test_plan_total_matches_the_frozen_design(plan, design):
    assert len(plan) == design["executions"]["total"] == 810


def test_plan_group_totals_match_the_frozen_design(plan, design):
    counts = collections.Counter(task["group"] for task in plan)
    assert counts["primary"] == design["executions"]["primary"] == 630
    assert counts["control"] == design["executions"]["controls"] == 180


def test_plan_uses_exactly_the_frozen_policies(plan, design):
    assert sorted({task["policy"] for task in plan}) == sorted(design["policies"])
    assert sorted(design["policies"]) == ["deployment_fixed", "fixed", "reactive"]


def test_every_family_policy_cell_has_the_declared_seed_count(plan, design):
    counts = collections.Counter((task["family"], task["policy"]) for task in plan)
    for family in design["primary_families"]:
        for policy in design["policies"]:
            assert counts[(family, policy)] == design["seeds_per_primary_family_policy"]
    for family in design["control_families"]:
        for policy in design["policies"]:
            assert counts[(family, policy)] == design["seeds_per_control_family_policy"]


def test_plan_contains_no_family_outside_the_frozen_design(plan, design):
    declared = set(design["primary_families"]) | set(design["control_families"])
    assert {task["family"] for task in plan} == declared


def test_indices_are_contiguous_from_zero_per_cell(plan, design):
    grouped = collections.defaultdict(set)
    for task in plan:
        grouped[(task["family"], task["policy"])].add(task["index"])
    for (family, _policy), indices in grouped.items():
        expected = (design["seeds_per_primary_family_policy"]
                    if family in design["primary_families"]
                    else design["seeds_per_control_family_policy"])
        assert indices == set(range(expected))


def test_every_task_targets_the_held_out_root_and_stage(plan, runner):
    assert {task["root"] for task in plan} == {runner.HELD_OUT_ROOT} == {32_000_000}
    assert {task["stage"] for task in plan} == {"heldout"}


def test_plan_is_paired_identically_across_policies(plan, design):
    """Each (family, index) cell must appear once under every policy."""
    cells = collections.defaultdict(set)
    for task in plan:
        cells[(task["family"], task["index"])].add(task["policy"])
    assert all(policies == set(design["policies"]) for policies in cells.values())


def test_fixed_and_reactive_use_the_authoritative_locked_configuration(plan, runner):
    locked = json.loads(runner.FIXED_LOCK.read_text())["configuration"]
    for task in plan:
        if task["policy"] in {"fixed", "reactive"}:
            assert task["configuration"] == locked


def test_deployment_informed_configuration_differs_only_by_technique(plan):
    """The fair comparator changes the technique and nothing else."""
    by_family = {}
    for task in plan:
        by_family.setdefault(task["family"], {})[task["policy"]] = task["configuration"]
    for family, configurations in by_family.items():
        locked = configurations["fixed"]
        informed = configurations["deployment_fixed"]
        differing = {key for key in locked if locked[key] != informed[key]}
        assert differing <= {"acoustic_technique"}, (family, differing)


def test_plan_enumeration_is_deterministic(runner):
    first = runner.build_plan()[1]
    second = runner.build_plan()[1]
    assert first == second


def test_plan_digest_is_stable(runner):
    _, plan, _ = runner.build_plan()
    key = lambda tasks: runner.digest(
        [{k: t[k] for k in ("family", "index", "policy", "configuration_id")} for t in tasks])
    assert key(plan) == key(runner.build_plan()[1])


def test_execution_parameters_are_inherited_not_chosen(runner):
    """Must equal the frozen development runners; the runner invents nothing."""
    assert runner.HORIZON_S == 180.0
    assert runner.DT_S == 2.0
    assert runner.IMAGE_PERIOD_S == 4.0
    assert runner.REDESIGN_VERSION == 3


def test_runner_refuses_a_non_held_out_root(runner):
    task = {"root": 31_900_000, "stage": "heldout", "family": "S3_NOMINAL",
            "index": 0, "policy": "fixed", "configuration_id": "fixed_155",
            "configuration": {}, "group": "primary"}
    with pytest.raises(RuntimeError, match="refuses a non-held-out root"):
        runner.execute(task)


def test_guards_are_all_present(runner):
    """Every declared precondition must be implemented, not merely documented."""
    source = (STUDY3 / "run_heldout.py").read_text()
    assert "--require-authorization" in source
    assert "PINNED_INTERPRETER" in source and "PINNED_OPENCV" in source
    assert "COMPLETION_MARKER" in source
    assert 'authorization.get("executed")' in source


def test_completion_marker_is_the_second_invocation_guard(runner):
    """The marker verify_lock already treats as forbidden output blocks re-runs."""
    assert runner.COMPLETION_MARKER.name == "heldout_result.json"
    verify_lock = (STUDY3 / "verify_lock.py").read_text()
    assert "heldout_result.json" in verify_lock


def test_held_out_was_executed_exactly_once(runner):
    """The one-shot block is spent. This asserts exactly one, never a second.

    Before execution this test asserted that no completed run existed. It was
    updated deliberately, once, when the authorized single invocation completed
    on 2026-08-11. It must never be relaxed again: a second invocation is a
    protocol violation, not a test failure to be fixed.
    """
    assert runner.COMPLETION_MARKER.exists(), "completed held-out evidence is missing"
    marker = json.loads(runner.COMPLETION_MARKER.read_text())
    authorization = json.loads(runner.AUTHORIZATION.read_text())
    assert authorization["executed"] is True
    assert authorization["maximum_invocations"] == 1
    assert authorization["execution_record"]["invocation"] == 1
    assert authorization["execution_record"]["packets"] == marker["executions"] == 810


def test_a_second_invocation_is_refused(runner):
    """The guards that make the one-shot irreversible must still hold."""
    failures = runner.check_guards()
    assert failures, "a second held-out invocation would not be refused"
    assert any("second invocation is forbidden" in f or "already" in f for f in failures)
