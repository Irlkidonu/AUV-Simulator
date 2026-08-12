"""Bitwise regression of the v2 copy against development-only golden rows."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

from uuv_mode_aware_navigation.availability import AvailabilityModel
from uuv_mode_aware_navigation.campaign import (
    BASELINE_TERRAIN_GRADIENT,
    NoiseProfile,
    Scenario,
    TerrainProfile,
    run_scenario,
)
from uuv_mode_aware_navigation.comparators import (
    FixedPolicy,
    ProposedPolicy,
    ResidualOnlyPolicy,
)
from uuv_mode_aware_navigation.imaging import OpticalFeedback
from uuv_mode_aware_navigation.manager import DEFAULT_CANDIDATES


ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = ROOT / "src" / "uuv_mode_aware_navigation"
GOLDEN = ROOT / "benchmarks" / "study2_legacy" / "golden" / "smoke_runs.csv"
MODELS = PACKAGE_ROOT / "models"
FIXED_NAME = "lidar+terrain_relative@1.0m/0.25mps/weight/continue"


def _campaign_script():
    scripts = PACKAGE_ROOT / "scripts"
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location("v2_run_campaign", scripts / "run_campaign.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAMPAIGN_SCRIPT = _campaign_script()
FAMILIES = {entry[0]: entry for entry in CAMPAIGN_SCRIPT.scenario_family()}
AVAILABILITY = AvailabilityModel.from_dict(
    json.loads((MODELS / "availability.json").read_text())
)
OPTICAL_FEEDBACK = OpticalFeedback.from_dict(
    json.loads((MODELS / "optical_feedback.json").read_text())
)
FIXED_CONFIG = next(c for c in DEFAULT_CANDIDATES if c.name == FIXED_NAME)


with GOLDEN.open(newline="") as handle:
    GOLDEN_ROWS = list(csv.DictReader(handle))


def _scenario(row: dict[str, str]) -> Scenario:
    entry = FAMILIES[row["scenario"]]
    return Scenario(
        name=entry[0],
        seed=int(row["seed"]),
        water=entry[1],
        schedule=entry[2],
        current=entry[3] if len(entry) > 3 else CAMPAIGN_SCRIPT.BASELINE_CURRENT,
        noise=entry[4] if len(entry) > 4 else NoiseProfile.constant(40.0),
        terrain=entry[5] if len(entry) > 5 else TerrainProfile.constant(
            BASELINE_TERRAIN_GRADIENT
        ),
        prior_map=entry[6] if len(entry) > 6 else True,
    )


def _policy(name: str):
    if name == "proposed":
        return ProposedPolicy(AVAILABILITY)
    if name == "fixed":
        return FixedPolicy(FIXED_CONFIG)
    if name == "residual_only":
        return ResidualOnlyPolicy()
    raise AssertionError(name)


def _assert_field_equal(name: str, expected: str, actual) -> None:
    if expected == "":
        assert actual is None, name
    elif expected in {"True", "False"}:
        assert actual is (expected == "True"), name
    elif isinstance(actual, bool):
        assert actual is (expected == "True"), name
    elif isinstance(actual, int):
        assert actual == int(expected), name
    elif isinstance(actual, float):
        target = float(expected)
        assert math.isclose(actual, target, rel_tol=0.0, abs_tol=0.0), (
            name, actual, target
        )
    else:
        assert str(actual) == expected, name


@pytest.mark.parametrize(
    "golden",
    GOLDEN_ROWS,
    ids=lambda row: f"{row['scenario']}-{row['policy']}-{row['seed']}",
)
def test_development_smoke_run_is_bitwise(golden: dict[str, str]) -> None:
    result = run_scenario(
        _scenario(golden),
        _policy(golden["policy"]),
        optical_feedback=OPTICAL_FEEDBACK,
    ).to_row()
    result["policy"] = golden["policy"]
    assert result.keys() == golden.keys()
    for field, expected in golden.items():
        _assert_field_equal(field, expected, result[field])
