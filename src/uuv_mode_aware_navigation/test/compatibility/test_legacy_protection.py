"""Additive guards around the immutable Study 2 implementation.

These tests live outside the legacy freeze glob.  They therefore protect the
published numerical path without rewriting the historical freeze record merely
because a protection test was added later.
"""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

from uuv_mode_aware_navigation.campaign import (
    DEVELOPMENT_SEED_ROOT,
    HELDOUT_SEED_ROOTS,
)
from uuv_mode_aware_navigation.sensors import SensorSuite


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
FREEZE = json.loads((PACKAGE_ROOT / "freeze_record.json").read_text())

CRITICAL_MODULES = (
    "acoustics.py",
    "availability.py",
    "campaign.py",
    "comparators.py",
    "environment.py",
    "estimator.py",
    "imaging.py",
    "manager.py",
    "mission.py",
    "modes.py",
    "optics.py",
    "sensors.py",
)

EXPECTED_FAMILIES = (
    "E1_nominal",
    "E2_dvl_short",
    "E3_dvl_long",
    "E4_optical_graded",
    "E5_optical_loss",
    "E6_acoustic_intermittent",
    "E7_compound",
    "E8_turbid_dvl_loss",
    "E9_current_unobservable",
    "E10_current_steady",
    "E11_current_building",
    "E12_current_rotating",
    "E13_acoustic_noise",
    "E14_noisy_dvl_loss",
    "E15_turbid_and_noisy",
    "E16_featureless_plain",
    "E17_terrain_recoverable",
    "E18_vessel_departs",
    "E19_unprepared_area",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_campaign_script():
    path = PACKAGE_ROOT / "scripts" / "run_campaign.py"
    scripts = str(path.parent)
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("legacy_run_campaign", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_campaign_critical_modules_match_historical_freeze() -> None:
    """P0.2: verify scientific modules, independently of demonstrator drift."""
    frozen = FREEZE["digests"]
    assert len(CRITICAL_MODULES) == 12
    for name in CRITICAL_MODULES:
        relative = f"uuv_mode_aware_navigation/{name}"
        assert frozen[relative] == _sha256(PACKAGE_ROOT / relative), relative


def test_seed_roots_and_positional_derivation_are_pinned() -> None:
    """P0.3: protect reserved roots and the published positional seed rule."""
    assert DEVELOPMENT_SEED_ROOT == 20_000_000
    assert HELDOUT_SEED_ROOTS == (20_400_000, 20_800_000)
    assert DEVELOPMENT_SEED_ROOT + 1000 + 7 == 20_001_007
    assert HELDOUT_SEED_ROOTS[-1] + 1000 + 18 == 20_801_018


def test_scenario_family_order_is_pinned() -> None:
    """P0.4: a reorder must fail because family position determines the seed."""
    module = _load_campaign_script()
    assert tuple(row[0] for row in module.scenario_family()) == EXPECTED_FAMILIES


def test_sensor_sample_invocation_order_is_pinned() -> None:
    """P0.5: cosmetic reordering of shared-RNG samplers must be detected."""
    source = (PACKAGE_ROOT / "uuv_mode_aware_navigation" / "sensors.py").read_text()
    tree = ast.parse(source)
    sensor_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "SensorSuite"
    )
    sample = next(
        node for node in sensor_class.body
        if isinstance(node, ast.FunctionDef) and node.name == "sample"
    )
    calls = [
        node.func.attr
        for node in ast.walk(sample)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "self"
        and node.func.attr in {
            "_dvl", "_dvl_water_track", "_optical", "_acoustic",
            "_inertial", "_depth", "_reported_quality",
        }
    ]
    calls.sort(key=lambda name: next(
        node.lineno for node in ast.walk(sample)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == name
    ))
    assert calls == [
        "_dvl", "_dvl_water_track", "_optical", "_acoustic",
        "_inertial", "_depth", "_reported_quality",
    ]


def test_sensor_initial_rng_draws_are_pinned() -> None:
    """P0.5: pin scenario-constant draws made before the first sample."""
    suite = SensorSuite(seed=20_000_123)
    np.testing.assert_array_equal(
        suite._accel_bias,
        np.array([
            float.fromhex("-0x1.ca61d6e001e84p-10"),
            float.fromhex("-0x1.0200084e70d48p-11"),
            float.fromhex("-0x1.00c5055283703p-8"),
        ]),
    )
    assert suite._dvl_scale == float.fromhex("0x1.0021c29bd89f6p+0")
    np.testing.assert_array_equal(
        suite._dvl_rotation,
        np.array([
            [float.fromhex("0x1.fff3c39d2f28cp-1"),
             float.fromhex("-0x1.bfbb38a57acdep-7"), 0.0],
            [float.fromhex("0x1.bfbb38a57acdep-7"),
             float.fromhex("0x1.fff3c39d2f28cp-1"), 0.0],
            [0.0, 0.0, 1.0],
        ]),
    )
