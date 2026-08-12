import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
PATH = ROOT / "experiments/platform_v2/p5_spike_v4/run.py"
SPEC = importlib.util.spec_from_file_location("p5v4", PATH)
P5 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(P5)


def test_direct_support_accepts_broad_two_dimensional_geometry():
    x, y = np.meshgrid(np.linspace(30, 160, 5), np.linspace(30, 160, 5))
    support = P5._support(np.column_stack((x.ravel(), y.ravel())))
    assert support["span_x"] >= 32
    assert support["span_y"] >= 32
    assert support["grid4"] >= 3
    assert support["point_condition"] <= 4
    assert support["hull"] >= 0.05


def test_direct_support_exposes_collinear_degeneracy():
    x = np.linspace(20, 170, 25)
    support = P5._support(np.column_stack((x, 0.2 * x + 40)))
    assert support["point_condition"] > 4
    assert support["hull"] < 0.10


def test_manifest_roots_are_deterministic_and_disjoint():
    calibration = P5.build_manifest(22_130_000)
    confirmation = P5.build_manifest(22_140_000)
    assert calibration == P5.build_manifest(22_130_000)
    assert calibration["identifier"] != confirmation["identifier"]
    assert calibration["pairs"] != confirmation["pairs"]
    assert len(calibration["pairs"]) == len(confirmation["pairs"]) == 600
