"""Experiment protocols and the immutable P5-v1 failure are checksummed."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
EXPERIMENTS = ROOT / "experiments" / "platform_v2"


def test_platform_v2_experiment_checksums() -> None:
    for line in (EXPERIMENTS / "SHA256SUMS").read_text().splitlines():
        if not line.strip():
            continue
        expected, relative = line.split(maxsplit=1)
        payload = (EXPERIMENTS / relative).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == expected, relative


def test_p5_v2_has_protocol_but_no_result() -> None:
    spike = EXPERIMENTS / "p5_spike_v2"
    assert (spike / "PROTOCOL.md").is_file()
    assert not (spike / "result.json").exists()


def test_p5_v3_is_a_single_completed_scientific_failure() -> None:
    spike = EXPERIMENTS / "p5_spike_v3"
    attempt = json.loads((spike / "ATTEMPT.json").read_text())
    result = json.loads((spike / "result.json").read_text())
    assert attempt["attempt_number"] == attempt["maximum_executions"] == 1
    assert attempt["status"] == result["status"] == "FAIL"
    assert result["seed_root"] == 22_120_000
    assert sum(len(rows) for rows in result["raw"].values()) == 600


def test_p5_v4_confirmation_is_development_only_and_single_attempt() -> None:
    spike = EXPERIMENTS / "p5_spike_v4"
    attempt = json.loads((spike / "CONFIRMATION_ATTEMPT.json").read_text())
    result = json.loads((spike / "confirmation_result.json").read_text())
    assert attempt["attempt_number"] == attempt["maximum_executions"] == 1
    assert "not held-out" in attempt["data_class"]
    assert attempt["status"] == result["status"] == "DEVELOPMENT PASS"
    assert result["seed_root"] == 22_140_000
    assert result["negative_false_fixes"] == 0
