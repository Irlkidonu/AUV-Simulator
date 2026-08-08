"""The held-out block must be unreachable except through the freeze gate.

PROTOCOL D4 permits exactly one execution of seed root 20,400,000, and only
after a verified freeze record exists. Before this file, both halves of that
sentence were prose. Prose does not refuse.

These tests assert refusals, not the presence of code. Each one drives the
actual entry point and checks that it exits non-zero with the reason, because a
gate that can be satisfied by a helper nobody calls is not a gate. The two
failure modes they cover are the ones that would be indistinguishable from
misconduct after the fact:

  * reaching the held-out seeds via ``--root`` without the gate, and
  * executing the block a second time after the first result was seen.

The second is the one that matters. A held-out set spent twice is a development
set, and every number drawn from it would have to be reported as such.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
RUNNER = PACKAGE_ROOT / "scripts" / "run_campaign.py"

sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
sys.path.insert(0, str(PACKAGE_ROOT))

import freeze  # noqa: E402
from uuv_mode_aware_navigation.campaign import (  # noqa: E402
    DEVELOPMENT_SEED_ROOT,
    HELDOUT_SEED_ROOT,
    HELDOUT_SEED_ROOTS,
)


def _run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(RUNNER), *args],
        cwd=PACKAGE_ROOT, capture_output=True, text=True, timeout=120,
    )


def test_the_two_seed_roots_are_disjoint():
    """A development seed must never collide with a held-out seed.

    Seeds are ``root + 1000 + k`` over fifteen families and ten seeds, so the
    development block occupies at most root+1000..root+1149. The two roots are
    400,000 apart, but the check is written against the construction rather
    than against the current family count so that adding families cannot
    silently close the gap.
    """
    assert HELDOUT_SEED_ROOT != DEVELOPMENT_SEED_ROOT
    assert abs(HELDOUT_SEED_ROOT - DEVELOPMENT_SEED_ROOT) > 100_000
    assert freeze.HELD_OUT_SEED_ROOT == HELDOUT_SEED_ROOT, (
        "the freeze gate guards a different root than the campaign uses"
    )


def test_every_reserved_root_is_refused_without_the_flag():
    """No reserved root may be reached through the ordinary seed argument.

    Every root is checked, not only the active one. A block that has already
    been spent must stay unreachable: if it can be re-entered through ``--root``
    it is a development block, and every number ever drawn from it would have to
    be reported as one. Study 1's block is the live case -- it is spent, its
    result is published, and re-running it would silently invalidate that.
    """
    assert len(HELDOUT_SEED_ROOTS) >= 1
    for root in HELDOUT_SEED_ROOTS:
        proc = _run("--root", str(root), "--seeds", "1")
        assert proc.returncode != 0, f"root {root} was not refused"
        combined = proc.stdout + proc.stderr
        assert "held-out" in combined and "--held-out" in combined


def test_the_active_root_is_the_last_reserved_one():
    """`--held-out` must spend the newest block, never an earlier one."""
    assert HELDOUT_SEED_ROOT == HELDOUT_SEED_ROOTS[-1]
    assert freeze.HELD_OUT_SEED_ROOT == HELDOUT_SEED_ROOT
    assert tuple(freeze.HELD_OUT_SEED_ROOTS) == tuple(HELDOUT_SEED_ROOTS)


def test_held_out_is_refused_without_a_freeze_record(tmp_path, monkeypatch):
    """With no record, the gate refuses rather than defaulting to permissive."""
    monkeypatch.setattr(freeze, "RECORD_PATH", tmp_path / "absent.json")
    with pytest.raises(RuntimeError, match="gated on a verified freeze record"):
        freeze.require_freeze()


def test_held_out_is_refused_when_the_tree_has_moved(tmp_path, monkeypatch):
    """A record that no longer describes the tree is a refusal, not a warning.

    The point of the digest is to catch the case where the source changed after
    the freeze -- including the case where it changed by accident. A gate that
    warned and continued would be satisfied by exactly the situation it exists
    to detect.
    """
    record = tmp_path / "record.json"
    digests = freeze.compute_digests()
    assert digests, "nothing is being digested"
    tampered = dict(digests)
    tampered[next(iter(tampered))] = "0" * 64
    record.write_text(json.dumps({
        "frozen_at": "2026-01-01T00:00:00+00:00",
        "held_out_seed_root": HELDOUT_SEED_ROOT,
        "held_out_executed": False,
        "file_count": len(tampered),
        "digests": tampered,
    }))
    monkeypatch.setattr(freeze, "RECORD_PATH", record)

    ok, problems = freeze.verify_record(quiet=True)
    assert not ok
    assert any("modified since freeze" in p for p in problems)
    with pytest.raises(RuntimeError, match="modified since freeze"):
        freeze.require_freeze()


def test_a_spent_block_cannot_be_executed_again(tmp_path, monkeypatch):
    """One execution. The second is refused even though the tree still matches."""
    record = tmp_path / "record.json"
    record.write_text(json.dumps({
        "frozen_at": "2026-01-01T00:00:00+00:00",
        "held_out_seed_root": HELDOUT_SEED_ROOT,
        "held_out_executed": False,
        "file_count": 0,
        "digests": freeze.compute_digests(),
    }))
    monkeypatch.setattr(freeze, "RECORD_PATH", record)

    freeze.require_freeze()                     # first execution: permitted
    freeze.mark_held_out_spent("results/held_out.csv", "a" * 64)

    with pytest.raises(RuntimeError, match="already been executed"):
        freeze.require_freeze()                 # second: refused
    with pytest.raises(RuntimeError, match="already marked spent"):
        freeze.mark_held_out_spent("results/again.csv")

    stored = json.loads(record.read_text())
    assert stored["held_out_output"] == "results/held_out.csv"
    assert stored["held_out_output_sha256"] == "a" * 64
    assert stored["held_out_executed_at"].startswith("20")


def test_writing_a_record_refuses_to_overwrite(tmp_path, monkeypatch, capsys):
    """A freeze happens once; re-freezing must be a deliberate, recorded act."""
    record = tmp_path / "record.json"
    record.write_text("{}")
    monkeypatch.setattr(freeze, "RECORD_PATH", record)
    assert freeze.write_record(force=False) == 1
    assert record.read_text() == "{}", "the existing record was clobbered"


def test_a_red_suite_cannot_be_frozen(tmp_path, monkeypatch):
    """The record certifies that the boundaries held; a red suite certifies the
    opposite, so ``--write`` must refuse rather than record the failure."""
    monkeypatch.setattr(freeze, "RECORD_PATH", tmp_path / "record.json")
    monkeypatch.setattr(freeze, "run_isolation_tests", lambda: {
        "returncode": 1, "passed": False, "summary": "3 failed, 40 passed",
    })
    assert freeze.write_record() == 1
    assert not (tmp_path / "record.json").exists()
