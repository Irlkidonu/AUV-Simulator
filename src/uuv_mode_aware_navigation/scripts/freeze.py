#!/usr/bin/env python3
"""Write and verify the freeze record, and gate held-out execution on it.

`PROTOCOL.md` §9 states that at the freeze a machine-readable record is written
containing SHA-256 digests of the source, the parameter sets, the scenario
definitions, the seed registry, the metric definitions and the isolation-test
results, and that **held-out execution is programmatically gated on a matching,
verified freeze record**.

Until this file existed, that gate was a sentence in a document. Nothing
computed a digest and nothing refused to run. A protocol clause that no code
enforces is an intention, not a control, and the difference only becomes visible
at the moment it is needed --- which for a held-out set is exactly once.

Usage::

    PYTHONPATH=. python3 scripts/freeze.py --write     # at the freeze
    PYTHONPATH=. python3 scripts/freeze.py --verify    # any time after
    PYTHONPATH=. python3 scripts/freeze.py --status    # human summary

and, from code that wants to touch the held-out root::

    from freeze import require_freeze
    require_freeze()          # raises unless the tree matches the record

What is deliberately *not* here: any ability to update a record in place. To
change the frozen design you delete the record and write a new one, which
records a new timestamp and leaves the old digests in the git history. There is
no path that silently re-baselines a mismatch, because that is the single
failure mode that would make the whole mechanism decorative.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
RECORD_PATH = PACKAGE_ROOT / "freeze_record.json"

# Everything whose change would alter a reported number. Scenario definitions,
# the seed registry, the metric and aggregation definitions and the parameter
# set of every method and comparator all live inside the package source and the
# campaign script, so digesting those trees covers PROTOCOL §9's list without
# needing a second, hand-maintained enumeration that could drift out of date.
DIGESTED = (
    "uuv_mode_aware_navigation/*.py",
    "uuv_mode_aware_navigation/nodes/*.py",
    "scripts/*.py",
    "test/*.py",
    "models/*.json",
    "worlds/*.sdf",
    "launch/*.launch.py",
    "setup.py",
    "package.xml",
)

# The held-out roots, each of which PROTOCOL D4 permits to be executed exactly
# once and only after a verified freeze record exists. Study 1's block is spent;
# the last entry is the one the current freeze governs.
HELD_OUT_SEED_ROOTS = (20_400_000, 20_800_000)
HELD_OUT_SEED_ROOT = HELD_OUT_SEED_ROOTS[-1]


def _iter_files() -> list[Path]:
    seen: set[Path] = set()
    for pattern in DIGESTED:
        for path in sorted(PACKAGE_ROOT.glob(pattern)):
            if path.is_file() and "__pycache__" not in path.parts:
                seen.add(path)
    return sorted(seen)


def _digest(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def compute_digests() -> dict[str, str]:
    return {
        str(p.relative_to(PACKAGE_ROOT)): _digest(p) for p in _iter_files()
    }


def _git_commit() -> str | None:
    """Record the commit, but never rely on it.

    A digest of the working tree is the authority here, because the tree can
    differ from any commit. The commit is recorded only so a reader can find the
    corresponding history.
    """
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=PACKAGE_ROOT,
            capture_output=True, text=True, timeout=10,
        )
        return out.stdout.strip() or None if out.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def run_isolation_tests() -> dict:
    """Run the suite and record the outcome verbatim.

    PROTOCOL §9 requires the isolation-test results to be *in* the record. A
    freeze taken while the boundary tests are failing would certify a tree whose
    guidance may be consuming truth, so the result is stored and `--write`
    refuses on failure rather than recording a red result and continuing.
    """
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "--no-header", "test"],
        cwd=PACKAGE_ROOT, capture_output=True, text=True,
    )
    tail = [ln for ln in proc.stdout.strip().splitlines() if ln.strip()]
    return {
        "returncode": proc.returncode,
        "passed": proc.returncode == 0,
        "summary": tail[-1] if tail else "(no output)",
    }


def write_record(force: bool = False) -> int:
    if RECORD_PATH.exists() and not force:
        print(f"refusing to overwrite an existing freeze record: {RECORD_PATH}")
        print("A freeze happens once. Delete the file deliberately if the "
              "design genuinely changed, so the replacement is a recorded act.")
        return 1

    print("running the isolation tests before digesting anything...")
    tests = run_isolation_tests()
    print(f"  {tests['summary']}")
    if not tests["passed"]:
        print("\nrefusing to freeze: the test suite is not green.")
        print("A freeze record certifies that the boundaries held at freeze "
              "time. Recording a red suite would certify the opposite.")
        return 1

    digests = compute_digests()
    record = {
        "frozen_at": datetime.now(timezone.utc).isoformat(),
        "git_commit": _git_commit(),
        "held_out_seed_root": HELD_OUT_SEED_ROOT,
        "held_out_executed": False,
        "isolation_tests": tests,
        "file_count": len(digests),
        "digests": digests,
    }
    RECORD_PATH.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(f"\nwrote {RECORD_PATH} covering {len(digests)} files")
    return 0


def verify_record(quiet: bool = False) -> tuple[bool, list[str]]:
    """Compare the tree against the record. Returns (ok, differences)."""
    if not RECORD_PATH.exists():
        return False, [f"no freeze record at {RECORD_PATH}"]

    record = json.loads(RECORD_PATH.read_text())
    frozen = record.get("digests", {})
    current = compute_digests()

    problems = []
    for name in sorted(set(frozen) | set(current)):
        if name not in current:
            problems.append(f"deleted since freeze: {name}")
        elif name not in frozen:
            problems.append(f"added since freeze:   {name}")
        elif frozen[name] != current[name]:
            problems.append(f"modified since freeze: {name}")

    if not quiet:
        if problems:
            print(f"freeze record does NOT match the tree "
                  f"({len(problems)} difference(s)):")
            for p in problems:
                print(f"  {p}")
        else:
            print(f"freeze record matches the tree "
                  f"({len(frozen)} files, frozen {record['frozen_at']})")
    return not problems, problems


def require_freeze() -> dict:
    """Gate for held-out execution. Raises unless the tree matches the record.

    PROTOCOL D4 allows the held-out seeds to be executed once, after the freeze
    record exists. This is the check that makes that enforceable rather than
    remembered.
    """
    ok, problems = verify_record(quiet=True)
    if not ok:
        raise RuntimeError(
            "held-out execution is gated on a verified freeze record, and the "
            "check failed:\n  " + "\n  ".join(problems)
        )
    record = json.loads(RECORD_PATH.read_text())
    if record.get("held_out_executed"):
        raise RuntimeError(
            "the held-out set has already been executed, on "
            f"{record.get('held_out_executed_at', 'an unrecorded date')}. "
            "PROTOCOL D4 permits exactly one execution: a second would make it "
            "a development set, and every number drawn from it would need to "
            "be reported as such."
        )
    return record


def mark_held_out_spent(output_path: str, digest: str | None = None) -> None:
    """Record that the held-out block has been executed, and against what.

    Called only after the held-out campaign has written its results, so that a
    crash mid-campaign does not burn the block. This is the one field of the
    record that may be updated in place: everything else describes the frozen
    source, which does not change, whereas this describes an event that had not
    happened when the record was written.

    The digest of the output is stored so that the file quoted in the paper can
    be shown to be the file this execution produced, rather than a later one.
    """
    record = json.loads(RECORD_PATH.read_text())
    if record.get("held_out_executed"):
        raise RuntimeError("the held-out block was already marked spent")
    record["held_out_executed"] = True
    record["held_out_executed_at"] = datetime.now(timezone.utc).isoformat()
    record["held_out_output"] = output_path
    if digest is not None:
        record["held_out_output_sha256"] = digest
    RECORD_PATH.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--write", action="store_true", help="write the record")
    g.add_argument("--verify", action="store_true", help="check the tree")
    g.add_argument("--status", action="store_true", help="human summary")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing record (deliberate re-freeze)")
    args = ap.parse_args()

    if args.write:
        return write_record(force=args.force)
    if args.verify:
        ok, _ = verify_record()
        return 0 if ok else 1

    if not RECORD_PATH.exists():
        print("status: NOT FROZEN")
        print(f"  no record at {RECORD_PATH}")
        print(f"  {len(compute_digests())} files would be covered")
        print("  held-out execution: BLOCKED (no record)")
        return 0
    record = json.loads(RECORD_PATH.read_text())
    ok, problems = verify_record(quiet=True)
    print(f"status: {'FROZEN, tree matches' if ok else 'FROZEN, TREE DIVERGED'}")
    print(f"  frozen at   {record['frozen_at']}")
    print(f"  commit      {record.get('git_commit') or '(not a git tree)'}")
    print(f"  files       {record['file_count']}")
    print(f"  tests       {record['isolation_tests']['summary']}")
    if problems:
        print(f"  differences {len(problems)}")
        for p in problems[:10]:
            print(f"    {p}")
    used = record.get("held_out_executed")
    print(f"  held-out    {'ALREADY EXECUTED' if used else 'unspent'} "
          f"(root {record['held_out_seed_root']})")
    print(f"  held-out execution: {'ALLOWED' if ok and not used else 'BLOCKED'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
