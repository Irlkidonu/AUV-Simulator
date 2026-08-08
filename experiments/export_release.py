#!/usr/bin/env python3
"""Build the public repository from the working tree.

The working tree is a research workspace: it holds the manuscript, planning
notes, build overlays and scratch alongside the software. A published repository
should hold the software, the evidence needed to check the paper's numbers, and
nothing else. This script produces the second from the first.

It copies rather than moves. The working tree is never modified, so this can be
run while other work is in progress and re-run as often as needed.

    python3 experiments/export_release.py --out ../uuv-mode-aware-navigation

What ships, and why each item is there:

  src/            the ROS 2 package: physics, estimator, manager, comparators,
                  campaign runner, Gazebo world, launch files, tests
  protocol/       the pre-registered protocol and the method specifications the
                  paper cites
  results/        the campaign outputs a reader needs to reproduce any reported
                  number, plus the freeze record that certifies which source
                  produced them
  analysis/       the scripts that turn a campaign CSV into the paper's tables

What does not ship:

  manuscript/     the paper is the publication; its LaTeX source, figures and
                  cover letter are not part of the software release
  PLAN.md,        internal planning and project-management notes
  PROJECT.md
  .build/,        build overlays, logs, caches, scratch
  .install/, log/
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "uuv_mode_aware_navigation"

#: Result files a reader needs in order to check a number in the paper. The
#: static sweeps are large but are the evidence behind the tuned baseline, which
#: is the comparison the whole study rests on; excluding them would leave the
#: strongest available criticism uncheckable.
RESULT_FILES = (
    "freeze_record.json",
    "results/PRE_CAMPAIGN_BASELINE.sha256",
    "results/DEVELOPMENT_NORMALISERS.json",
    # Study 1
    "results/campaign_v5.csv",
    "results/campaign_v5.log",
    "results/held_out.csv",
    "results/held_out.log",
    "results/static_sweep_development_v5.csv",
    "results/static_sweep_held_out.csv",
    # Study 2
    "results/campaign_v7.csv",
    "results/campaign_v7.log",
    "results/held_out_2.csv",
    "results/held_out_2.log",
    # Part B: the full 144-configuration sweep over the held-out block, 54,720
    # runs. This is the file behind the claim that the pre-registered baseline
    # was also the hindsight-best of 144, which is the strongest answer to "you
    # compared against a weak baseline". Shipping the paper without it would
    # leave that unverifiable.
    "results/static_sweep_held_out_2.csv",
    "results/heldout_sweep_partB.log",
)

#: Package subtrees copied wholesale.
PACKAGE_TREES = (
    "uuv_mode_aware_navigation",
    "test",
    "scripts",
    "launch",
    "worlds",
    "models",
    "resource",
)

#: Copied from models/ only if small. models/external holds ~191 MB of
#: downloaded third-party glTF; NOTICE records every source and the exact
#: changes each needs, and the world tolerates their absence, so the release
#: points at them rather than carrying them. Generated meshes are excluded for
#: the same reason: three scripts rebuild them deterministically.
MODEL_EXCLUDE = shutil.ignore_patterns("external", "*.obj", "*.gltf.orig")

PACKAGE_FILES = ("setup.py", "setup.cfg", "package.xml", "pytest.ini", "LICENSE")

EXCLUDE = shutil.ignore_patterns(
    "__pycache__", "*.pyc", "*.pyo", ".pytest_cache", "*.egg-info",
)


def _copy_tree(src: Path, dst: Path) -> None:
    if src.exists():
        shutil.copytree(src, dst, dirs_exist_ok=True, ignore=EXCLUDE)


def build(out: Path) -> None:
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    # --- software -------------------------------------------------------
    pkg_out = out / "src" / "uuv_mode_aware_navigation"
    pkg_out.mkdir(parents=True)
    for tree in PACKAGE_TREES:
        if tree == "models":
            shutil.copytree(PACKAGE / tree, pkg_out / tree,
                            dirs_exist_ok=True, ignore=MODEL_EXCLUDE)
        else:
            _copy_tree(PACKAGE / tree, pkg_out / tree)
    for name in PACKAGE_FILES:
        if (PACKAGE / name).exists():
            shutil.copy2(PACKAGE / name, pkg_out / name)

    # --- protocol and specifications ------------------------------------
    proto = out / "protocol"
    proto.mkdir()
    if (ROOT / "experiments" / "PROTOCOL.md").exists():
        shutil.copy2(ROOT / "experiments" / "PROTOCOL.md", proto / "PROTOCOL.md")
    _copy_tree(ROOT / "method", proto / "method")

    # --- evidence -------------------------------------------------------
    res = out / "results"
    res.mkdir()
    copied = []
    for rel in RESULT_FILES:
        src = PACKAGE / rel
        if src.exists():
            shutil.copy2(src, res / Path(rel).name)
            copied.append(Path(rel).name)

    # --- analysis -------------------------------------------------------
    ana = out / "analysis"
    ana.mkdir()
    for name in ("analyse_campaign.py", "analyse_held_out.py"):
        if (ROOT / "experiments" / name).exists():
            shutil.copy2(ROOT / "experiments" / name, ana / name)

    # --- repository furniture -------------------------------------------
    # The root README is written for a reader arriving from the paper, not for
    # someone working inside the research tree, so it is a separate document
    # rather than a copy of the package one. The package README stays where it
    # is and covers running the software.
    readme = ROOT / "experiments" / "release_README.md"
    if readme.exists():
        shutil.copy2(readme, out / "README.md")
    for name in ("LICENSE", "CITATION.cff"):
        if (PACKAGE / name).exists():
            shutil.copy2(PACKAGE / name, out / name)
    # Third-party attribution travels with the code, not with the paper.
    if (ROOT / "NOTICE").exists():
        shutil.copy2(ROOT / "NOTICE", out / "NOTICE")
    (out / ".gitignore").write_text(
        "__pycache__/\n*.py[cod]\n*.egg-info/\n.pytest_cache/\n"
        "build/\ninstall/\nlog/\n.build/\n.install/\n"
        ".vscode/\n.idea/\n.DS_Store\n*.swp\n"
    )

    print(f"exported to {out}")
    print(f"  package trees : {len(PACKAGE_TREES)}")
    print(f"  result files  : {len(copied)}")
    for c in copied:
        print(f"      {c}")
    missing = [r for r in RESULT_FILES if not (PACKAGE / r).exists()]
    if missing:
        print("  not yet present (expected before the final export):")
        for m in missing:
            print(f"      {m}")

    size = sum(f.stat().st_size for f in out.rglob("*") if f.is_file())
    print(f"  total size    : {size / 1e6:.1f} MB")


def verify(out: Path) -> int:
    """Refuse to ship a tree that leaks working-repository material."""
    problems = []
    for pattern in ("*.tex", "*.bib", "cover_letter*", "PLAN.md", "PROJECT.md",
                    "main.pdf", "*.aux", "*.synctex.gz"):
        for hit in out.rglob(pattern):
            problems.append(f"manuscript or planning material: {hit.relative_to(out)}")
    for hit in out.rglob("__pycache__"):
        problems.append(f"cache directory: {hit.relative_to(out)}")
    if not (out / "src" / "uuv_mode_aware_navigation" / "worlds").exists():
        problems.append("the Gazebo world is missing")
    if not (out / "README.md").exists():
        problems.append("no README.md at the repository root")
    if not (out / "LICENSE").exists():
        problems.append("no LICENSE")
    if not (out / "NOTICE").exists():
        problems.append("no NOTICE, so third-party assets are uncredited")
    if (out / "src" / "uuv_mode_aware_navigation" / "models" / "external").exists():
        problems.append("models/external leaked into the export (~191 MB)")
    partb = out / "results" / "static_sweep_held_out_2.csv"
    if not partb.exists():
        problems.append("the Part B sweep is missing; the baseline claim "
                        "cannot be checked without it")
    size = sum(f.stat().st_size for f in out.rglob("*") if f.is_file())
    if size > 120e6:
        problems.append(f"export is {size/1e6:.0f} MB, which is too large "
                        "for a research repository")

    if problems:
        print("\nexport is NOT ready to publish:")
        for p in problems:
            print(f"  {p}")
        return 1
    print("\nexport looks publishable")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path,
                    default=ROOT.parent / "uuv-mode-aware-navigation")
    ap.add_argument("--verify-only", action="store_true")
    args = ap.parse_args()
    out = args.out.resolve()
    if not args.verify_only:
        build(out)
    return verify(out)


if __name__ == "__main__":
    raise SystemExit(main())
