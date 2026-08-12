"""Automatic run provenance for platform-v2 outputs."""

from __future__ import annotations

import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys

import numpy as np


def sha256_file(path):
    digest=hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""): digest.update(block)
    return digest.hexdigest()


def _commit(repository):
    result=subprocess.run(["git","rev-parse","HEAD"],cwd=repository,
                          text=True,capture_output=True,check=False)
    return result.stdout.strip() if result.returncode==0 else None


def build_run_manifest(repository,benchmark_path,configuration,output_path=None):
    benchmark_path=Path(benchmark_path); repository=Path(repository)
    benchmark=json.loads(benchmark_path.read_text())
    manifest={
      "schema_version":1,"benchmark":benchmark.get("benchmark",benchmark.get("identifier")),
      "benchmark_sha256":sha256_file(benchmark_path),"commit":_commit(repository),
      "python":sys.version.split()[0],"numpy":np.__version__,"platform":platform.platform(),
      "configuration":configuration,
    }
    if output_path is not None:
        manifest["output_sha256"]=sha256_file(output_path)
    return manifest
