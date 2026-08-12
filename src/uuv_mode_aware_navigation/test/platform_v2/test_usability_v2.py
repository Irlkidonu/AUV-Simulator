import json
from pathlib import Path

from uuv_mode_aware_navigation.cli import main
from uuv_mode_aware_navigation.provenance import build_run_manifest,sha256_file


ROOT=Path(__file__).resolve().parents[4]


def test_cli_lists_benchmarks_without_ros(capsys) -> None:
    assert main(["list-benchmarks"])==0
    text=capsys.readouterr().out
    assert "study2_legacy_v1.0" in text and "platform_v2_dev" in text


def test_cli_status_discloses_adverse_and_unspent_evidence(capsys) -> None:
    assert main(["platform-status"])==0
    text=capsys.readouterr().out
    assert "P6-v2: FAIL" in text
    assert "held-out evaluation: not executed" in text


def test_cli_validates_safe_development_system_configuration(capsys) -> None:
    path=ROOT/"benchmarks/platform_v2_system_integration.json"
    assert main(["validate-platform-config",str(path)])==0
    configuration=json.loads(capsys.readouterr().out)
    assert configuration["optical_frontend"]=="p5_v4"
    assert not configuration["held_out"]


def test_manifest_captures_benchmark_and_output_hash(tmp_path) -> None:
    output=tmp_path/"result.json"; output.write_text('{"ok": true}\n')
    benchmark=ROOT/"benchmarks"/"platform_v2.json"
    manifest=build_run_manifest(ROOT,benchmark,{"policy":"development"},output)
    assert manifest["benchmark"]=="platform_v2_dev"
    assert manifest["benchmark_sha256"]==sha256_file(benchmark)
    assert manifest["output_sha256"]==sha256_file(output)
    json.dumps(manifest)
