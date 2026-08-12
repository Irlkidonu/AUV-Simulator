import importlib.util
import subprocess
import sys
from pathlib import Path


RUNNER=(Path(__file__).resolve().parents[4]/"experiments"/"study3"/
        "run_redesign_development.py")


def load_runner():
    spec=importlib.util.spec_from_file_location("study3_confirmation_runner",RUNNER)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_confirmation_constructs_locked_fixed155_identity_for_all_wrappers():
    runner=load_runner()
    tasks,(fixed,reactive,predictive)=runner.infrastructure_tasks(
        "infrastructure_confirmation",31_820_000,range(17))
    assert len(tasks)==680
    assert {task[1] for task in tasks}=={31_820_000}
    assert {task[5] for task in tasks}=={
        "fixed_155","robust_fusion_fixed_155",
        "reactive_shared_fixed_155","predictive_shared_fixed_155"}
    keys=("optical_channel","altitude_m","speed_mps","acoustic_technique","fusion_mode")
    expected={key:fixed[key] for key in keys}
    assert expected=={"optical_channel":"lidar","altitude_m":5.0,"speed_mps":0.5,
                      "acoustic_technique":"usbl","fusion_mode":"weight"}
    assert {key:reactive[key] for key in keys}==expected
    assert {key:predictive[key] for key in keys}==expected


def test_smoke_is_four_packets_and_cannot_collide_with_confirmation():
    runner=load_runner()
    tasks,_=runner.infrastructure_tasks(
        "infrastructure_smoke",31_819_000,range(1),(runner.FAMILIES[0],))
    assert len(tasks)==4
    assert {task[0] for task in tasks}=={"infrastructure_smoke"}
    assert {task[1] for task in tasks}=={31_819_000}


def test_v3_development_comparison_uses_fresh_root_and_locked_fixed_155():
    runner=load_runner()
    assert runner.ROOTS["mode_comparison_v3"]==31_850_000
    tasks,(fixed,reactive,predictive)=runner.infrastructure_tasks(
        "mode_comparison_v3",31_850_000,range(17))
    assert len(tasks)==10*17*4
    expected={"optical_channel":"lidar","altitude_m":5.0,"speed_mps":0.5,
              "acoustic_technique":"usbl","fusion_mode":"weight"}
    keys=tuple(expected)
    for configuration in (fixed,reactive,predictive):
        assert {key:configuration[key] for key in keys}==expected
    assert {task[5] for task in tasks}=={
        "fixed_155","robust_fusion_fixed_155",
        "reactive_shared_fixed_155","predictive_shared_fixed_155"}
    assert {task[1] for task in tasks}=={31_850_000}


def test_runner_documented_direct_invocation_imports_without_pythonpath():
    """The repository-root command must reach argparse without manual setup."""
    repository=Path(__file__).resolve().parents[4]
    runner=repository/"experiments/study3/run_redesign_development.py"
    completed=subprocess.run(
        [sys.executable,str(runner),"--help"],cwd=repository,
        env={"PATH":"/usr/bin:/bin"},capture_output=True,text=True,check=False)
    assert completed.returncode==0,completed.stderr
    assert "mode_comparison_v3" in completed.stdout


def test_discovery_fairness_constructs_exact_paired_510_packet_design():
    runner=load_runner()
    assert runner.ROOTS["discovery_fairness_v1"]==31_880_000
    tasks,(fixed,reactive,deployment)=runner.discovery_fairness_tasks(
        31_880_000,range(17))
    assert len(tasks)==10*17*3
    assert len({(t[2],t[3],t[4]) for t in tasks})==len(tasks)
    assert {t[1] for t in tasks}=={31_880_000}
    assert {t[4] for t in tasks}=={"fixed","deployment_fixed","reactive"}
    locked={"optical_channel":"lidar","altitude_m":5.0,"speed_mps":.5,
            "acoustic_technique":"usbl","fusion_mode":"weight"}
    assert {k:fixed[k] for k in locked}==locked
    assert {k:reactive[k] for k in locked}==locked
    for family,configuration in deployment.items():
        for key,value in locked.items():
            if key!="acoustic_technique":assert configuration[key]==value
    assert deployment["S3_OPTICAL_GRADUAL"]["acoustic_technique"]=="none"
    assert deployment["S3_ACOUSTIC_GEOMETRY_ASYNC"]["acoustic_technique"]=="lbl"
    assert deployment["S3_INFRASTRUCTURE_WARNING"]["acoustic_technique"]=="usbl"
