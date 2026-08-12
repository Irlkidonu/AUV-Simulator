import importlib.util
import json
import math
from pathlib import Path

import numpy as np

from uuv_mode_aware_navigation.localization import P5V4ImageLocalizer


ROOT=Path(__file__).resolve().parents[4]
RUN=ROOT/"experiments/platform_v2/p5_spike_v4/run.py"
SPEC=importlib.util.spec_from_file_location("selected_p5_v4",RUN)
SELECTED=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(SELECTED)


def test_packaged_frontend_matches_selected_confirmation_on_stratified_subset():
    manifest=json.loads((ROOT/"experiments/platform_v2/p5_spike_v4/confirmation_manifest.json").read_text())
    SELECTED.configure(manifest["seed_root"])
    worlds={k:SELECTED.BASE._world(k,SELECTED.BASE._seed(f"world:{k}"))
            for k in ("normal","repeated","feature_poor")}
    camera=SELECTED.BASE.CameraModel(192,192,SELECTED.BASE.FOV)
    renderers={k:SELECTED.BASE.GeoreferencedRenderer(v,camera,SELECTED.BASE._seed(f"sensor:{k}"),True)
               for k,v in worlds.items()}
    localizer=P5V4ImageLocalizer()
    # Six representatives per stratum: enough to cover accept/reject stages
    # without turning the unit suite into another 600-pair campaign.
    selected=[pair for index,pair in enumerate(manifest["pairs"]) if index%10==0]
    assert len(selected)==60
    for pair in selected:
        kind=pair["kind"];base="feature_poor" if kind=="feature_independent" else kind
        a,b=SELECTED.BASE._pose(pair["a"]),SELECTED.BASE._pose(pair["b"])
        water=SELECTED.BASE.WaterState(c=pair["attenuation"])
        first=renderers[base].render(a,water,SELECTED.BASE.CAMERA_OFFAXIS)
        if kind=="feature_independent":
            second=SELECTED.BASE.GeoreferencedRenderer(
                SELECTED.BASE._world("feature_poor",pair["world_b_seed"]),camera,
                pair["world_b_seed"]+1,True).render(b,water,SELECTED.BASE.CAMERA_OFFAXIS)
        else:second=renderers[base].render(b,water,SELECTED.BASE.CAMERA_OFFAXIS)
        expected=SELECTED.estimate(first,second,a,b,2.3)
        actual=localizer.localize(first,second,a)
        assert actual.localization_success==expected["localization_success"]
        for key in ("keypoints_a","keypoints_b","matches"):
            assert actual.metrics.get(key)==expected.get(key)
        if "inliers" in expected:
            assert actual.metrics["inliers"]==expected["inliers"]
            assert math.isclose(actual.metrics["inlier_fraction"],expected["inlier_fraction"],abs_tol=1e-12)
        if actual.localization_success:
            error=np.linalg.norm([actual.estimated_pose.x_m-b.x_m,actual.estimated_pose.y_m-b.y_m])
            assert math.isclose(error,expected["translation_error_m"],abs_tol=1e-10)
            assert np.allclose(actual.covariance_m2,expected["covariance_m2"],rtol=1e-10,atol=1e-12)
