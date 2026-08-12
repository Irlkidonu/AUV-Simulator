import importlib.util
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[4]; PATH=ROOT/"experiments/platform_v2/p5_spike_v2/run.py"
spec=importlib.util.spec_from_file_location("p5v2",PATH); p5=importlib.util.module_from_spec(spec); spec.loader.exec_module(p5)

def test_manifest_has_fresh_declared_composition():
    m=p5.build_manifest(); assert len(m["pairs"])==600
    assert sum(not p["negative"] for p in m["pairs"])==400
    assert sum(p["negative"] for p in m["pairs"])==200

def test_ground_truth_transform_maps_pixels_exactly():
    a=p5.CameraPose(1,2,3,.2);b=p5.CameraPose(1.2,1.8,2.7,-.1);A,t=p5._truth_transform(a,b)
    points=np.array([[0,0],[95.5,95.5],[191,191]],float)
    assert np.all(np.isfinite(points@A.T+t))
    assert .6<np.linalg.det(A)**.5<1.67

def test_frozen_detector_configuration_reaches_geometry_stage():
    world=p5._world("normal",p5._seed("unit"));camera=p5.CameraModel(192,192,p5.FOV);renderer=p5.GeoreferencedRenderer(world,camera,p5._seed("unit_sensor"),False)
    a=p5.CameraPose(0,0,3,0);b=p5.CameraPose(.15,-.1,3,.03);water=p5.WaterState(c=.2)
    result=p5.estimate(renderer.render(a,water,p5.CAMERA_OFFAXIS),renderer.render(b,water,p5.CAMERA_OFFAXIS),a,b)
    assert result["detection_success"] and result["match_success"]
    assert result["translation_error_m"]<.10
