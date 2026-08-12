import importlib.util
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[4];PATH=ROOT/"experiments/platform_v2/p5_spike_v3/run.py"
SPEC=importlib.util.spec_from_file_location("p5v3",PATH);P5=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(P5)

class ReturnsTransform:
    def __init__(self,transform):self.transform=transform
    def estimateAffinePartial2D(self,*args,**kwargs):return self.transform,np.ones((4,1),np.uint8)

def test_zero_scale_transform_is_rejected_not_inverted():
    proxy=P5.SafeCv2Proxy(ReturnsTransform(np.array([[0.,0.,4.],[0.,0.,7.]])))
    assert proxy.estimateAffinePartial2D(None,None)==(None,None)

def test_nonfinite_transform_is_rejected():
    proxy=P5.SafeCv2Proxy(ReturnsTransform(np.array([[np.nan,0.,0.],[0.,1.,0.]])))
    assert proxy.estimateAffinePartial2D(None,None)==(None,None)

def test_valid_similarity_is_preserved_exactly():
    transform=np.array([[.9,-.1,2.],[.1,.9,-3.]])
    proxy=P5.SafeCv2Proxy(ReturnsTransform(transform));actual,mask=proxy.estimateAffinePartial2D(None,None)
    assert np.array_equal(actual,transform) and mask.shape==(4,1)

def test_v3_has_fresh_root_and_unchanged_pair_counts():
    manifest=P5.build_manifest()
    assert manifest["seed_root"]==22_120_000 and manifest["identifier"]=="p2v2_p5_spike_v3"
    assert len(manifest["pairs"])==600
