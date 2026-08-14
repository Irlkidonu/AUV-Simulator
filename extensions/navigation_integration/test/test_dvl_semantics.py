import math
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from uuv_navigation_extension.dvl_semantics import *

R_DB=rpy_rotation_dvl_from_body(*np.radians([1.5,-2.0,3.0]))
LEVER=np.array([0.02,0.0,-0.235])


def result(v_nav,yaw=0.0,omega=(0,0,0)):
    q=[0,0,math.sin(yaw/2),math.cos(yaw/2)]
    return compute_dvl(v_nav,quaternion_xyzw_to_rotation_nav_from_body(q),omega,LEVER,R_DB)


def test_stationary_and_axes():
    assert np.allclose(result([0,0,0]).r3_equivalent_velocity_dvl,0,atol=1e-12)
    assert np.allclose(result([1,0,0]).r3_equivalent_velocity_dvl,R_DB@np.array([1,0,0]))
    assert np.allclose(result([0,1,0]).r3_equivalent_velocity_dvl,R_DB@np.array([0,1,0]))
    assert np.allclose(result([0,0,1]).r3_equivalent_velocity_dvl,R_DB@np.array([0,0,1]))


def test_ninety_degree_yaw_body_forward_invariance():
    zero=result([1,0,0],0).r3_equivalent_velocity_dvl
    ninety=result([0,1,0],math.pi/2).r3_equivalent_velocity_dvl
    assert np.allclose(zero,ninety,atol=1e-12)


def test_lever_arm_is_computed_then_compensated_for_body_origin_model():
    item=result([0.5,0.1,0],0.0,omega=[0,0,0.4])
    expected=np.cross(np.array([0,0,0.4]),LEVER)
    assert np.allclose(item.lever_velocity_body,expected)
    assert np.linalg.norm(expected) > 0.004
    assert np.allclose(item.physical_velocity_dvl-item.r3_equivalent_velocity_dvl,R_DB@expected)
    assert np.allclose(item.r3_equivalent_velocity_dvl,R_DB@np.array([0.5,0.1,0]))


def test_rate_gate_preserves_acquisition_samples_without_stale_republish():
    gate=DeterministicRateGate(5.0)
    accepted=[t for t in np.arange(0,1.001,0.02) if gate.due(float(t))]
    assert np.allclose(accepted,[0,0.2,0.4,0.6,0.8,1.0])


def test_mounting_rotation_is_proper_and_not_applied_twice():
    assert np.allclose(R_DB@R_DB.T,np.eye(3),atol=1e-12)
    assert np.isclose(np.linalg.det(R_DB),1.0)
    once=result([1,0,0]).r3_equivalent_velocity_dvl
    assert not np.allclose(once,R_DB@once)
