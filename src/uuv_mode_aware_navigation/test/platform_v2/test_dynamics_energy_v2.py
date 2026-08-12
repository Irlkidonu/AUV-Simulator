import math
import numpy as np

from uuv_mode_aware_navigation.dynamics import FirstOrderDynamics, SixDofDynamics, SixDofState, VehicleCommand
from uuv_mode_aware_navigation.energy import LegacyOpticalEnergyModel, FullPlatformEnergyModel
from uuv_mode_aware_navigation.sensor_models.coupling import dvl_bottom_lock_probability, image_motion_blur_m


def test_first_order_wrapper_matches_legacy_equation() -> None:
    model=FirstOrderDynamics(np.zeros(3),np.array([.1,0,0]))
    model.step(VehicleCommand(np.array([.5,0,0])),.1)
    assert np.array_equal(model.velocity_mps,np.array([.18,0,0]))


def test_six_dof_respects_force_and_remains_finite() -> None:
    model=SixDofDynamics(SixDofState(np.zeros(3)))
    for _ in range(200): model.step(VehicleCommand(np.array([.7,0,0]),.1),np.zeros(3),.05)
    assert np.all(np.isfinite(model.state.position_m))
    assert 0<model.state.velocity_body_mps[0]<1.0


def test_legacy_energy_reproduces_reported_payload_quantity() -> None:
    model=LegacyOpticalEnergyModel(); model.step(45.0,379.46666666666664)
    assert math.isclose(model.energy_j,17076.0,abs_tol=1e-9)


def test_full_energy_keeps_sensing_and_total_distinct() -> None:
    model=FullPlatformEnergyModel(); result=model.step(.5,12,4,2,100)
    assert result.sensing_j==1600
    assert result.total_j>result.sensing_j
    assert 0<model.state_of_charge<1


def test_vehicle_state_changes_sensor_capability() -> None:
    assert dvl_bottom_lock_probability(5,0)>dvl_bottom_lock_probability(29,math.radians(20))
    assert image_motion_blur_m(.75,.05,.2,3)>image_motion_blur_m(.25,.01,0,3)
