import numpy as np

from uuv_mode_aware_navigation.acoustics import LBL, USBL, SINGLE_BEACON, NoiseState
from uuv_mode_aware_navigation.acoustics_v2 import (
    AcousticPacketModel, AcousticWorldGeometry, geometry_aware_fix,
)


GEOMETRY=AcousticWorldGeometry(
    ((-25,-25,-20),(25,-25,-20),(25,25,-20),(-25,25,-20)),
    (0,0,-20),(0,0,0),vessel_velocity_mps=(0.2,0,0),
)


def test_lbl_covariance_comes_from_actual_geometry() -> None:
    centre=geometry_aware_fix(LBL,(0,0,-17),GEOMETRY,NoiseState())
    edge=geometry_aware_fix(LBL,(45,0,-17),GEOMETRY,NoiseState())
    assert centre.available
    assert np.all(np.linalg.eigvalsh(centre.covariance_m2)>0)
    assert (not edge.available) or np.trace(edge.covariance_m2)>np.trace(centre.covariance_m2)


def test_moving_vessel_changes_usbl_geometry() -> None:
    early=geometry_aware_fix(USBL,(20,0,-17),GEOMETRY,NoiseState(),0)
    late=geometry_aware_fix(USBL,(20,0,-17),GEOMETRY,NoiseState(),50)
    assert early.slant_range_m!=late.slant_range_m
    assert not np.allclose(early.covariance_m2,late.covariance_m2)


def test_packet_arrival_is_later_than_validity_and_loss_is_seeded() -> None:
    model=AcousticPacketModel(packet_loss_probability=.3)
    rng=np.random.default_rng(22_230_001)
    packets=[model.generate(i,30,LBL,rng) for i in range(20)]
    assert [p.sequence for p in packets]==list(range(1,21))
    assert all(p.arrival_time_s>p.validity_time_s for p in packets)
    assert 0<sum(p.dropped for p in packets)<20


def test_single_beacon_does_not_pretend_to_be_full_position_fix() -> None:
    fix=geometry_aware_fix(SINGLE_BEACON,(10,0,-17),GEOMETRY,NoiseState())
    assert fix.available
    assert np.max(np.linalg.eigvalsh(fix.covariance_m2))>1e5
