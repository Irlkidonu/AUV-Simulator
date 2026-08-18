"""The docking collar must actually be enterable.

v2.0.0 shipped a collar whose collision boxes had their radial and tangential
extents transposed, so the emitted throat had an inner radius of 0.1587 m rather
than the documented 0.240 m. The vehicle's hull half-diagonal is 0.1749 m, so a
perfectly centred vehicle was physically blocked and docking capture was
impossible. Correction C2 claimed the 0.240 m aperture; nothing asserted it.

The existing P14 suite did not catch this because it tested that contact was
*well behaved* -- no tunnelling, bounded forces, finite states -- all of which
remain true when the vehicle simply stops at a wall. No test asserted the
functional property the correction exists to provide: that a correctly aligned
vehicle can pass through.

These tests measure the geometry actually emitted into the SDF, not the constants
the source intends, because the defect was precisely a disagreement between the
two.
"""

from __future__ import annotations

import re

import numpy as np

from uuv_sim_physics import world_builder

#: Hull collision box from config/vehicle_bluerov2_phys.yaml (x, y, z).
HULL_SIZE_M = (0.46, 0.30, 0.18)
HULL_HALF_DIAGONAL_M = float(np.hypot(HULL_SIZE_M[1] / 2, HULL_SIZE_M[2] / 2))

#: C2's documented capture clearance, quoted from config/corrections.yaml.
DOCUMENTED_CLEARANCE_M = 0.065


def _collar_segments(sdf: str):
    """Yield (centre_yz, radial_unit, tangential_unit, half_y, half_z) per box."""
    start = sdf.index('<link name="collar_collision">')
    block = sdf[start:sdf.index("</link>", start)]
    found = re.findall(r"<pose>([^<]*)</pose>.*?<size>([^<]*)</size>", block, re.S)
    assert found, "no collar collision boxes emitted"
    for pose_text, size_text in found:
        _, py, pz, roll, _, _ = (float(v) for v in pose_text.split())
        _, sy, sz = (float(v) for v in size_text.split())
        cos_a, sin_a = np.cos(roll), np.sin(roll)
        # roll about X maps local +y to radial and local +z to tangential
        yield (np.array([py, pz]), np.array([cos_a, sin_a]),
               np.array([-sin_a, cos_a]), sy / 2.0, sz / 2.0)


def _emitted_inner_radius(sdf: str) -> float:
    radii = []
    for centre, local_y, local_z, half_y, half_z in _collar_segments(sdf):
        radius = float(np.linalg.norm(centre))
        radial = centre / radius
        half_extent = abs(local_y @ radial) * half_y + abs(local_z @ radial) * half_z
        radii.append(radius - half_extent)
    return float(min(radii))


def _octagon_normals(sdf: str):
    return [centre / np.linalg.norm(centre)
            for centre, *_ in _collar_segments(sdf)]


def _hull_fits(sdf: str, offset_yz) -> bool:
    """Is the hull cross-section inside the collar aperture at this offset?

    The aperture is convex, so containment of the four rectangle corners is
    necessary and sufficient.
    """
    apothem = _emitted_inner_radius(sdf)
    normals = _octagon_normals(sdf)
    half_y, half_z = HULL_SIZE_M[1] / 2, HULL_SIZE_M[2] / 2
    for sign_y in (-1.0, 1.0):
        for sign_z in (-1.0, 1.0):
            corner = np.asarray(offset_yz) + np.array([sign_y * half_y,
                                                       sign_z * half_z])
            if max(float(n @ corner) for n in normals) > apothem + 1e-12:
                return False
    return True


def _max_offset(sdf: str, direction) -> float:
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    low, high = 0.0, 0.5
    for _ in range(60):
        mid = 0.5 * (low + high)
        if _hull_fits(sdf, mid * direction):
            low = mid
        else:
            high = mid
    return low


def _validated_sdf() -> str:
    return world_builder.build_world_sdf(validated=True)


# --- P14a: a centred vehicle must be able to enter the throat ---------------

def test_p14a_centred_vehicle_fits_through_the_throat() -> None:
    sdf = _validated_sdf()
    assert _hull_fits(sdf, (0.0, 0.0)), (
        "a perfectly centred vehicle cannot pass the collar. Emitted inner "
        f"radius {_emitted_inner_radius(sdf):.4f} m is below the hull "
        f"half-diagonal {HULL_HALF_DIAGONAL_M:.4f} m. This is the v2.0.0 defect.")


def test_p14a_emitted_inner_radius_matches_the_documented_value() -> None:
    emitted = _emitted_inner_radius(_validated_sdf())
    assert abs(emitted - world_builder.COLLAR_INNER_R) < 1e-6, (
        f"emitted collar inner radius {emitted:.4f} m disagrees with "
        f"COLLAR_INNER_R = {world_builder.COLLAR_INNER_R} m")


def test_p14a_throat_admits_the_hull_with_margin() -> None:
    emitted = _emitted_inner_radius(_validated_sdf())
    assert emitted > HULL_HALF_DIAGONAL_M, (
        f"throat radius {emitted:.4f} m <= hull half-diagonal "
        f"{HULL_HALF_DIAGONAL_M:.4f} m: capture is geometrically impossible")


# --- P14b: an offset inside the documented clearance must still enter -------

def test_p14b_permitted_lateral_offset_still_enters() -> None:
    sdf = _validated_sdf()
    inside = DOCUMENTED_CLEARANCE_M * 0.9
    for direction in ((1, 0), (0, 1), (1, 1), (1, -1)):
        assert _hull_fits(sdf, inside * np.array(direction, float)
                          / np.linalg.norm(direction)), (
            f"offset {inside:.4f} m along {direction} is inside the documented "
            f"{DOCUMENTED_CLEARANCE_M} m clearance but does not fit")


# --- P14c: an offset well beyond the clearance must be rejected -------------

def test_p14c_excessive_offset_is_blocked() -> None:
    sdf = _validated_sdf()
    for direction in ((1, 0), (0, 1), (1, 1)):
        assert not _hull_fits(sdf, 0.20 * np.array(direction, float)
                              / np.linalg.norm(direction)), (
            f"a 0.20 m offset along {direction} should contact the collar")


def test_p14c_capture_clearance_is_near_the_documented_value() -> None:
    """The worst-direction clearance should reproduce C2's ~0.065 m.

    The 8-segment discretisation makes the aperture an octagon rather than a
    circle, so the achievable offset is slightly larger than the circular
    idealisation (0.240 - 0.1749 = 0.0651 m). A generous band is used because
    the point is to pin the order of magnitude, not the discretisation.
    """
    sdf = _validated_sdf()
    worst = min(_max_offset(sdf, (np.cos(a), np.sin(a)))
                for a in np.linspace(0, 2 * np.pi, 181))
    assert 0.05 <= worst <= 0.11, (
        f"worst-direction capture clearance {worst:.4f} m is far from the "
        f"documented ~{DOCUMENTED_CLEARANCE_M} m")


# --- P14d: the seating region must be geometrically unobstructed ------------

def test_p14d_seating_region_is_clear_of_collar_and_funnel() -> None:
    """Nothing may obstruct the axis between the funnel mouth and the collar.

    The vehicle seats with its centre near the collar plane, so the aperture
    must admit the hull along the whole terminal run, not only at one plane.
    """
    sdf = _validated_sdf()
    assert _hull_fits(sdf, (0.0, 0.0)), "collar blocks the seating region"

    config = world_builder.load_config(validated=True)
    funnel = config["dock_station"]["links"]["funnel"]["visuals"]
    half_y, half_z = HULL_SIZE_M[1] / 2, HULL_SIZE_M[2] / 2
    for plate in funnel:
        _, py, pz = plate["pose"][:3]
        pitch, yaw = plate["pose"][4], plate["pose"][5]
        length = plate["size"][0]
        # The plate is a thin box tilted toward the axis. Its nearest approach
        # is the offset, less the in-plane reach of the half-length, less the
        # half-thickness projected onto the same axis. The thin dimension is
        # size[2] for the top/bottom plates and size[1] for the left/right ones.
        if abs(pitch) > 1e-9:            # top / bottom plate, offset in z
            half_thick = plate["size"][2] / 2
            reach = (abs(pz) - abs(np.sin(pitch)) * length / 2
                     - half_thick * abs(np.cos(pitch)))
            assert reach > half_z, (
                f"funnel plate {plate['name']} reaches z={reach:.3f} m, inside "
                f"the hull half-height {half_z} m")
        elif abs(yaw) > 1e-9:            # left / right plate, offset in y
            half_thick = plate["size"][1] / 2
            reach = (abs(py) - abs(np.sin(yaw)) * length / 2
                     - half_thick * abs(np.cos(yaw)))
            assert reach > half_y, (
                f"funnel plate {plate['name']} reaches y={reach:.3f} m, inside "
                f"the hull half-width {half_y} m")


# --- the regression guard proper -------------------------------------------

def test_v200_defect_signature_is_absent() -> None:
    """Pin the specific defect: local-y must carry the radial thickness.

    If the size vector is ever transposed back, the emitted inner radius drops
    to mid - width/2 = 0.1587 m and this fails immediately.
    """
    sdf = _validated_sdf()
    emitted = _emitted_inner_radius(sdf)
    assert emitted > 0.20, (
        f"collar inner radius {emitted:.4f} m looks like the transposed-extent "
        "defect from v2.0.0 (0.1587 m)")
