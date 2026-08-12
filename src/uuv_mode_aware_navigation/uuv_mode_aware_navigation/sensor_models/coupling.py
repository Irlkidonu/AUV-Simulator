"""Physical sensing consequences of platform-v2 vehicle state."""

import math


def dvl_bottom_lock_probability(altitude_m: float, tilt_rad: float,
                                maximum_range_m: float=30.0) -> float:
    slant=altitude_m/max(math.cos(abs(tilt_rad)),1e-3)
    range_term=max(0.0,min(1.0,(maximum_range_m-slant)/(0.25*maximum_range_m)))
    tilt_term=math.exp(-(abs(tilt_rad)/math.radians(25.0))**2)
    return range_term*tilt_term


def image_motion_blur_m(speed_mps: float, exposure_s: float, angular_rate_rps: float,
                        altitude_m: float) -> float:
    return abs(speed_mps)*exposure_s+abs(angular_rate_rps)*altitude_m*exposure_s


def camera_footprint_width_m(altitude_m: float, horizontal_fov_rad: float,
                             pitch_rad: float=0.0) -> float:
    return 2.0*altitude_m*math.tan(horizontal_fov_rad/2.0)/max(math.cos(pitch_rad),1e-3)
