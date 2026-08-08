"""Validation tests V1-V7 from OPTICAL_PROPAGATION_SPEC.md section 10.

V4 and V5 are *gating*: if either fails, the affected claim is dropped from the
paper rather than rescued by retuning the physics. They are marked so they can be
selected with ``pytest -m gating``.
"""

import math

import numpy as np
import pytest

from uuv_mode_aware_navigation.optics import (
    ALTITUDE_LOW_M,
    ALTITUDE_NOMINAL_M,
    CAMERA_COAXIAL,
    CAMERA_OFFAXIS,
    LIDAR,
    WATER_LEVELS,
    ChannelConfig,
    WaterState,
    backscatter_integral,
    channel_response,
    near_field_cutoff,
    optical_depth,
)

W0_CLEAR, W1_MODERATE, W2_DEGRADED, W3_TURBID = WATER_LEVELS


def _availability(water_c, altitude, config, trials=400, seed=12345):
    """Empirical availability rate over independent draws."""
    rng = np.random.default_rng(seed)
    water = WaterState(c=water_c)
    hits = sum(
        channel_response(water, altitude, config, rng=rng).available
        for _ in range(trials)
    )
    return hits / trials


# ---------------------------------------------------------------------------
# V1 -- monotonicity
# ---------------------------------------------------------------------------
def test_v1_contrast_decreases_with_attenuation():
    prev = None
    for c in (0.1, 0.3, 0.6, 1.2, 2.0):
        r = channel_response(WaterState(c=c), ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS)
        if prev is not None:
            assert r.contrast < prev, f"contrast rose at c={c}"
        prev = r.contrast


def test_v1_contrast_decreases_with_altitude():
    prev = None
    for h in (0.5, 1.0, 2.0, 3.0, 5.0):
        r = channel_response(WaterState(c=W1_MODERATE), h, CAMERA_OFFAXIS)
        if prev is not None:
            assert r.contrast < prev, f"contrast rose at h={h}"
        prev = r.contrast


def test_v1_availability_non_increasing_in_tau():
    rates = [
        _availability(c, ALTITUDE_NOMINAL_M, LIDAR)
        for c in (0.1, 0.4, 0.8, 1.2, 1.6)
    ]
    for earlier, later in zip(rates, rates[1:]):
        assert later <= earlier + 1e-9, f"availability rose: {rates}"


# ---------------------------------------------------------------------------
# V2 -- limits
# ---------------------------------------------------------------------------
def test_v2_clear_water_is_signal_limited():
    r = channel_response(WaterState(c=1e-6), ALTITUDE_NOMINAL_M, CAMERA_COAXIAL)
    assert r.contrast > 0.9, "near-zero attenuation should leave contrast intact"


def test_v2_extreme_turbidity_kills_every_channel():
    for config in (CAMERA_COAXIAL, CAMERA_OFFAXIS, LIDAR):
        rate = _availability(50.0, ALTITUDE_NOMINAL_M, config)
        assert rate == 0.0, f"{config.name} still available in opaque water"


def test_v2_no_spuriously_confident_fix_in_opaque_water():
    r = channel_response(WaterState(c=50.0), ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS)
    assert r.p_available == 0.0
    assert r.quality == 0.0


# ---------------------------------------------------------------------------
# V3 -- off-axis sign (CORRECTED, see note below)
# ---------------------------------------------------------------------------
def test_v3_separating_the_lamp_raises_contrast():
    water = WaterState(c=W2_DEGRADED)
    prev = None
    for d in (0.0, 0.1, 0.2, 0.35, 0.5):
        cfg = ChannelConfig(name=f"d{d}", tau_max=3.0, baseline_m=d)
        r = channel_response(water, ALTITUDE_LOW_M, cfg)
        if prev is not None:
            assert r.contrast > prev, f"contrast did not rise at d={d}"
        prev = r.contrast


def test_v3_bias_vanishes_at_zero_baseline_and_appears_when_offset():
    """The spec's V3 as originally written claimed bias increases strictly with
    the baseline. That is wrong: a symmetric veil cannot displace a centroid, so
    bias is zero at d=0, and it falls again at large d because the veil itself is
    suppressed. Bias is non-monotone, peaking at intermediate baseline. What must
    hold -- and what makes off-axis lighting a genuine trade rather than a free
    win -- is that leaving d=0 buys contrast and costs a non-zero bias."""
    water = WaterState(c=W2_DEGRADED)

    coaxial = ChannelConfig(name="d0", tau_max=3.0, baseline_m=0.0)
    offset = ChannelConfig(name="d035", tau_max=3.0, baseline_m=0.35)

    r_coax = channel_response(water, ALTITUDE_LOW_M, coaxial)
    r_off = channel_response(water, ALTITUDE_LOW_M, offset)

    assert np.linalg.norm(r_coax.bias_m) == pytest.approx(0.0, abs=1e-12)
    assert np.linalg.norm(r_off.bias_m) > 0.0
    assert r_off.contrast > r_coax.contrast


def test_v3_off_axis_lighting_is_not_a_free_win():
    """Contrast gain must be paid for. If a configuration ever improved contrast
    at zero bias cost, the model would be wrong."""
    water = WaterState(c=W2_DEGRADED)
    base = channel_response(
        water, ALTITUDE_LOW_M, ChannelConfig(name="d0", tau_max=3.0, baseline_m=0.0)
    )
    for d in (0.15, 0.35, 0.6):
        cfg = ChannelConfig(name=f"d{d}", tau_max=3.0, baseline_m=d)
        r = channel_response(water, ALTITUDE_LOW_M, cfg)
        if r.contrast > base.contrast:
            assert np.linalg.norm(r.bias_m) > 0.0, (
                f"baseline {d} improved contrast at zero bias cost"
            )


# ---------------------------------------------------------------------------
# V4 -- envelope non-nesting (GATING)
# ---------------------------------------------------------------------------
@pytest.mark.gating
def test_v4_region_where_lidar_works_and_camera_does_not():
    """Published range limits (1-2 / ~3 / 5-6 attenuation lengths) imply the
    optical channels fail at different water/altitude states. If this region is
    empty the multi-modal claim is unsupported and the paper must narrow."""
    found = []
    for c in (0.6, 1.2, 2.0):
        for h in (ALTITUDE_LOW_M, 2.0, ALTITUDE_NOMINAL_M):
            lidar = _availability(c, h, LIDAR)
            camera = _availability(c, h, CAMERA_OFFAXIS)
            if lidar > 0.8 and camera < 0.2:
                found.append((c, h, lidar, camera))
    assert found, "no (c, h) state where the laser survives and the camera does not"


@pytest.mark.gating
def test_v4_camera_retains_a_regime_of_its_own():
    """The laser must not dominate everywhere, or there is no decision to make.
    In clear water the camera is available AND carries a large rate advantage."""
    camera = _availability(W0_CLEAR, ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS)
    assert camera > 0.9, "camera should be reliable in clear water at survey altitude"
    assert CAMERA_OFFAXIS.rate_hz > LIDAR.rate_hz * 3, "camera rate advantage lost"
    assert CAMERA_OFFAXIS.power_w < LIDAR.power_w, "camera power advantage lost"


# ---------------------------------------------------------------------------
# V5 -- the altitude lever (GATING)
# ---------------------------------------------------------------------------
@pytest.mark.gating
def test_v5_descending_restores_the_camera():
    """At the moderate water level, dropping from survey altitude to low altitude
    must bring the camera back. If it does not, the altitude action carries no
    contribution and the declared levels are wrong."""
    high = _availability(W1_MODERATE, ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS)
    low = _availability(W1_MODERATE, ALTITUDE_LOW_M, CAMERA_OFFAXIS)
    assert high < 0.2, f"camera unexpectedly usable at survey altitude ({high:.2f})"
    assert low > 0.8, f"descending failed to restore the camera ({low:.2f})"


@pytest.mark.gating
def test_v5_altitude_halves_optical_depth():
    """tau = 2*c*h, so halving altitude must halve optical depth exactly."""
    c = W1_MODERATE
    assert optical_depth(c, 2.0) == pytest.approx(optical_depth(c, 4.0) / 2.0)


# ---------------------------------------------------------------------------
# V6 -- determinism
# ---------------------------------------------------------------------------
def test_v6_identical_seed_reproduces_identical_output():
    water = WaterState(c=W1_MODERATE)
    a = [
        channel_response(
            water, ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS, rng=np.random.default_rng(7)
        )
        for _ in range(5)
    ]
    b = [
        channel_response(
            water, ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS, rng=np.random.default_rng(7)
        )
        for _ in range(5)
    ]
    for x, y in zip(a, b):
        assert x.available == y.available
        assert x.tau == y.tau
        assert x.sigma_m == y.sigma_m
        assert np.array_equal(x.bias_m, y.bias_m)


def test_v6_module_never_touches_global_rng():
    np.random.seed(1)
    before = np.random.random()
    np.random.seed(1)
    channel_response(WaterState(c=W1_MODERATE), ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS)
    after = np.random.random()
    assert before == after, "global numpy RNG was disturbed"


# ---------------------------------------------------------------------------
# V7 -- no hidden-state leakage
# ---------------------------------------------------------------------------
def test_v7_navigation_view_excludes_hidden_state():
    r = channel_response(WaterState(c=W2_DEGRADED), ALTITUDE_LOW_M, CAMERA_OFFAXIS)
    view = r.navigation_view()
    for forbidden in ("tau", "contrast", "snr", "sigma_m", "bias_m", "p_available"):
        assert forbidden not in view, f"{forbidden} leaked to the navigation side"
    assert "quality" in view


def test_v7_water_state_is_not_reachable_from_the_navigation_view():
    r = channel_response(WaterState(c=W3_TURBID), ALTITUDE_LOW_M, LIDAR)
    view = r.navigation_view()
    for value in view.values():
        assert not isinstance(value, WaterState)


# ---------------------------------------------------------------------------
# Derived error magnitudes -- plausibility, not just monotonicity
#
# These exist because the first implementation produced a 41 cm position sigma in
# clear water without any test noticing. A model can be perfectly monotone and
# still physically absurd; magnitude has to be asserted explicitly.
# ---------------------------------------------------------------------------
def test_clear_water_fix_is_centimetre_class():
    r = channel_response(WaterState(c=W0_CLEAR), ALTITUDE_NOMINAL_M, CAMERA_OFFAXIS)
    assert 0.005 <= r.sigma_m <= 0.10, f"implausible clear-water sigma: {r.sigma_m:.3f} m"


def test_fix_uncertainty_grows_with_turbidity():
    sigmas = [
        channel_response(WaterState(c=c), ALTITUDE_LOW_M, CAMERA_OFFAXIS).sigma_m
        for c in (0.2, 0.6, 1.2, 2.0)
    ]
    for earlier, later in zip(sigmas, sigmas[1:]):
        assert later > earlier, f"sigma did not grow with turbidity: {sigmas}"


def test_fix_uncertainty_grows_with_altitude():
    near = channel_response(WaterState(c=W1_MODERATE), 1.0, CAMERA_OFFAXIS).sigma_m
    far = channel_response(WaterState(c=W1_MODERATE), 2.5, CAMERA_OFFAXIS).sigma_m
    assert far > near


def test_derived_bias_is_small_and_not_the_headline():
    """The invalidated earlier study used a hand-chosen 0.44 m bias that drove
    almost its entire result. The derived geometric bias here is sub-centimetre,
    which is the honest consequence of the geometry -- Paper 2's contribution
    rests on availability and reconfiguration, not on bias rejection."""
    r = channel_response(WaterState(c=W2_DEGRADED), ALTITUDE_LOW_M, CAMERA_OFFAXIS)
    assert np.linalg.norm(r.bias_m) < 0.05, "derived bias unexpectedly large"


# ---------------------------------------------------------------------------
# Supporting geometry
# ---------------------------------------------------------------------------
def test_near_field_cutoff_matches_the_documented_geometry():
    # d / (tan(theta_cam) + tan(theta_light)); 0.35 / (2*tan30) ~ 0.30 m
    assert near_field_cutoff(0.35) == pytest.approx(0.35 / (2 * math.tan(math.radians(30))))
    assert near_field_cutoff(0.0) == 0.0


def test_backscatter_integral_falls_with_attenuation_and_cutoff():
    assert backscatter_integral(0.5, 0.1, 5.0) > backscatter_integral(2.0, 0.1, 5.0)
    assert backscatter_integral(0.5, 0.5, 5.0) < backscatter_integral(0.5, 0.1, 5.0)
    assert backscatter_integral(0.5, 5.0, 5.0) == 0.0


def test_water_state_interpolates_declared_levels():
    assert WaterState.from_turbidity(0.0).c == pytest.approx(W0_CLEAR)
    assert WaterState.from_turbidity(1.0).c == pytest.approx(W3_TURBID)
    mid = WaterState.from_turbidity(0.5).c
    assert W1_MODERATE < mid < W2_DEGRADED
