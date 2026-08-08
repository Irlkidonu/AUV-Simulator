"""Seabed relief for the demonstrator: one definition, used twice.

The world file needs a heightmap image; the vehicle node needs to know how far
it is above the bottom. If those two came from different definitions the
demonstrator would render one seabed and fly over another, and the altimeter --
which drives optical availability, terrain matching and every mode decision that
depends on them -- would be reading a surface that is not on screen.

So the relief is defined here as a function, ``height_at(x, y)``. The heightmap
is rendered from it by ``scripts/make_seabed.py`` and the vehicle node samples
the same function directly. The two cannot disagree.

**This is demonstrator-only.** The campaign does not import this module and does
not load a world. Its terrain enters as ``TerrainProfile``, a scalar gradient
magnitude along the survey, which is the quantity the terrain-matching accuracy
model actually needs; a spatial map would add detail the study does not use. No
reported number depends on anything here.
"""

from __future__ import annotations

import numpy as np

#: Mean seabed depth, metres, negative down. Matches the flat value the rest of
#: the package uses so that mean altitude is unchanged by adding relief.
MEAN_DEPTH_M: float = -20.0

#: Half-width of the modelled patch, metres. The survey box sits well inside it.
EXTENT_M: float = 60.0

#: Relief inside the survey box, metres peak to trough.
#:
#: Deliberately gentle. The survey line runs at -17 m over a mean bottom of
#: -20 m, so the vehicle has 3 m of nominal altitude; ground that rose more than
#: about a metre would start eating the clearance the guidance assumes and could
#: put the vehicle underground on a low-altitude leg. Terrain matching needs
#: gradient, not amplitude, so this is enough for the technique to work on.
RELIEF_M: float = 1.2

#: Relief outside it. Unconstrained, because the vehicle never goes there.
#:
#: Without this the seabed reads as a plane: 1.2 m spread over 120 m is a one
#: per cent slope, which the eye correctly sees as flat. Real bathymetry has
#: structure at every scale, and what makes a seabed look like a seabed is
#: relief in the distance, not underfoot.
FAR_RELIEF_M: float = 11.0

#: Radii, metres, over which the amplitude ramps from the near value to the far
#: one. The inner radius comfortably contains the survey box.
NEAR_R: float = 16.0
FAR_R: float = 46.0


def _amplitude(x, y):
    """Relief amplitude as a function of distance from the survey area.

    A smooth ramp, not a step: a discontinuity in amplitude would show as a
    visible ring on the seabed.
    """
    r = np.hypot(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
    s = np.clip((r - NEAR_R) / (FAR_R - NEAR_R), 0.0, 1.0)
    s = s * s * (3.0 - 2.0 * s)                       # smoothstep
    return (RELIEF_M + (FAR_RELIEF_M - RELIEF_M) * s) / 2.0


def _octave(x, y, freq, phase, rot):
    c, s = np.cos(rot), np.sin(rot)
    u = (x * c - y * s) * freq
    v = (x * s + y * c) * freq
    return np.sin(u + phase[0]) * np.cos(v + phase[1])


def height_at(x, y):
    """Seabed height above the mean depth, in metres. Accepts scalars or arrays.

    Layered sinusoids at rotated, incommensurate frequencies: a broad slope, a
    ridge set, cross-cutting structure, and fine sand-wave roughness. Not a
    physical sedimentation model -- it is a surface with relief at several
    scales, which is what an altimeter and a terrain matcher need to have
    something to work with.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    h = np.zeros(np.broadcast(x, y).shape, dtype=float)
    # (weight, frequency, phase, rotation)
    layers = (
        (1.00, 0.045, (0.3, 1.1), 0.00),   # broad basin and rise
        (0.55, 0.115, (2.1, 0.4), 0.90),   # ridge set
        (0.30, 0.240, (1.4, 2.7), 2.10),   # cross structure
        (0.14, 0.520, (0.8, 1.9), 0.45),   # sand waves
        (0.07, 1.150, (2.6, 0.2), 1.70),   # fine roughness
    )
    total = sum(w for w, _, _, _ in layers)
    for weight, freq, phase, rot in layers:
        h += weight * _octave(x, y, freq, phase, rot)
    h /= total

    # Two long-wavelength features that only exist out where the amplitude is
    # free: a ridge and the valley beside it. They give the distance something
    # to be, which fine roughness alone never does.
    far = np.hypot(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
    ridge = np.exp(-((far - 34.0) / 12.0) ** 2) * np.sin(np.arctan2(y, x) * 1.6)
    h = h + 0.55 * ridge

    return h * _amplitude(x, y)


def depth_at(x, y):
    """Absolute seabed depth (negative down) at a horizontal position."""
    return MEAN_DEPTH_M + height_at(x, y)


def gradient_at(x, y, step: float = 0.5):
    """Magnitude of the seabed slope, metres per metre.

    Central differences on :func:`height_at`. This is the quantity terrain
    matching accuracy depends on -- see the sigma = sigma_depth / |grad h|
    relation -- so a demonstrator flying over the rendered relief can show the
    technique working where there is structure and failing where there is not.
    """
    hx = (height_at(x + step, y) - height_at(x - step, y)) / (2.0 * step)
    hy = (height_at(x, y + step) - height_at(x, y - step)) / (2.0 * step)
    return float(np.hypot(hx, hy))
