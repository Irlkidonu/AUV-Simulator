#!/usr/bin/env python3
"""Fit the availability and optical-feedback models and write them to disk.

The demonstrator loads these rather than fitting at start-up, so the ROS nodes
and the headless campaign provably use the same coefficients. Both are fitted on
development data only.

Usage::

    PYTHONPATH=. python3 scripts/fit_models.py --outdir models/
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uuv_mode_aware_navigation.availability import (  # noqa: E402
    AvailabilityModel,
    AvailabilitySample,
)
from uuv_mode_aware_navigation.campaign import DEVELOPMENT_SEED_ROOT  # noqa: E402
from uuv_mode_aware_navigation.imaging import (  # noqa: E402
    OpticalFeedback,
    analyse_image,
    render_patch,
    seabed_texture,
)
from uuv_mode_aware_navigation.optics import (  # noqa: E402
    CAMERA_OFFAXIS,
    CONFIGURATIONS,
    WaterState,
    channel_response,
)

ALTITUDES = (1.0, 1.5, 2.0, 2.5, 3.0, 4.0)
TURBIDITIES = (0.15, 0.35, 0.6, 0.9, 1.2, 1.6, 2.0)


def fit_availability(seed: int) -> AvailabilityModel:
    """Delegate to the campaign's own calibration.

    This function used to re-implement the fit, and the two copies drifted: the
    campaign's version was extended with trend-bearing samples and future-horizon
    labels while this one kept training on the instantaneous case. The exported
    model would then have been fitted differently from the model the campaign
    validated, and the ROS demonstrator would have made different decisions from
    the results in the paper -- silently, because both fits succeed.

    There is now one definition. If the campaign's calibration changes, the
    exported artefact changes with it.
    """
    from run_campaign import calibrate

    return calibrate(seed)


def fit_optical_feedback(seed: int) -> tuple[OpticalFeedback, dict]:
    rng = np.random.default_rng(seed)
    textures = [seabed_texture(seed=seed + 101 + i) for i in range(4)]
    features, quality = [], []
    for c in np.linspace(0.15, 2.2, 15):
        for altitude in ALTITUDES:
            for config in CONFIGURATIONS:
                for texture in textures:
                    water = WaterState(c=float(c))
                    features.append(
                        analyse_image(render_patch(water, altitude, config, texture, rng))
                    )
                    quality.append(channel_response(water, altitude, config).quality)
    model = OpticalFeedback().fit(features, quality)

    # Agreement on conditions used for neither fitting nor feature selection.
    check_rng = np.random.default_rng(seed + 900_000)
    check_textures = [seabed_texture(seed=seed + 700_000 + i) for i in range(3)]
    check_f, check_q = [], []
    for c in np.linspace(0.25, 2.15, 13):
        for altitude in (1.1, 1.8, 2.7, 3.6):
            for config in CONFIGURATIONS:
                for texture in check_textures:
                    water = WaterState(c=float(c))
                    check_f.append(analyse_image(
                        render_patch(water, altitude, config, texture, check_rng)
                    ))
                    check_q.append(channel_response(water, altitude, config).quality)
    return model, model.agreement(check_f, check_q)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", default="models")
    parser.add_argument("--root", type=int, default=DEVELOPMENT_SEED_ROOT)
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    availability = fit_availability(args.root + 1)
    (outdir / "availability.json").write_text(
        json.dumps(dict(availability.to_dict()), indent=2)
    )
    print(f"availability model -> {outdir / 'availability.json'}")

    feedback, agreement = fit_optical_feedback(args.root + 2)
    (outdir / "optical_feedback.json").write_text(
        json.dumps(feedback.to_dict(), indent=2)
    )
    print(f"optical feedback   -> {outdir / 'optical_feedback.json'}")
    print("  agreement with the analytic index on withheld conditions:")
    for key, value in agreement.items():
        print(f"    {key:<12}{value}")
    (outdir / "optical_feedback_agreement.json").write_text(
        json.dumps(agreement, indent=2)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
