#!/usr/bin/env python3
"""Fit and validate the environment classifier on features from real runs.

The classifier's first validation used features generated directly from the
propagation and multipath models. That shows the model class can separate the
classes given clean evidence; it does *not* show that the statistics a vehicle
actually accumulates during a survey carry the same information. Window
statistics from a live filter are noisier, correlated in time, and contaminated
by whatever the vehicle happened to be doing.

This script closes that gap. It flies real scenarios, records the exact
:class:`EnvironmentFeatures` the manager would have been handed at each decision
tick, pairs them with the evaluator-side truth for that instant, fits on
development seeds, and scores on held-out seeds that were used for neither
fitting nor any modelling choice.

Only the numbers this script produces may be reported.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uuv_mode_aware_navigation.campaign import (  # noqa: E402
    DEVELOPMENT_SEED_ROOT,
    NoiseProfile,
    Scenario,
    run_scenario,
)
from uuv_mode_aware_navigation.environment import (  # noqa: E402
    EnvironmentClassifier,
    EnvironmentEstimate,
    current_label,
    noise_label,
    turbidity_label,
)
from uuv_mode_aware_navigation.manager import VehicleConfiguration  # noqa: E402
from uuv_mode_aware_navigation.comparators import FixedPolicy  # noqa: E402
from uuv_mode_aware_navigation.optics import CAMERA_OFFAXIS  # noqa: E402

from run_campaign import (  # noqa: E402
    BASELINE_CURRENT,
    calibrate_optical_feedback,
    scenario_family,
)

#: Set once scenarios carry an ambient-noise profile and the label actually
#: varies. Until then the axis is fitted against a constant and its accuracy is
#: 1.000 by construction -- a perfect score measuring nothing, which is why this
#: guard exists rather than a comment.
NOISE_AXIS_READY = True


class _Recorder:
    """Stands in for the classifier during collection.

    Receives exactly the features the real classifier would receive, records
    them, and returns a neutral estimate so the run proceeds unchanged.
    """

    def __init__(self) -> None:
        self.features: list = []
        self.ticks: list[int] = []
        self._tick = 0

    def classify(self, features) -> EnvironmentEstimate:
        self.features.append(features)
        self.ticks.append(self._tick)
        self._tick += 1
        return EnvironmentEstimate("unknown", "unknown", "unknown", 0.0, 0.0, 0.0)


def collect(seed_root: int, seeds: int, feedback):
    """Fly every scenario family and return (features, turbidity, noise, current)."""
    features, turbidity, noise, current = [], [], [], []
    config = VehicleConfiguration(CAMERA_OFFAXIS, 3.0, 0.5)
    for entry in scenario_family():
        for k in range(seeds):
            scenario = Scenario(
                name=entry[0],
                seed=seed_root + 1000 + k,
                water=entry[1],
                schedule=entry[2],
                current=entry[3] if len(entry) > 3 else BASELINE_CURRENT,
                noise=entry[4] if len(entry) > 4 else NoiseProfile.constant(40.0),
            )
            recorder = _Recorder()
            run_scenario(
                scenario,
                FixedPolicy(config),
                optical_feedback=feedback,
                classifier=recorder,
            )
            for tick, feature in zip(recorder.ticks, recorder.features):
                # The decision period is what advances the tick counter, so the
                # instant a feature describes is recoverable exactly.
                t = tick * scenario.decision_period_s
                features.append(feature)
                turbidity.append(turbidity_label(scenario.water.at(t).c))
                noise.append(
                    noise_label(scenario.noise.at(t).spectral_level_db)
                )
                current.append(
                    current_label(float(np.linalg.norm(scenario.current.at(t))))
                )
    return features, turbidity, noise, current


def _report(name: str, classifier, features, truth, axis: str) -> dict:
    """Print the confusion matrix, per-class recall, and balanced accuracy.

    Overall accuracy alone is not reportable for these axes. Fitted without
    class balancing, the turbidity model scored 0.907 while predicting the
    ``medium`` class 5 times in 9,994 and recovering none of the 514 genuinely
    medium samples -- the score came entirely from the two common extremes.
    Per-class recall makes that visible immediately, and balanced accuracy (the
    mean of the recalls) is the figure that cannot be inflated by imbalance.
    """
    matrix, classes = classifier.confusion(features, truth, axis)
    accuracy = classifier.accuracy(features, truth, axis)
    support = matrix.sum(axis=1)
    recall = np.divide(
        np.diag(matrix), support,
        out=np.zeros(len(classes)), where=support > 0,
    )
    balanced = float(recall[support > 0].mean()) if (support > 0).any() else 0.0

    print(f"\n{name} -- {axis}: accuracy = {accuracy:.3f}  "
          f"balanced = {balanced:.3f}  (n = {len(truth)})")
    print("   true\\pred  " + " ".join(f"{c:>9}" for c in classes)
          + "     recall  support")
    for i, c in enumerate(classes):
        print(f"   {c:<10} " + " ".join(f"{v:>9d}" for v in matrix[i])
              + f"     {recall[i]:6.3f}  {support[i]:7d}")
    never = [c for i, c in enumerate(classes) if matrix[:, i].sum() == 0]
    if never:
        print(f"   WARNING: never predicted: {', '.join(never)}")
    return {
        "axis": axis,
        "accuracy": accuracy,
        "balanced_accuracy": balanced,
        "per_class_recall": {c: float(recall[i]) for i, c in enumerate(classes)},
        "support": {c: int(support[i]) for i, c in enumerate(classes)},
        "never_predicted": never,
        "classes": list(classes),
        "confusion": matrix.tolist(),
        "n": len(truth),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=2)
    parser.add_argument("--root", type=int, default=DEVELOPMENT_SEED_ROOT)
    parser.add_argument(
        "--heldout-root", type=int, default=DEVELOPMENT_SEED_ROOT + 200_000,
        help="disjoint from both the development root and the reserved "
             "held-out root at 20,400,000, which stays untouched until freeze",
    )
    parser.add_argument("--out", type=str, default="models/environment.json")
    args = parser.parse_args()

    feedback = calibrate_optical_feedback(args.root + 2)

    print("collecting development features from real runs ...")
    f_train, t_train, n_train, c_train = collect(args.root, args.seeds, feedback)
    print(f"  {len(f_train)} decision ticks")

    print("collecting held-out features from disjoint seeds ...")
    f_test, t_test, n_test, c_test = collect(args.heldout_root, args.seeds, feedback)
    print(f"  {len(f_test)} decision ticks")

    classifier = EnvironmentClassifier().fit(f_train, t_train, n_train, c_train)

    results = [
        _report("held-out", classifier, f_test, t_test, "turbidity"),
        _report("held-out", classifier, f_test, c_test, "current"),
    ]
    if NOISE_AXIS_READY and len(set(n_train)) > 1:
        results.append(_report("held-out", classifier, f_test, n_test, "noise"))
    else:
        print("\nnoise: NOT REPORTED -- the label does not vary across the "
              "collected runs, so any score would measure nothing.")

    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "development_ticks": len(f_train),
        "heldout_ticks": len(f_test),
        "noise_axis_reported": NOISE_AXIS_READY,
        "results": results,
    }, indent=1))
    print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
