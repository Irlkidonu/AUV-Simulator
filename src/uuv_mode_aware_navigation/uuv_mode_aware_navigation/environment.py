"""Environment identification from observables alone.

The manager infers *capability* modes -- what the vehicle can still do. This
module infers *conditions* -- what the vehicle is in. They are deliberately
separate. Labelling capability modes ``clear``/``medium``/``turbid``, as the
invalidated earlier system did, makes the mode a restatement of a commanded
environment variable rather than an inference, which is why modes here are named
for the decision they force. Conditions are a different question with a different
answer, and keeping them apart is what lets each be checked on its own.

Why this exists
---------------
The manager's configuration choice is driven by predicted availability: will this
sensor produce a fix if I switch to it? Measured against a clairvoyant that knows
the answer, the manager captures well under half the achievable score, and the
entire gap sits in that prediction. Availability depends on conditions, so a
better statement of the conditions is a better predictor -- which makes
environment identification both the contribution and the repair.

Three axes are classified, because three different things degrade three different
sensing modalities:

* **turbidity** -- what the optical channels care about, read from image content;
* **acoustic noise** -- what the ranging techniques care about, read from the
  behaviour of the ranges themselves;
* **current** -- what guidance and station-keeping care about, read from the
  filter's own current state.

Each is a discrete label rather than a continuous index, because the actions the
vehicle takes are discrete. Nothing changes at a turbidity of 0.83 that does not
also change at 0.85; what changes is whether the camera can still work.

Rule N2 applies throughout: every input is something a deployed vehicle can
measure. There is no path from this module to the true water state, the commanded
noise level, the true current, or the fault schedule. That is enforced by
:class:`EnvironmentFeatures` carrying only observable quantities, and checked by
test.

Model class
-----------
A multinomial logistic regression over a short time window, fitted with ridge
regularisation. Deliberately small: the features are a handful of scalars per
tick, and a larger model would be fitted on far less data than it needs and would
be impossible to defend. A window rather than a single sample because conditions
evolve and a sequence carries information a snapshot does not -- a frame that is
dark because the water clouded looks the same as one that is dark because the
vehicle passed over a shadow, until you see what came before it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

__all__ = [
    "TURBIDITY_CLASSES",
    "NOISE_CLASSES",
    "CURRENT_CLASSES",
    "EnvironmentFeatures",
    "EnvironmentEstimate",
    "EnvironmentClassifier",
    "turbidity_label",
    "noise_label",
    "current_label",
]

#: Class labels, ordered from benign to severe.
TURBIDITY_CLASSES: tuple[str, ...] = ("clear", "medium", "turbid")
NOISE_CLASSES: tuple[str, ...] = ("quiet", "noisy")
CURRENT_CLASSES: tuple[str, ...] = ("weak", "moderate", "strong")

#: Class boundaries, in the units of the underlying truth. These define what the
#: labels *mean*; they are not thresholds the classifier applies, since it never
#: sees these quantities. They are stated here so the ground truth used for
#: training and scoring is explicit and reproducible.
#:
#: Turbidity boundaries are placed where the optical channels change behaviour
#: rather than at round numbers: below 0.5 the nominal camera works at survey
#: altitude, above 1.2 it does not work at any altitude in the action space.
TURBIDITY_BOUNDARIES: tuple[float, float] = (0.5, 1.2)
#: Noise boundary at the midpoint of the modelled ambient range, which is also
#: where the multipath outlier rate passes roughly one in ten interrogations.
NOISE_BOUNDARY_DB: float = 55.0
#: Current boundaries as fractions of the slowest configuration speed (0.25 m/s):
#: below 0.05 m/s the flow is negligible against any configuration, above
#: 0.15 m/s it is a majority of the slow configuration's speed.
CURRENT_BOUNDARIES: tuple[float, float] = (0.05, 0.15)


def turbidity_label(beam_attenuation: float) -> str:
    low, high = TURBIDITY_BOUNDARIES
    if beam_attenuation < low:
        return "clear"
    return "medium" if beam_attenuation < high else "turbid"


def noise_label(spectral_level_db: float) -> str:
    return "quiet" if spectral_level_db < NOISE_BOUNDARY_DB else "noisy"


def current_label(speed_mps: float) -> str:
    low, high = CURRENT_BOUNDARIES
    if speed_mps < low:
        return "weak"
    return "moderate" if speed_mps < high else "strong"


@dataclass(frozen=True)
class EnvironmentFeatures:
    """Observable evidence about conditions, aggregated over a window.

    Every field is something a deployed vehicle measures. Nothing here is a
    commanded value or an evaluator-side truth.
    """

    #: Image-derived optical quality, mean and spread over the window. The mean
    #: says how good the imagery is; the spread separates steadily poor water
    #: from imagery that is fluctuating.
    optical_quality_mean: float
    optical_quality_std: float
    #: Fraction of the window in which the active optical channel produced a fix.
    optical_fix_rate: float
    #: Fraction of offered acoustic measurements the filter's gate rejected.
    #: Multipath outliers are large and one-sided, so a rising rejection rate is
    #: the signature of a noisy, reverberant environment -- observable without
    #: any acoustic instrumentation beyond the ranges themselves.
    acoustic_reject_rate: float
    #: Mean normalised innovation squared of accepted acoustic measurements.
    acoustic_nis_mean: float
    #: Estimated current speed and how well it is known, from the filter.
    current_speed_mps: float
    current_sigma_mps: float

    def vector(self) -> np.ndarray:
        return np.array(
            [
                self.optical_quality_mean,
                self.optical_quality_std,
                self.optical_fix_rate,
                self.acoustic_reject_rate,
                self.acoustic_nis_mean,
                self.current_speed_mps,
                self.current_sigma_mps,
            ],
            dtype=float,
        )


@dataclass(frozen=True)
class EnvironmentEstimate:
    """A classified environment, with the confidence of each label."""

    turbidity: str
    noise: str
    current: str
    turbidity_confidence: float
    noise_confidence: float
    current_confidence: float

    @property
    def minimum_confidence(self) -> float:
        """Weakest of the three. Below this the manager should not act on it."""
        return min(
            self.turbidity_confidence,
            self.noise_confidence,
            self.current_confidence,
        )


def _design(features: Sequence[EnvironmentFeatures]) -> np.ndarray:
    """Feature matrix with an intercept column."""
    rows = [f.vector() for f in features]
    x = np.asarray(rows, dtype=float)
    return np.hstack([np.ones((x.shape[0], 1)), x])


class _MultinomialLogistic:
    """Ridge-regularised multinomial logistic regression.

    Fitted by gradient descent on the regularised cross-entropy. Chosen over a
    closed-form or heavier learner because the model has to be defensible: a few
    dozen coefficients over seven observable scalars, all inspectable, with a
    single regularisation constant that is stated rather than searched.
    """

    def __init__(self, classes: Sequence[str], ridge: float = 1e-2) -> None:
        self.classes = tuple(classes)
        self.ridge = float(ridge)
        self.weights: Optional[np.ndarray] = None
        self._mean: Optional[np.ndarray] = None
        self._scale: Optional[np.ndarray] = None

    def _standardise(self, design: np.ndarray, fit: bool = False) -> np.ndarray:
        if fit:
            self._mean = design.mean(axis=0)
            self._scale = design.std(axis=0)
            # The intercept column has zero spread; leave it alone.
            self._scale = np.where(self._scale < 1e-9, 1.0, self._scale)
            self._mean[0] = 0.0
            self._scale[0] = 1.0
        return (design - self._mean) / self._scale

    def fit(
        self,
        features: Sequence[EnvironmentFeatures],
        labels: Sequence[str],
        iterations: int = 4000,
        learning_rate: float = 0.25,
    ) -> "_MultinomialLogistic":
        design = self._standardise(_design(features), fit=True)
        index = {c: i for i, c in enumerate(self.classes)}
        target = np.zeros((len(labels), len(self.classes)))
        for row, label in enumerate(labels):
            target[row, index[label]] = 1.0

        # Class-balanced weighting. Without it the rare classes are simply not
        # predicted: fitted on real runs, the three-class turbidity model placed
        # 5 of 9,994 held-out samples in ``medium`` and none of the 514 genuinely
        # medium ones, scoring 0.907 purely on the two common extremes. A
        # classifier that never emits a class is not identifying three
        # conditions, and an accuracy that hides it is worse than no accuracy.
        #
        # Medium turbidity is rare in the data because ramps pass through it, and
        # it is also where the decision matters most -- some configurations still
        # work there and others do not -- so it is the last class that should be
        # allowed to vanish.
        counts = target.sum(axis=0)
        # An axis fitted on a constant label has empty classes, so guard the
        # division rather than relying on np.where, which evaluates both
        # branches and warns.
        balance = np.zeros_like(counts)
        present = counts > 0
        balance[present] = target.shape[0] / (int(present.sum()) * counts[present])
        sample_weight = (target * balance).sum(axis=1, keepdims=True)

        weights = np.zeros((design.shape[1], len(self.classes)))
        n = max(float(sample_weight.sum()), 1e-9)
        for _ in range(iterations):
            scores = design @ weights
            scores -= scores.max(axis=1, keepdims=True)
            exp = np.exp(scores)
            probabilities = exp / exp.sum(axis=1, keepdims=True)
            gradient = design.T @ (sample_weight * (probabilities - target)) / n
            gradient += self.ridge * weights
            weights -= learning_rate * gradient
        self.weights = weights
        return self

    def probabilities(self, features: Sequence[EnvironmentFeatures]) -> np.ndarray:
        if self.weights is None:
            raise RuntimeError("classifier used before fitting")
        design = self._standardise(_design(features))
        scores = design @ self.weights
        scores -= scores.max(axis=1, keepdims=True)
        exp = np.exp(scores)
        return exp / exp.sum(axis=1, keepdims=True)

    def predict(self, features: Sequence[EnvironmentFeatures]) -> list[str]:
        p = self.probabilities(features)
        return [self.classes[i] for i in p.argmax(axis=1)]


class EnvironmentClassifier:
    """Classifies turbidity, acoustic noise, and current from observables."""

    def __init__(self, ridge: float = 1e-2) -> None:
        self._turbidity = _MultinomialLogistic(TURBIDITY_CLASSES, ridge)
        self._noise = _MultinomialLogistic(NOISE_CLASSES, ridge)
        self._current = _MultinomialLogistic(CURRENT_CLASSES, ridge)
        self.fitted = False

    def fit(
        self,
        features: Sequence[EnvironmentFeatures],
        turbidity: Sequence[str],
        noise: Sequence[str],
        current: Sequence[str],
    ) -> "EnvironmentClassifier":
        self._turbidity.fit(features, turbidity)
        self._noise.fit(features, noise)
        self._current.fit(features, current)
        self.fitted = True
        return self

    def classify(self, features: EnvironmentFeatures) -> EnvironmentEstimate:
        batch = [features]
        t = self._turbidity.probabilities(batch)[0]
        n = self._noise.probabilities(batch)[0]
        c = self._current.probabilities(batch)[0]
        return EnvironmentEstimate(
            turbidity=TURBIDITY_CLASSES[int(t.argmax())],
            noise=NOISE_CLASSES[int(n.argmax())],
            current=CURRENT_CLASSES[int(c.argmax())],
            turbidity_confidence=float(t.max()),
            noise_confidence=float(n.max()),
            current_confidence=float(c.max()),
        )

    # -- validation ---------------------------------------------------------
    def confusion(
        self,
        features: Sequence[EnvironmentFeatures],
        truth: Sequence[str],
        axis: str = "turbidity",
    ) -> tuple[np.ndarray, tuple[str, ...]]:
        """Confusion matrix ``[true, predicted]`` and its class order.

        Reported rather than a bare accuracy because the errors are not
        interchangeable: calling turbid water clear invites the vehicle to keep
        using a camera that cannot see, while calling clear water turbid only
        costs survey rate. A single accuracy figure hides which of those is
        happening.
        """
        model = {
            "turbidity": self._turbidity,
            "noise": self._noise,
            "current": self._current,
        }[axis]
        predicted = model.predict(features)
        classes = model.classes
        index = {c: i for i, c in enumerate(classes)}
        matrix = np.zeros((len(classes), len(classes)), dtype=int)
        for actual, guess in zip(truth, predicted):
            matrix[index[actual], index[guess]] += 1
        return matrix, classes

    def accuracy(
        self,
        features: Sequence[EnvironmentFeatures],
        truth: Sequence[str],
        axis: str = "turbidity",
    ) -> float:
        matrix, _ = self.confusion(features, truth, axis)
        total = int(matrix.sum())
        return float(np.trace(matrix)) / total if total else 0.0
