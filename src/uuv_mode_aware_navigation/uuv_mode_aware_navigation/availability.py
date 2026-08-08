"""Calibrated predictive model of optical-channel availability.

Reference implementation of ``OPTICAL_PROPAGATION_SPEC.md`` section 8.2 and
``MODE_MANAGER_SPEC.md`` section 2 step 2.

This is the formal content of "optical feedback": before committing to a sensing
configuration, the vehicle predicts what it *would* be able to see under each
configuration it could adopt, and acts on that prediction.

The counterfactual the manager actually faces
---------------------------------------------
The question is **not** "how available is configuration k at altitude h", which
is unanswerable from an observation alone. It is::

    Given quality q observed through the active configuration at altitude
    h_observed, would candidate configuration k produce a fix at altitude
    h_candidate?

Both altitudes are required. Quality and altitude are *jointly* informative
because optical depth is their product: ``tau = 2*c*h``. Observing q at
h_observed identifies the water state; the candidate altitude then rescales it by
``h_candidate / h_observed``. A model given only one altitude cannot separate
"the water got worse" from "I flew higher", and will happily learn that climbing
improves visibility.

That failure is not hypothetical -- it is what the first version of this module
did, and the altitude-lever test caught it.

Honest-degradation properties
-----------------------------
* Trained on the **observable** ``q``, never on beam attenuation or optical
  depth, so predictions degrade honestly when the water state is ambiguous.
* Scored for **calibration**, not just accuracy. A confident-but-wrong
  availability model would make the manager act decisively on bad predictions,
  which is worse than an uncertain one.
* Fitted on development seeds only, frozen before held-out execution, never
  refitted after inspecting held-out outcomes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

import numpy as np

__all__ = [
    "AvailabilitySample",
    "AvailabilityModel",
    "ReliabilityBin",
    "brier_score",
]

#: Ridge strength. Prevents divergence under perfectly separable development
#: data, which does occur: in clear water at low altitude every configuration
#: succeeds, and a plain logistic fit would send weights to infinity.
DEFAULT_RIDGE = 1e-3
DEFAULT_ITERATIONS = 60
_CONVERGENCE_TOL = 1e-9


@dataclass(frozen=True)
class AvailabilitySample:
    """One development observation.

    Attributes
    ----------
    quality:
        Image-derived quality observed through the *active* configuration.
    observed_altitude_m:
        Altitude at which ``quality`` was observed.
    candidate_altitude_m:
        Altitude being considered for the candidate configuration.
    configuration:
        Name of the *candidate* configuration whose availability was observed.
    quality_trend:
        Change in observed quality over the identification window. Zero when
        unknown, which reproduces the memoryless model exactly.
    available:
        Whether the candidate configuration produced a fix.
    """

    quality: float
    observed_altitude_m: float
    candidate_altitude_m: float
    configuration: str
    available: bool
    quality_trend: float = 0.0


@dataclass(frozen=True)
class ReliabilityBin:
    """One row of a reliability diagram."""

    lower: float
    upper: float
    count: int
    mean_predicted: float
    observed_rate: float

    @property
    def gap(self) -> float:
        """Calibration gap. Large values mean the model's confidence is a lie."""
        return abs(self.mean_predicted - self.observed_rate)


def _features(
    quality: np.ndarray,
    observed_altitude: np.ndarray,
    candidate_altitude: np.ndarray,
    trend: np.ndarray | None = None,
) -> np.ndarray:
    """Design matrix ``[1, q, r, q*r, log r, d, d*r]`` with ``r = h_cand / h_obs``.

    ``d`` is the recent *trend* in observed quality: the change over the
    identification window, positive when conditions are improving. It is added
    because the model was otherwise memoryless, and a memoryless predictor is
    exactly wrong in the situation that matters most. Turbidity ramps: a quality
    of 0.4 on the way down means a candidate will shortly be unavailable, while
    the same 0.4 on the way up means it will shortly be fine. Measured against a
    clairvoyant that knows the answer over the decision horizon, the manager
    captured well under half the achievable score, and every input it had
    described the present instant only.

    The ``d*r`` interaction is included for the same reason as ``q*r``: a trend
    observed at one altitude implies a different trend at another, because the
    optical depth being trended through scales with the path length.

    The altitude *ratio* is the physically meaningful variable: optical depth
    scales linearly with path length, so moving from ``h_obs`` to ``h_cand``
    multiplies tau by exactly ``r``, whatever the water. Using the ratio rather
    than the raw candidate altitude is what lets a model fitted at one set of
    altitudes generalise to another.

    The ``log r`` term is included because viability is exponential in tau, so
    the decision boundary is closer to linear in log-space.
    """
    quality = np.asarray(quality, dtype=float)
    observed_altitude = np.asarray(observed_altitude, dtype=float)
    candidate_altitude = np.asarray(candidate_altitude, dtype=float)
    trend = (
        np.zeros_like(quality) if trend is None
        else np.asarray(trend, dtype=float)
    )
    ratio = candidate_altitude / np.clip(observed_altitude, 1e-6, None)
    return np.column_stack(
        [
            np.ones_like(quality),
            quality,
            ratio,
            quality * ratio,
            np.log(np.clip(ratio, 1e-6, None)),
            trend,
            trend * ratio,
        ]
    )


def _sigmoid(z: np.ndarray) -> np.ndarray:
    """Overflow-safe logistic applied elementwise."""
    out = np.empty_like(z, dtype=float)
    positive = z >= 0
    out[positive] = 1.0 / (1.0 + np.exp(-z[positive]))
    exp_z = np.exp(z[~positive])
    out[~positive] = exp_z / (1.0 + exp_z)
    return out


def brier_score(predicted: Sequence[float], observed: Sequence[bool]) -> float:
    """Mean squared error of probabilistic predictions. Lower is better.

    Preferred over accuracy because it penalises confident mistakes, which are
    the ones that make a manager act badly.
    """
    p = np.asarray(predicted, dtype=float)
    y = np.asarray(observed, dtype=float)
    if p.size == 0:
        raise ValueError("no predictions to score")
    return float(np.mean((p - y) ** 2))


class AvailabilityModel:
    """Per-configuration ridge-regularised logistic availability model."""

    #: Must match the columns ``_features`` builds, in order. This tuple is the
    #: single declaration of the design: it is written into every exported model
    #: and checked on load, so a model fitted against a different design cannot
    #: be restored silently.
    FEATURE_NAMES = (
        "1",
        "quality",
        "altitude_ratio",
        "quality*ratio",
        "log_ratio",
        "quality_trend",
        "quality_trend*ratio",
    )

    def __init__(self, ridge: float = DEFAULT_RIDGE) -> None:
        self.ridge = float(ridge)
        self._weights: dict[str, np.ndarray] = {}
        self._base_rate: dict[str, float] = {}
        self._fitted = False

    # -- fitting ----------------------------------------------------------
    def fit(
        self,
        samples: Iterable[AvailabilitySample],
        iterations: int = DEFAULT_ITERATIONS,
    ) -> "AvailabilityModel":
        """Fit one logistic model per candidate configuration.

        Uses Newton-Raphson (IRLS) with a ridge penalty. Deterministic: no
        random initialisation, fixed iteration budget, no global RNG.
        """
        grouped: dict[str, list[AvailabilitySample]] = {}
        for sample in samples:
            grouped.setdefault(sample.configuration, []).append(sample)
        if not grouped:
            raise ValueError("no samples supplied")

        for name, group in grouped.items():
            x = _features(
                np.array([s.quality for s in group]),
                np.array([s.observed_altitude_m for s in group]),
                np.array([s.candidate_altitude_m for s in group]),
                np.array([s.quality_trend for s in group]),
            )
            y = np.array([1.0 if s.available else 0.0 for s in group])
            self._base_rate[name] = float(y.mean())

            w = np.zeros(x.shape[1])
            ridge_eye = self.ridge * np.eye(x.shape[1])
            for _ in range(iterations):
                p = _sigmoid(x @ w)
                gradient = x.T @ (y - p) - self.ridge * w
                s = np.clip(p * (1.0 - p), 1e-9, None)
                hessian = x.T @ (x * s[:, None]) + ridge_eye
                try:
                    step = np.linalg.solve(hessian, gradient)
                except np.linalg.LinAlgError:  # pragma: no cover - guarded by ridge
                    break
                w = w + step
                if float(np.max(np.abs(step))) < _CONVERGENCE_TOL:
                    break
            self._weights[name] = w

        self._fitted = True
        return self

    # -- prediction -------------------------------------------------------
    def predict(
        self,
        quality: float,
        observed_altitude_m: float,
        candidate_altitude_m: float,
        configuration: str,
        quality_trend: float = 0.0,
    ) -> float:
        """Probability that ``configuration`` yields a fix at the candidate altitude.

        An unseen configuration falls back to its base rate if known, and to a
        maximally uncertain 0.5 otherwise. It never returns a confident answer
        about something it was not trained on.
        """
        if not self._fitted:
            raise RuntimeError("model must be fitted before prediction")
        w = self._weights.get(configuration)
        if w is None:
            return float(self._base_rate.get(configuration, 0.5))
        x = _features(
            np.array([quality]),
            np.array([observed_altitude_m]),
            np.array([candidate_altitude_m]),
            np.array([quality_trend]),
        )
        return float(_sigmoid(x @ w)[0])

    def predict_many(self, samples: Sequence[AvailabilitySample]) -> list[float]:
        return [
            self.predict(
                s.quality,
                s.observed_altitude_m,
                s.candidate_altitude_m,
                s.configuration,
                s.quality_trend,
            )
            for s in samples
        ]

    # -- calibration assessment -------------------------------------------
    def brier(self, samples: Sequence[AvailabilitySample]) -> float:
        """Brier score on held-back development data."""
        return brier_score(self.predict_many(samples), [s.available for s in samples])

    def reliability(
        self, samples: Sequence[AvailabilitySample], bins: int = 10
    ) -> list[ReliabilityBin]:
        """Reliability diagram rows: does 70% confidence mean 70% success?"""
        if bins < 1:
            raise ValueError("bins must be positive")
        predicted = np.asarray(self.predict_many(samples), dtype=float)
        observed = np.array([1.0 if s.available else 0.0 for s in samples])
        edges = np.linspace(0.0, 1.0, bins + 1)
        rows: list[ReliabilityBin] = []
        for lower, upper in zip(edges[:-1], edges[1:]):
            if upper >= 1.0:
                mask = (predicted >= lower) & (predicted <= upper)
            else:
                mask = (predicted >= lower) & (predicted < upper)
            count = int(mask.sum())
            rows.append(
                ReliabilityBin(
                    lower=float(lower),
                    upper=float(upper),
                    count=count,
                    mean_predicted=float(predicted[mask].mean()) if count else 0.0,
                    observed_rate=float(observed[mask].mean()) if count else 0.0,
                )
            )
        return rows

    def max_calibration_gap(
        self,
        samples: Sequence[AvailabilitySample],
        bins: int = 10,
        minimum_count: int = 5,
    ) -> float:
        """Worst calibration gap over adequately populated bins."""
        rows = [r for r in self.reliability(samples, bins) if r.count >= minimum_count]
        return max((r.gap for r in rows), default=0.0)

    # -- serialisation for the freeze record -------------------------------
    def to_dict(self) -> Mapping[str, object]:
        """Exportable parameters. Written into the freeze record verbatim."""
        return {
            "ridge": self.ridge,
            "features": list(self.FEATURE_NAMES),
            "weights": {k: v.tolist() for k, v in self._weights.items()},
            "base_rate": dict(self._base_rate),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "AvailabilityModel":
        """Restore a fitted model, refusing a stale one.

        The feature list is checked against the current design rather than
        ignored. A stored model fitted before the trend terms were added declares
        five features while the code builds seven, and loading it would either
        raise deep inside a matrix product or -- worse -- succeed against a
        silently mismatched design and give the ROS demonstrator different
        predictions from the campaign that validated them. Failing loudly here is
        the only way the two stay the same system.
        """
        stored = payload.get("features")
        if stored is not None and list(stored) != list(cls.FEATURE_NAMES):
            raise ValueError(
                "availability model is stale: it was fitted for features "
                f"{list(stored)} but the current design is "
                f"{list(cls.FEATURE_NAMES)}. Re-run scripts/fit_models.py."
            )
        model = cls(ridge=float(payload.get("ridge", DEFAULT_RIDGE)))
        model._weights = {
            k: np.asarray(v, dtype=float)
            for k, v in dict(payload.get("weights", {})).items()
        }
        for name, w in model._weights.items():
            if w.shape[0] != len(cls.FEATURE_NAMES):
                raise ValueError(
                    f"availability model for {name!r} has {w.shape[0]} weights "
                    f"but the design has {len(cls.FEATURE_NAMES)} features"
                )
        model._base_rate = {
            k: float(v) for k, v in dict(payload.get("base_rate", {})).items()
        }
        model._fitted = True
        return model
