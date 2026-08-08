"""Navigation filter shared by every method under comparison.

A twelve-state extended Kalman filter over position, velocity, accelerometer
bias, and water current, fused from inertial prediction plus DVL bottom-track
velocity, DVL water-track velocity, pressure depth, optical position, and
acoustic range.

Why the current is a filter state
---------------------------------
A survey vehicle flying a commanded speed along a commanded heading does not
travel along that heading: it travels along the vector sum of its own motion
through the water and the motion of the water itself. Guidance that ignores the
difference produces a steady cross-track offset that no amount of waypoint
re-pointing removes, because the vehicle is corrected toward the waypoint at
exactly the rate it is being carried away from the line.

Compensating requires knowing the flow, and the flow is not directly measurable.
It is *inferable*: bottom track measures velocity over ground, water track
measures velocity through the water, and the difference between them is the
current. Estimating it inside the shared filter rather than in a side
calculation means the estimate carries a covariance, so the guidance law knows
not merely what the current is but how well it is known -- which is the quantity
that matters once an outage makes the estimate stale.

Fairness by construction
------------------------
This filter is **identical** for the proposed manager, every comparator, and
every ablation. Fairness rule R1 requires it: in June 2026 a 98.6% headline in
this workspace was traced to a baseline that omitted gravity compensation while
the proposed method had it, so the figure measured physics correctness rather
than method quality. There is exactly one filter here, and methods differ only
in *which measurements they choose to obtain and admit* -- never in the
estimator's mathematics.

Declared simplification
-----------------------
The state carries no attitude. Body-frame DVL velocity is treated as world-frame,
which is exact for the level, heading-aligned survey flight used in this study
and inexact under roll and pitch. Attitude estimation, gyro bias, and DVL
misalignment belong to the localization scope of the companion paper; Paper 2's
contribution is the manager, and the filter is deliberately a shared, simple,
auditable component rather than a second contribution. This is stated as a
limitation in the manuscript rather than left implicit.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

import numpy as np

__all__ = ["FilterConfig", "NavigationFilter", "UpdateOutcome", "FusionMode"]

GRAVITY = np.array([0.0, 0.0, -9.81])


@dataclass(frozen=True)
class FilterConfig:
    """Process and measurement noise. Identical across all methods."""

    accel_process_mps2: float = 0.021
    bias_random_walk: float = 1e-4
    velocity_random_walk: float = 0.015
    dvl_sigma_mps: float = 0.0025
    #: Water-track velocity noise. Mirrors ``SensorNoise.dvl_water_track_mps``.
    dvl_water_sigma_mps: float = 0.025
    depth_sigma_m: float = 0.01
    acoustic_sigma_m: float = 0.35
    #: Current random walk (m/s per sqrt(s)).
    #:
    #: This is the only place the filter encodes how fast the flow may change,
    #: and it sets how quickly the estimate goes stale once both DVL modes are
    #: lost. The value corresponds to about 0.03 m/s of drift over a 100 s
    #: outage, which is the order of the tidal-band variation the scenarios
    #: impose. Setting it far lower would make the filter certain of a current it
    #: can no longer see, which is the failure the covariance exists to prevent.
    current_random_walk: float = 3e-3
    #: Normalised innovation squared above which a measurement is rejected.
    nis_gate: float = 9.0
    #: After an aiding outage, the first measurements are admitted with inflated
    #: covariance rather than hard-rejected. Hard rejection after an outage
    #: causes a reject-drift-reject collapse: the estimate drifts while gated,
    #: which makes the next innovation larger, which gates it again.
    reacquisition_inflation: float = 25.0
    reacquisition_confirmations: int = 2
    initial_position_var_m2: float = 1.0
    initial_velocity_var_m2s2: float = 0.25
    initial_bias_var: float = 1e-3
    #: Initial current variance. The filter starts believing the current is zero
    #: and knowing almost nothing about it: 0.25 m^2/s^2 is one standard
    #: deviation of 0.5 m/s, which spans the range the scenarios impose. The
    #: vehicle is not told the flow it is launched into.
    initial_current_var_m2s2: float = 0.25


class FusionMode(Enum):
    """How a suspicious measurement is admitted.

    The two strategies fail in opposite ways, which is what makes choosing
    between them a decision rather than a preference.

    ``GATE`` rejects any measurement whose normalised innovation exceeds the
    threshold. That protects the estimate from a corrupted fix, but it also
    discards good-but-surprising ones -- and once the estimate has drifted, a
    *correct* fix looks surprising, so the gate can lock itself out. The
    companion paper's version eight failed exactly that way, rejecting all 73
    available fixes in one run and diverging by 1,123 m.

    ``WEIGHT`` always admits the measurement but scales its covariance by the
    normalised innovation, so a suspicious fix moves the estimate a little rather
    than not at all. It cannot lock out, but it also cannot fully reject: a
    contaminated fix always drags the estimate somewhat.

    Which is right depends on what the errors look like. Under a systematic
    velocity drift with clean absolute fixes, weighting wins, because the drift
    must be corrected and a gate refuses the corrections. Under multipath, where
    a reflected arrival reads 15--20 m long and one-sided, gating wins, because
    no amount of down-weighting makes a 20 m error harmless.
    """

    GATE = "gate"
    WEIGHT = "weight"


@dataclass(frozen=True)
class UpdateOutcome:
    """Result of attempting one measurement update."""

    accepted: bool
    nis: float
    inflated: bool = False


#: State vector layout. Named so the update Jacobians below read as physics
#: rather than as index arithmetic.
#: How many recent gate decisions the innovation-exceedance rate is taken over.
#: Long enough that the rate is not dominated by a single measurement, short
#: enough that it reflects present conditions rather than the whole mission.
EXCEEDANCE_WINDOW = 20

N_STATES = 12
POS = slice(0, 3)
VEL = slice(3, 6)
BIAS = slice(6, 9)
CUR = slice(9, 12)


class NavigationFilter:
    """Twelve-state EKF: [position(3), velocity(3), accel bias(3), current(3)].

    Velocity is over ground. Current is the world-frame water velocity. The
    vehicle's velocity through the water is their difference, which is never
    stored because it is never needed as a state -- only as a measurement
    prediction.
    """

    def __init__(
        self,
        config: FilterConfig = FilterConfig(),
        initial_position: Sequence[float] = (0.0, 0.0, -17.0),
    ) -> None:
        self.config = config
        self.x = np.zeros(N_STATES)
        self.x[POS] = np.asarray(initial_position, dtype=float)
        self.P = np.diag(
            [config.initial_position_var_m2] * 3
            + [config.initial_velocity_var_m2s2] * 3
            + [config.initial_bias_var] * 3
            + [config.initial_current_var_m2s2] * 3
        )
        self._pending_reacquisition = 0
        self._outage = False
        self.rejected = 0
        self.accepted = 0
        #: Recent gate outcomes, newest last, as booleans meaning "this
        #: measurement's NIS exceeded the gate". The manager needs to know not
        #: merely whether a fix *arrives* but whether arriving fixes are
        #: consistent with the filter, because a fix the gate throws away
        #: carries no information. Bounded so the rate tracks current
        #: conditions rather than the whole mission.
        self._gate_history: deque[bool] = deque(maxlen=EXCEEDANCE_WINDOW)
        #: Default admission strategy. Set per configuration by the manager, and
        #: identical across methods unless a method declares otherwise, so the
        #: filter's mathematics stay shared (fairness rule R1).
        self.fusion = FusionMode.GATE

    # -- accessors ---------------------------------------------------------
    @property
    def position(self) -> np.ndarray:
        return self.x[POS].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[VEL].copy()

    @property
    def current(self) -> np.ndarray:
        """Estimated world-frame water velocity (m/s)."""
        return self.x[CUR].copy()

    @property
    def position_covariance_trace(self) -> float:
        return float(np.trace(self.P[POS, POS]))

    @property
    def innovation_exceedance_rate(self) -> float:
        """Fraction of recent measurements whose NIS exceeded the gate.

        This is the observable that distinguishes *a fix arrived* from *a fix
        helped*. A configuration can offer measurements at a high rate and have
        every one of them rejected as inconsistent, in which case the aiding it
        appears to provide is worth nothing. Reporting the rate lets the manager
        price that difference instead of assuming every arriving measurement
        collapses uncertainty to the aided floor.

        Returns 0.0 before any measurement has been offered, which is the
        correct prior: nothing has been found inconsistent yet.
        """
        if not self._gate_history:
            return 0.0
        return sum(self._gate_history) / len(self._gate_history)

    @property
    def current_covariance_trace(self) -> float:
        """How well the flow is known (m^2/s^2).

        Grows without bound while both DVL modes are unavailable, which is the
        signal that current compensation is running on a stale estimate.
        """
        return float(np.trace(self.P[CUR, CUR]))

    # -- predict -----------------------------------------------------------
    def predict(self, accel_measured: np.ndarray, dt: float) -> None:
        """Inertial propagation with gravity compensation and bias removal."""
        if dt <= 0.0:
            return
        cfg = self.config
        # a = f + g - bias.  (f = a - g, so recovering a ADDS gravity back.)
        accel = np.asarray(accel_measured, dtype=float) - self.x[BIAS] + GRAVITY

        self.x[POS] = self.x[POS] + self.x[VEL] * dt + 0.5 * accel * dt * dt
        self.x[VEL] = self.x[VEL] + accel * dt
        # The current is not propagated. It is modelled as a random walk, so its
        # mean is unchanged by prediction and only its covariance grows. The
        # accelerometer already senses whatever the flow does to the vehicle, so
        # the current must not also be added into the kinematics -- doing so
        # would count the same physical effect twice.

        F = np.eye(N_STATES)
        F[POS, VEL] = np.eye(3) * dt
        F[POS, BIAS] = -0.5 * np.eye(3) * dt * dt
        F[VEL, BIAS] = -np.eye(3) * dt

        Q = np.zeros((N_STATES, N_STATES))
        accel_var = cfg.accel_process_mps2**2
        Q[POS, POS] = np.eye(3) * (0.25 * accel_var * dt**4)
        Q[VEL, VEL] = np.eye(3) * (
            accel_var * dt * dt + cfg.velocity_random_walk**2 * dt
        )
        Q[BIAS, BIAS] = np.eye(3) * (cfg.bias_random_walk**2 * dt)
        Q[CUR, CUR] = np.eye(3) * (cfg.current_random_walk**2 * dt)

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    # -- generic update ----------------------------------------------------
    def _update(
        self,
        H: np.ndarray,
        innovation: np.ndarray,
        R: np.ndarray,
        gate: bool,
        fusion: "FusionMode | None" = None,
    ) -> UpdateOutcome:
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:  # pragma: no cover - guarded numerically
            return UpdateOutcome(accepted=False, nis=float("inf"))

        nis = float(innovation.T @ S_inv @ innovation)
        inflated = False

        # Record the gate decision before any covariance inflation rewrites the
        # NIS. What the manager needs to know is whether the measurement *as
        # offered* was consistent with the filter, which is the question the
        # gate asks; inflating R to admit a surprising measurement changes how
        # it is used, not whether it was surprising.
        self._gate_history.append(bool(gate and nis > self.config.nis_gate))

        mode = fusion or self.fusion
        if gate and nis > self.config.nis_gate and mode is FusionMode.WEIGHT:
            # Covariance weighting: admit, but scale the measurement covariance
            # by how surprising the measurement was. The scaling is the ratio of
            # the observed normalised innovation to the gate, so a measurement
            # exactly at the threshold is unchanged and one at ten times the
            # threshold contributes a tenth as much.
            inflated = True
            R = R * (nis / self.config.nis_gate)
            S = H @ self.P @ H.T + R
            S_inv = np.linalg.inv(S)
            nis = float(innovation.T @ S_inv @ innovation)
        elif gate and nis > self.config.nis_gate:
            if self._outage and self._pending_reacquisition > 0:
                # Bounded recovery: admit with inflated covariance instead of
                # rejecting, so the filter can re-latch after an outage.
                inflated = True
                R = R * self.config.reacquisition_inflation
                S = H @ self.P @ H.T + R
                S_inv = np.linalg.inv(S)
                self._pending_reacquisition -= 1
                if self._pending_reacquisition == 0:
                    self._outage = False
            else:
                self.rejected += 1
                return UpdateOutcome(accepted=False, nis=nis)

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        I_KH = np.eye(N_STATES) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.accepted += 1
        return UpdateOutcome(accepted=True, nis=nis, inflated=inflated)

    # -- measurement-specific updates --------------------------------------
    def update_velocity(self, velocity: np.ndarray) -> UpdateOutcome:
        """DVL bottom track: velocity over ground."""
        H = np.zeros((3, N_STATES))
        H[:, VEL] = np.eye(3)
        R = np.eye(3) * self.config.dvl_sigma_mps**2
        innovation = np.asarray(velocity, dtype=float) - self.x[VEL]
        return self._update(H, innovation, R, gate=True)

    def update_water_velocity(self, water_velocity: np.ndarray) -> UpdateOutcome:
        """DVL water track: velocity through the water, which is ``v - current``.

        This single sign is what makes the current observable. Paired with a
        bottom-track update in the same tick the two measurements differ only by
        the current, and the filter attributes the difference to it. With
        bottom track absent, this measurement still constrains the *combination*
        ``v - c``; the split between them then depends on position aiding, and if
        that is absent too the current uncertainty simply grows -- correctly,
        because nothing is observing it.
        """
        H = np.zeros((3, N_STATES))
        H[:, VEL] = np.eye(3)
        H[:, CUR] = -np.eye(3)
        R = np.eye(3) * self.config.dvl_water_sigma_mps**2
        predicted = self.x[VEL] - self.x[CUR]
        innovation = np.asarray(water_velocity, dtype=float) - predicted
        return self._update(H, innovation, R, gate=True)

    def update_depth(self, depth_m: float) -> UpdateOutcome:
        H = np.zeros((1, N_STATES))
        H[0, 2] = 1.0
        R = np.array([[self.config.depth_sigma_m**2]])
        innovation = np.array([depth_m - self.x[2]])
        return self._update(H, innovation, R, gate=True)

    def update_position(
        self, position: np.ndarray, sigma_m: float
    ) -> UpdateOutcome:
        H = np.zeros((3, N_STATES))
        H[:, POS] = np.eye(3)
        R = np.eye(3) * max(sigma_m, 1e-4) ** 2
        innovation = np.asarray(position, dtype=float) - self.x[POS]
        return self._update(H, innovation, R, gate=True)

    def update_acoustic_range(
        self, measured_range_m: float, beacon: Sequence[float]
    ) -> UpdateOutcome:
        """Range-only update. Observability depends on beacon geometry."""
        delta = self.x[POS] - np.asarray(beacon, dtype=float)
        predicted = float(np.linalg.norm(delta))
        if predicted < 1e-6:
            return UpdateOutcome(accepted=False, nis=float("inf"))
        H = np.zeros((1, N_STATES))
        H[0, POS] = delta / predicted
        R = np.array([[self.config.acoustic_sigma_m**2]])
        innovation = np.array([measured_range_m - predicted])
        return self._update(H, innovation, R, gate=True)

    # -- outage handling ---------------------------------------------------
    def note_aiding_outage(self) -> None:
        """Record that aiding was lost, arming bounded recovery on its return."""
        if not self._outage:
            self._outage = True
            self._pending_reacquisition = self.config.reacquisition_confirmations
