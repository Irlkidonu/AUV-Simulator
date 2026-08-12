"""Declarative platform-v2 action space and axis-level reachability model."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class SelectionConditions:
    optical_attenuation_m_inv: float = 0.2
    exposure_s: float = 0.01
    texture_scale_m: float = 0.10
    time_pressure: float = 0.0
    collision_risk: float = 0.0
    infrastructure: frozenset[str] = frozenset({"beacon", "lbl", "surface"})
    total_blackout_s: float = 0.0
    fix_expected_s: float = 0.0
    estimator_drift_m: float = 0.0


@dataclass(frozen=True)
class SelectionScore:
    action: str
    score: float
    physical_benefit: float
    operational_cost: float


@dataclass(frozen=True)
class ActionSpaceV2:
    declaration: dict

    @classmethod
    def load(cls, path: str | Path) -> "ActionSpaceV2":
        data = json.loads(Path(path).read_text())
        required = {"identifier", "optical_channels", "altitudes_m", "speeds_mps",
                    "acoustic_techniques", "fusion_modes", "mission_actions", "costs"}
        if set(data) != required:
            raise ValueError("action-space declaration has missing or unknown keys")
        return cls(data)

    @classmethod
    def default(cls) -> "ActionSpaceV2":
        return cls({
            "identifier":"platform_v2_action_space_v1",
            "optical_channels":["camera_coaxial","camera_offaxis","lidar"],
            "altitudes_m":[1.0,2.0,3.0],"speeds_mps":[.25,.5,.75],
            "acoustic_techniques":["single_beacon","lbl","usbl"],
            "fusion_modes":["gate","weight"],
            "mission_actions":["continue","hold_for_fix","return_to_last_good_fix","abort_leg","surface_for_gps"],
            "costs":{"time_weight":.6,"power_weight":.4,"risk_weight":1.2,
                     "blur_variance_weight_m2":1.0,"nominal_speed_mps":.5,
                     "maximum_speed_mps":.75},
        })

    @property
    def speeds_mps(self):
        return tuple(float(v) for v in self.declaration["speeds_mps"])

    def rank_speeds(self, conditions: SelectionConditions) -> list[SelectionScore]:
        costs = self.declaration["costs"]
        nominal = float(costs["nominal_speed_mps"])
        maximum = float(costs["maximum_speed_mps"])
        out = []
        for speed in self.speeds_mps:
            # Image displacement during exposure relative to resolvable texture.
            blur_ratio = speed * conditions.exposure_s / max(conditions.texture_scale_m, 1e-9)
            blur_variance = float(costs["blur_variance_weight_m2"]) * blur_ratio**2
            slow_time = max(0.0, nominal-speed)/nominal
            fast_benefit = conditions.time_pressure * max(0.0, speed-nominal) / max(maximum-nominal, 1e-9)
            operational = float(costs["time_weight"]) * slow_time + blur_variance
            out.append(SelectionScore(f"speed:{speed:.2f}", operational-fast_benefit,
                                      fast_benefit-blur_variance, operational))
        return sorted(out, key=lambda item: item.score)

    def rank_altitudes(self, conditions: SelectionConditions) -> list[SelectionScore]:
        out=[]
        for altitude in map(float,self.declaration["altitudes_m"]):
            # Two-way radiometric attenuation means the inverse-information
            # penalty grows exponentially with optical path length.
            optical_loss = 0.01 * (math.exp(min(
                2*conditions.optical_attenuation_m_inv*altitude, 20.0
            )) - 1.0)
            risk = conditions.collision_risk / max(altitude, 0.1)
            swath_loss = (3.0-altitude)/3.0
            score = optical_loss + risk + 0.25*swath_loss
            out.append(SelectionScore(f"altitude:{altitude:.1f}",score,-optical_loss, risk+0.25*swath_loss))
        return sorted(out,key=lambda item:item.score)

    def reachable_mission_action(self, conditions: SelectionConditions) -> str:
        if conditions.total_blackout_s >= 30.0:
            return "surface_for_gps"
        if conditions.estimator_drift_m >= 5.0:
            return "abort_leg"
        if conditions.estimator_drift_m >= 2.0:
            return "return_to_last_good_fix"
        if conditions.fix_expected_s > 0.0 and conditions.fix_expected_s <= 10.0:
            return "hold_for_fix"
        return "continue"
