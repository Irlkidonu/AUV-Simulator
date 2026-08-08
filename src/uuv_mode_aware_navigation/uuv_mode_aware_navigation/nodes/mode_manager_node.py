#!/usr/bin/env python3
"""The mode-aware manager, as a ROS 2 node.

This is a thin wrapper. It marshals ROS topics into an :class:`Observables`
record, calls :meth:`ModeAwareManager.update`, and publishes the decision. All
of the method lives in ``manager.py`` and ``modes.py``, unchanged, so the
demonstrator and the statistical campaign run the same code path.

The information boundary is preserved here exactly as it is in the campaign: the
node subscribes only to quantities a vehicle can measure. There is no
subscription to ground truth, to turbidity, or to the fault schedule, and adding
one would invalidate the study rather than merely the demonstration.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from ..availability import AvailabilityModel
from ..manager import ModeAwareManager
from ..modes import Observables


class ModeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mode_manager")
        self.declare_parameter("availability_model_path", "")
        self.declare_parameter("decision_period_s", 0.5)

        path = str(self.get_parameter("availability_model_path").value)
        availability = None
        if path and Path(path).is_file():
            availability = AvailabilityModel.from_dict(
                json.loads(Path(path).read_text())
            )
            self.get_logger().info(f"loaded availability model from {path}")
        else:
            self.get_logger().warn(
                "no availability model supplied; the manager will fall back to "
                "raw observables, which is ablation A5 and not the proposed method"
            )
        self._manager = ModeAwareManager(availability)

        # --- observables, and nothing else ---
        self._quality = 1.0
        self._optical_available = True
        self._bottom_lock = True
        self._acoustic_age = 0.0
        self._altitude = 3.0
        self._covariance_trace = 0.01
        self._growth_rate = 0.0
        # Fail-closed defaults: until the vehicle says otherwise, assume no
        # water track and no knowledge of the flow.
        self._water_track = False
        self._current_speed = 0.0
        self._current_covariance = 0.0

        self.create_subscription(Float32, "/uuv/optical_quality", self._q, 10)
        self.create_subscription(Bool, "/uuv/optical_available", self._oa, 10)
        self.create_subscription(Bool, "/uuv/dvl_bottom_lock", self._bl, 10)
        self.create_subscription(Float32, "/uuv/acoustic_fix_age", self._aa, 10)
        self.create_subscription(Float32, "/uuv/altitude", self._alt, 10)
        self.create_subscription(
            Float32, "/uuv/position_covariance_trace", self._cov, 10
        )
        self.create_subscription(Bool, "/uuv/dvl_water_track", self._wt, 10)
        self.create_subscription(Float32, "/uuv/current_speed", self._cs, 10)
        self.create_subscription(
            Float32, "/uuv/current_covariance", self._cc, 10
        )

        self._mode_pub = self.create_publisher(String, "/uuv/nav_mode", 10)
        self._config_pub = self.create_publisher(String, "/uuv/configuration", 10)
        self._channel_pub = self.create_publisher(String, "/uuv/optical_channel", 10)
        self._altitude_cmd = self.create_publisher(
            Float32, "/uuv/commanded_altitude", 10
        )
        self._speed_cmd = self.create_publisher(Float32, "/uuv/commanded_speed", 10)
        self._action_pub = self.create_publisher(String, "/uuv/mission_action", 10)
        self._reason_pub = self.create_publisher(String, "/uuv/decision_reason", 10)

        period = float(self.get_parameter("decision_period_s").value)
        self._period = period
        self.create_timer(period, self._decide)
        self.get_logger().info("mode manager active")

    # -- observable callbacks ------------------------------------------------
    def _q(self, m: Float32) -> None:
        self._quality = float(m.data)

    def _oa(self, m: Bool) -> None:
        self._optical_available = bool(m.data)

    def _bl(self, m: Bool) -> None:
        self._bottom_lock = bool(m.data)

    def _aa(self, m: Float32) -> None:
        self._acoustic_age = float(m.data)

    def _alt(self, m: Float32) -> None:
        self._altitude = float(m.data)

    def _wt(self, m: Bool) -> None:
        self._water_track = bool(m.data)

    def _cs(self, m: Float32) -> None:
        self._current_speed = float(m.data)

    def _cc(self, m: Float32) -> None:
        self._current_covariance = float(m.data)

    def _cov(self, m: Float32) -> None:
        previous = self._covariance_trace
        self._covariance_trace = float(m.data)
        self._growth_rate = max(
            (self._covariance_trace - previous) / max(self._period, 1e-6), 0.0
        )

    # -- decision ------------------------------------------------------------
    def _decide(self) -> None:
        observables = Observables(
            optical_quality=self._quality,
            optical_available=self._optical_available,
            dvl_bottom_lock=self._bottom_lock,
            dvl_age_s=0.0 if self._bottom_lock else 5.0,
            acoustic_fix_age_s=self._acoustic_age,
            imu_age_s=0.0,
            depth_age_s=0.0,
            position_covariance_trace=self._covariance_trace,
            covariance_growth_rate=self._growth_rate,
            altitude_m=max(self._altitude, 0.05),
            dvl_water_track=self._water_track,
            current_speed_mps=self._current_speed,
            current_covariance_trace=self._current_covariance,
        )
        decision = self._manager.update(observables, self._period)
        config = decision.configuration

        self._mode_pub.publish(String(data=decision.mode.value))
        self._config_pub.publish(String(data=config.name))
        self._channel_pub.publish(String(data=config.optical.name))
        self._altitude_cmd.publish(Float32(data=float(config.altitude_m)))
        self._speed_cmd.publish(Float32(data=float(config.speed_mps)))
        self._action_pub.publish(String(data=config.mission_action.value))
        self._reason_pub.publish(String(data=decision.reason))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
