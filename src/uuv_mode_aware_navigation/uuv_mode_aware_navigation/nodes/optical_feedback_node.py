#!/usr/bin/env python3
"""Estimates navigation-relevant optical quality from the camera image.

This is the node the paper's title refers to. It subscribes to calibrated
radiance, computes three no-reference image statistics, and publishes a quality
index in [0, 1]. It never sees the turbidity, the altitude, or anything else
about the water -- only pixels.

It runs the identical :class:`OpticalFeedback` estimator the headless campaign
uses, loaded from the same fitted coefficients, so the quantity the manager acts
on here and the quantity it acts on in the statistics are the same quantity.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from ..imaging import OpticalFeedback, analyse_image


class OpticalFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("optical_feedback")
        self.declare_parameter("model_path", "")
        self._bridge = CvBridge()

        path = str(self.get_parameter("model_path").value)
        if path and Path(path).is_file():
            self._model = OpticalFeedback.from_dict(
                json.loads(Path(path).read_text())
            )
            self.get_logger().info(f"loaded optical feedback model from {path}")
        else:
            # Refusing to invent coefficients: an unfitted estimator would emit
            # plausible-looking numbers with no relationship to anything.
            raise SystemExit(
                "optical_feedback requires model_path pointing at a fitted "
                "model. Generate one with scripts/fit_optical_feedback.py."
            )

        self.create_subscription(
            Image, "/uuv/camera_radiance", self._on_image, 10
        )
        self._quality = self.create_publisher(Float32, "/uuv/optical_quality", 10)
        self._contrast = self.create_publisher(
            Float32, "/uuv/optical_structure_contrast", 10
        )
        self._snr = self.create_publisher(
            Float32, "/uuv/optical_structure_to_noise", 10
        )

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        features = analyse_image(np.asarray(frame, dtype=float))
        self._quality.publish(
            Float32(data=float(self._model.predict_features(features)))
        )
        self._contrast.publish(Float32(data=float(features.structure_contrast)))
        self._snr.publish(Float32(data=float(features.structure_to_noise)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpticalFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
