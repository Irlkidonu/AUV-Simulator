#!/usr/bin/env python3
"""Capture the demonstrator figure: rendered frames and what the node makes of them.

The paper claims the loop closes on rendered imagery rather than on a synthetic
quality index. That claim is cheap to assert and easy to doubt, so this script
produces the evidence: frames taken from the running demonstrator at several
turbidities, each labelled with the quality the optical-feedback node estimated
from that frame -- from pixels only, with no access to the turbidity that
produced it.

It attaches to a demonstrator that is already running::

    ros2 launch uuv_mode_aware_navigation demo.launch.py headless:=true
    python3 scripts/capture_demonstrator_figure.py --out figures/fig_demonstrator.pdf

and drives ``/vehicle``'s ``turbidity_c`` parameter through the requested
values, waiting between each for the estimate to settle before sampling. The
figure contributes no number to any table; it is an illustration of a pipeline
whose statistics come from the headless campaign.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import rclpy  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402

# Settling time after a turbidity change. The water-column node degrades the
# next frame it receives, but the feedback node's estimate is filtered, so
# sampling immediately would capture a value part-way between the two
# conditions and label the frame with a quality that was never estimated for it.
SETTLE_S = 6.0


def _to_array(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        frame = data.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        return frame[:, :, ::-1] if msg.encoding == "bgr8" else frame
    if msg.encoding == "mono8":
        gray = data.reshape(msg.height, msg.step)[:, : msg.width]
        return np.repeat(gray[:, :, None], 3, axis=2)
    raise SystemExit(f"unhandled image encoding {msg.encoding!r}")


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("demonstrator_capture")
        self.frame: Image | None = None
        self.quality: float | None = None
        self.channel: str | None = None
        self.create_subscription(Image, "/uuv/camera_degraded",
                                 lambda m: setattr(self, "frame", m), 10)
        self.create_subscription(Float32, "/uuv/optical_quality",
                                 lambda m: setattr(self, "quality", m.data), 10)
        self.create_subscription(String, "/uuv/nav_mode",
                                 lambda m: setattr(self, "channel", m.data), 10)
        self._params = self.create_client(SetParameters,
                                          "/vehicle/set_parameters")

    def set_turbidity(self, value: float) -> None:
        if not self._params.wait_for_service(timeout_sec=10.0):
            raise SystemExit("no /vehicle parameter service: is the demo running?")
        request = SetParameters.Request(parameters=[Parameter(
            name="turbidity_c",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(value)),
        )])
        future = self._params.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

    def sample(self, settle_s: float) -> tuple[np.ndarray, float, str]:
        """Spin for `settle_s`, then return the most recent frame and estimate."""
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.frame is None or self.quality is None:
            raise SystemExit(
                "no frame or no quality estimate arrived; the demonstrator is "
                "running but its optical path is not producing anything"
            )
        return _to_array(self.frame), float(self.quality), self.channel or "?"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--turbidity", type=float, nargs="+",
                    default=[0.2, 0.8, 1.6],
                    help="beam attenuation coefficients to capture, in m^-1")
    ap.add_argument("--settle", type=float, default=SETTLE_S)
    ap.add_argument("--out", type=str,
                    default="figures/"
                            "fig_demonstrator.pdf")
    args = ap.parse_args()

    rclpy.init()
    node = Capture()
    panels = []
    try:
        for c in args.turbidity:
            node.set_turbidity(c)
            frame, quality, mode = node.sample(args.settle)
            print(f"c = {c:.2f} m^-1  ->  estimated quality {quality:.3f}  "
                  f"mode {mode}")
            panels.append((c, frame, quality, mode))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    fig, axes = plt.subplots(1, len(panels), figsize=(3.1 * len(panels), 3.4))
    axes = np.atleast_1d(axes)
    for ax, (c, frame, quality, mode) in zip(axes, panels):
        ax.imshow(frame)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(rf"$c = {c:.1f}\ \mathrm{{m}}^{{-1}}$", fontsize=10)
        ax.set_xlabel(f"estimated quality {quality:.3f}\n{mode}", fontsize=8)
    fig.tight_layout()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
