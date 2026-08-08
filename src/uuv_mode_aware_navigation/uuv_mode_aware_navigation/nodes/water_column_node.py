#!/usr/bin/env python3
"""Applies the water column to the Gazebo camera feed.

Gazebo renders a clear-water scene. This node degrades it through the same
propagation model the campaign uses -- attenuation over the two-way path,
forward-scattering blur, and common-volume backscatter veiling -- at the current
turbidity and vehicle altitude, and republishes the result.

Doing it here rather than in the renderer is deliberate. Gazebo's fog is a
rendering effect with no defensible relationship to a beam attenuation
coefficient, so a study that varied fog density would be reporting the
behaviour of a graphics setting. Applying ``imaging.apply_water_column`` instead
means the demonstrator and the statistical campaign degrade imagery through one
model, and the number on the screen means the same thing in both.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from ..imaging import apply_water_column
from ..optics import CONFIGURATIONS, WaterState


class WaterColumnNode(Node):
    def __init__(self) -> None:
        super().__init__("water_column")
        self.declare_parameter("turbidity_c", 0.2)
        self.declare_parameter("altitude_m", 3.0)
        self.declare_parameter("channel", "camera_offaxis")
        self._bridge = CvBridge()
        self._channels = {c.name: c for c in CONFIGURATIONS}
        self._rng = np.random.default_rng(20_000_601)

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        self.create_subscription(
            Float32, "/uuv/turbidity_c", self._on_turbidity, 10
        )
        self.create_subscription(Float32, "/uuv/altitude", self._on_altitude, 10)

        #: Radiance, in the units the quality estimator was fitted in. This is
        #: the topic the optical feedback node consumes.
        #:
        #: It cannot be the display image below. One of the estimator's three
        #: features is *absolute* smoothed modulation, which is what carries
        #: transmittance -- adding it raised agreement with the analytic index
        #: from 0.44 to 0.80. Rescaling a frame to fill 8-bit range for viewing
        #: destroys exactly that information, so an estimator fed the display
        #: image would silently lose most of its accuracy. A real system solves
        #: the same problem the same way, by working from calibrated radiance
        #: rather than from display counts.
        self._radiance = self.create_publisher(Image, "/uuv/camera_radiance", 10)
        #: Human-viewable version. Auto-scaled, and for looking at only.
        self._display = self.create_publisher(Image, "/uuv/camera_degraded", 10)
        self._exposure = 0.0
        self.get_logger().info("water column active")

    def _on_turbidity(self, msg: Float32) -> None:
        self.set_parameters(
            [rclpy.parameter.Parameter("turbidity_c", value=float(msg.data))]
        )

    def _on_altitude(self, msg: Float32) -> None:
        self.set_parameters(
            [rclpy.parameter.Parameter("altitude_m", value=float(msg.data))]
        )

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        scene = frame.astype(float) / 255.0

        c = float(self.get_parameter("turbidity_c").value)
        altitude = max(float(self.get_parameter("altitude_m").value), 0.05)
        config = self._channels.get(
            str(self.get_parameter("channel").value), CONFIGURATIONS[1]
        )

        degraded = apply_water_column(
            scene, WaterState(c=c), altitude, config, rng=self._rng
        )

        radiance = self._bridge.cv2_to_imgmsg(
            degraded.astype(np.float32), encoding="32FC1"
        )
        radiance.header = msg.header
        self._radiance.publish(radiance)

        self._publish_display(msg, c, altitude, config)

    #: Extra attenuation of each display band, relative to the modelled
    #: coefficient. OpenCV channel order: blue, green, red.
    #:
    #: Water is not spectrally neutral, and the way it is not is the most
    #: recognisable thing about an underwater image: red is gone within metres
    #: while blue-green carries furthest. Blue is the reference band at 1.0 and
    #: the others lose light relative to it.
    #:
    #: These affect the DISPLAY image only. The radiance image the optical
    #: feedback estimator consumes is single-band and is computed above from its
    #: own untouched conversion, so nothing here can move a reported number.
    #: Treat this as illustrative, not as a calibrated spectral model.
    #: Ordered red, green, blue to match the rgb8 encoding published below.
    #: The first version published bgr8. The maths was right and the picture
    #: still came out orange, because a viewer that reads bgr8 bytes as rgb
    #: swaps the two ends of the spectrum: a correctly blue-green frame arrives
    #: looking like a sunset. Publishing rgb8 removes the ambiguity.
    DISPLAY_BANDS = ((0, 1.85), (1, 1.15), (2, 1.00))

    #: Floor on the per-band transmission used for display.
    #:
    #: Without it the tint is technically right and useless to look at: past
    #: about one attenuation length the red band is down by four orders of
    #: magnitude, the frame saturates to flat blue, and every trace of seabed
    #: structure disappears into one channel. The floor keeps a residual amount
    #: of each band so the picture stays legible as the water closes in, which
    #: is the whole point of showing it. The radiance image the estimator reads
    #: has no such floor and is not affected.
    BAND_FLOOR = 0.10

    #: Time constant of the display auto-exposure, in frames. The first version
    #: normalised each frame to its own peak, which made the image flicker
    #: whenever a bright particle crossed the view: the whole scene rescaled to
    #: one pixel. A slow-adapting reference removes that.
    EXPOSURE_TC = 30.0

    def _publish_display(self, msg: Image, c: float, altitude: float,
                         config) -> None:
        """The human-viewable frame, in colour. Nothing reads this but a screen.

        Built by tinting the propagated image rather than by re-propagating each
        band. That distinction matters and the other way round is wrong: the
        water column adds a backscatter veil that *grows* with the attenuation
        coefficient, so running the red band through it at red's much larger
        coefficient makes red the brightest channel and the image comes out
        orange -- the opposite of what water does. Physically, red is absorbed,
        and light backscattered from red wavelengths is absorbed on the way back
        too, so red loses on both paths.

        So the veil is computed once, at the modelled coefficient, exactly as
        the science path does; each band then carries its own additional
        transmission loss over the two-way path.
        """
        try:
            colour = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception:  # noqa: BLE001 - a mono source is not an error here
            return

        # Half resolution. The propagation runs in NumPy on the CPU, and at full
        # frame it held the viewer to roughly three frames a second against a
        # fifteen-hertz sensor, which reads as a stutter rather than as a dive.
        # Quartering the pixel count is invisible in a viewer window and is the
        # difference between watching a simulation and watching a slideshow.
        # The estimator's image is untouched and still full resolution.
        colour = colour[::2, ::2, :]

        water = WaterState(c=c)
        path = 2.0 * max(altitude, 0.05)
        out = np.zeros(colour.shape, dtype=float)
        for index, factor in self.DISPLAY_BANDS:
            band = colour[:, :, index].astype(float) / 255.0
            propagated = apply_water_column(band, water, altitude, config)
            # Additional loss for this band relative to the reference one.
            tint = max(math.exp(-(factor - 1.0) * c * path), self.BAND_FLOOR)
            out[:, :, index] = propagated * tint

        peak = float(out.max())
        if peak > 0.0:
            self._exposure += (peak - self._exposure) / self.EXPOSURE_TC
        reference = max(self._exposure, 1e-6)
        image = self._bridge.cv2_to_imgmsg(
            np.clip(out / reference * 235.0, 0, 255).astype(np.uint8),
            encoding="rgb8",
        )
        image.header = msg.header
        self._display.publish(image)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaterColumnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
