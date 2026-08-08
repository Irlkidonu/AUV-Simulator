#!/usr/bin/env python3
"""Swims the fish, and makes them get out of the way.

Constant-velocity fish are not fish. A real one holds a loose station, mills
about, and when something the size of a vehicle comes at it, leaves fast and in
the direction that increases the range quickest. That last behaviour is the one
that reads as alive, and it is the one a static scene cannot fake.

Each fish is a Gazebo model driven through the velocity-control plugin, so this
node has only to decide what each should be doing and publish it:

    cruise   a slow wander, heading drifting, holding a working depth
    flee     within the startle radius: away from the vehicle at burst speed
    recover  after fleeing, coast back down to cruise rather than stopping dead

The startle radius and burst speed are ordinary numbers, not measurements: this
is scenery, not a behavioural model, and nothing in the study depends on it.
What matters is that a driver flying at the school sees it scatter, because that
is what tells them the thing they are flying is in a place rather than in front
of a backdrop.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ..seabed import depth_at

#: Distance at which a fish notices the vehicle and bolts, metres.
STARTLE_M = 3.2
#: Distance beyond which it settles back to cruising.
CALM_M = 5.5
#: Cruise and burst speeds, m/s. A startled reef fish accelerates hard for a
#: second or two; these are chosen to look right rather than measured.
CRUISE_MPS = 0.35
BURST_MPS = 1.9
#: How fast speed converges toward its target. Fleeing is nearly instant;
#: settling back is slow, so a scattered school drifts back rather than snapping.
FLEE_GAIN = 0.55
CALM_GAIN = 0.06

#: Working depth band above the seabed, metres.
MIN_ALT, MAX_ALT = 0.7, 4.0
#: Horizontal bound, metres. Fish turn back rather than leave the survey area.
BOUND_M = 14.0


class Fish:
    def __init__(self, name: str, rng: np.random.Generator) -> None:
        self.name = name
        self.pos = np.array([
            rng.uniform(-11, 11), rng.uniform(-11, 11), 0.0])
        self.pos[2] = depth_at(self.pos[0], self.pos[1]) + rng.uniform(1.0, 3.2)
        self.heading = rng.uniform(0.0, 2.0 * math.pi)
        #: Where the body actually points. The heading above is where the fish
        #: wants to go; the two converge at a finite turn rate.
        self.yaw = self.heading
        self.speed = CRUISE_MPS * rng.uniform(0.7, 1.3)
        self.cruise = CRUISE_MPS * rng.uniform(0.7, 1.3)
        self.turn = rng.uniform(-0.25, 0.25)
        self.vertical = 0.0
        self._rng = rng
        self.fleeing = False

    def update(self, vehicle: np.ndarray, dt: float) -> Twist:
        to_fish = self.pos - vehicle
        to_fish[2] *= 0.6                     # vertical separation matters less
        range_m = float(np.linalg.norm(to_fish))

        if range_m < STARTLE_M:
            self.fleeing = True
        elif range_m > CALM_M:
            self.fleeing = False

        if self.fleeing:
            # Straight away from the vehicle is the direction that opens the
            # range fastest, which is what a startled fish actually does.
            away = math.atan2(to_fish[1], to_fish[0])
            delta = (away - self.heading + math.pi) % (2.0 * math.pi) - math.pi
            self.heading += delta * min(1.0, 6.0 * dt)
            self.speed += (BURST_MPS - self.speed) * FLEE_GAIN
            self.vertical += (0.35 * np.sign(to_fish[2] or 1.0) - self.vertical) * 0.3
        else:
            # Cruising: the heading wanders, so the school does not fly in
            # straight lines forever.
            self.turn += self._rng.normal(0.0, 0.35) * dt
            self.turn = float(np.clip(self.turn, -0.5, 0.5))
            self.heading += self.turn * dt
            self.speed += (self.cruise - self.speed) * CALM_GAIN
            self.vertical += (self._rng.normal(0.0, 0.04) - self.vertical) * 0.1

        # Stay inside the area and inside a sensible depth band.
        if abs(self.pos[0]) > BOUND_M or abs(self.pos[1]) > BOUND_M:
            inward = math.atan2(-self.pos[1], -self.pos[0])
            delta = (inward - self.heading + math.pi) % (2.0 * math.pi) - math.pi
            self.heading += delta * min(1.0, 2.0 * dt)
        altitude = self.pos[2] - depth_at(self.pos[0], self.pos[1])
        if altitude < MIN_ALT:
            self.vertical = abs(self.vertical) + 0.15
        elif altitude > MAX_ALT:
            self.vertical = -abs(self.vertical) - 0.15

        # Steer toward the wanted heading rather than teleporting to it: the
        # velocity plugin applies linear velocity in the MODEL's own frame, so
        # the only way a fish goes where it is aimed is to yaw until its nose
        # points there and then swim forward along it. Commanding a world-frame
        # vector, which is what this did first, makes every fish travel along
        # whatever axis it happened to spawn on and ignore the steering
        # entirely.
        error = (self.heading - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        rate = float(np.clip(error * 2.5, -2.5, 2.5))
        self.yaw += rate * dt

        # Dead-reckon our own belief about where the fish is. Gazebo owns the
        # truth, but reading it back per fish would need a pose subscription
        # each; integrating what we commanded is accurate enough to decide when
        # to bolt, and costs nothing.
        self.pos = self.pos + np.array([
            self.speed * math.cos(self.yaw),
            self.speed * math.sin(self.yaw),
            self.vertical,
        ]) * dt

        msg = Twist()
        msg.linear.x = float(self.speed)     # along its own nose
        msg.linear.z = float(self.vertical)
        msg.angular.z = rate                 # turn to face where it is going
        return msg


class Jelly:
    """A jellyfish. It does not swim like a fish and should not look like one.

    Locomotion is a bell contraction: a short push, then a long passive glide
    during which it sinks slightly. So the vertical velocity is a sharp positive
    pulse decaying to a small negative drift, not a constant. Horizontally it
    barely steers at all; it goes where the water goes, which is why the current
    setting shows up in the school before it shows up anywhere else.
    """

    def __init__(self, index: int, rng: np.random.Generator) -> None:
        self.index = index
        self.period = rng.uniform(2.6, 4.4)       # seconds per contraction
        self.phase = rng.uniform(0.0, self.period)
        self.thrust = rng.uniform(0.16, 0.30)     # peak rise speed, m/s
        self.sink = rng.uniform(0.030, 0.055)     # passive sink between pulses
        self.drift = rng.uniform(0.0, 2.0 * math.pi)
        self.wander = rng.uniform(0.010, 0.035)

    def update(self, dt: float) -> Twist:
        self.phase = (self.phase + dt) % self.period
        f = self.phase / self.period
        # A contraction occupying the first fifth of the cycle, then a glide.
        if f < 0.20:
            rise = self.thrust * math.sin(math.pi * f / 0.20) ** 0.6
        else:
            rise = -self.sink
        self.drift += 0.35 * dt
        msg = Twist()
        msg.linear.x = self.wander * math.cos(self.drift)
        msg.linear.y = self.wander * math.sin(self.drift * 0.7)
        msg.linear.z = rise
        # A slow roll about its own axis, which is what stops a bell looking
        # like a lampshade hanging in the water.
        msg.angular.z = 0.12 * math.sin(self.drift * 0.5)
        return msg


class Weed:
    """A clump of weed leaning at its holdfast.

    The velocity plugin holds whatever angular velocity it is given, so this
    cannot simply command "sway". A constant term of any kind is a constant
    rotation rate, and the first version of this added a current bias that way:
    every plant rotated steadily until it lay flat on the seabed, which looked
    exactly like a bad up-axis and was not.

    So the lean is closed-loop. The node integrates the angle it has commanded,
    compares it against the angle the current and the sway ask for, and sends
    the velocity needed to close the gap. Net rotation over a cycle is zero by
    construction, and the plant returns upright when the current drops.
    """

    #: Radians of lean per m/s of current. A half-knot bed lies over noticeably
    #: without going flat.
    LEAN_PER_MPS = 0.9
    #: How hard the clump returns to the angle being asked for, 1/s.
    STIFFNESS = 1.8
    #: Ceiling on lean, radians. Weed bends; it does not fold over.
    MAX_LEAN = 0.7

    def __init__(self, rng: np.random.Generator) -> None:
        self.period = rng.uniform(4.0, 7.5)
        self.phase = rng.uniform(0.0, 2.0 * math.pi)
        self.amplitude = rng.uniform(0.06, 0.14)   # radians of sway
        self.give = rng.uniform(0.75, 1.3)         # how floppy this clump is
        self.angle = np.zeros(2)                   # integrated lean, x and y

    def update(self, dt: float, t: float, current: np.ndarray) -> Twist:
        sway = self.amplitude * math.sin(2.0 * math.pi * t / self.period + self.phase)
        # Where this clump should be leaning: downstream, plus its own sway.
        target = np.array([
            self.give * self.LEAN_PER_MPS * float(current[1]) + sway,
            -self.give * self.LEAN_PER_MPS * float(current[0]) + sway * 0.6,
        ])
        # Clamp how far the clump leans, not how far it leans along each axis
        # separately. Clipping the two components independently lets a diagonal
        # current tilt it by MAX_LEAN on both at once, which is root two times
        # the ceiling: measured at 0.688 and -0.684 rad under a 1.62 m/s flow,
        # a 0.97 rad lean from a limit that says 0.7. The weeds' visual lifts in
        # the world file are computed at MAX_LEAN, so a lean past it puts the
        # lower leaves back under the seabed, which is the thing the lift exists
        # to prevent.
        lean = float(np.linalg.norm(target))
        if lean > self.MAX_LEAN:
            target = target * (self.MAX_LEAN / lean)
        rate = (target - self.angle) * self.STIFFNESS
        self.angle = self.angle + rate * dt        # integrate what we commanded

        msg = Twist()
        msg.angular.x = float(rate[0])
        msg.angular.y = float(rate[1])
        return msg


class FishSchool(Node):
    def __init__(self) -> None:
        super().__init__("fish_school")
        self.declare_parameter("count", 10)
        self.declare_parameter("rate_hz", 10.0)
        count = int(self.get_parameter("count").value)
        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / rate

        rng = np.random.default_rng(4242)
        self._fish = [Fish(f"fish_{i}", rng) for i in range(count)]
        self._pubs = [
            self.create_publisher(Twist, f"/model/fish_{i}/cmd_vel", 10)
            for i in range(count)
        ]
        self.declare_parameter("jellies", 12)
        jellies = int(self.get_parameter("jellies").value)
        self._jelly = [Jelly(i, rng) for i in range(jellies)]
        self._jelly_pubs = [
            self.create_publisher(Twist, f"/model/jelly_{i}/cmd_vel", 10)
            for i in range(jellies)
        ]
        self.declare_parameter("weeds", 24)
        weeds = int(self.get_parameter("weeds").value)
        self._weed = [Weed(rng) for _ in range(weeds)]
        self._weed_pubs = [
            self.create_publisher(Twist, f"/model/weed_{i}/cmd_vel", 10)
            for i in range(weeds)
        ]
        self._current = np.zeros(3)
        self._elapsed = 0.0
        self.create_subscription(
            Vector3, "/uuv/set_current",
            lambda m: setattr(self, "_current",
                              np.array([m.x, m.y, m.z], dtype=float)), 10)
        self._vehicle = np.array([-10.0, -9.0, -17.0])
        self.create_subscription(
            Vector3, "/uuv/true_position", self._on_vehicle, 10)
        self.create_timer(self._dt, self._step)
        self.get_logger().info(
            f"{count} fish (startle {STARTLE_M} m), {jellies} jellyfish, "
            f"{weeds} weed clumps swaying with the current")

    def _on_vehicle(self, msg: Vector3) -> None:
        self._vehicle = np.array([msg.x, msg.y, msg.z], dtype=float)

    def _step(self) -> None:
        for fish, pub in zip(self._fish, self._pubs):
            pub.publish(fish.update(self._vehicle, self._dt))
        for jelly, pub in zip(self._jelly, self._jelly_pubs):
            pub.publish(jelly.update(self._dt))
        self._elapsed += self._dt
        for weed, pub in zip(self._weed, self._weed_pubs):
            pub.publish(weed.update(self._dt, self._elapsed, self._current))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FishSchool()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
